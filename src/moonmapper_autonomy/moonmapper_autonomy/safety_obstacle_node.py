"""Gate cmd_vel: stop forward motion if obstacle in front laser sector."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class SafetyObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_obstacle_node")

        self.declare_parameter("front_stop_distance", 0.35)
        self.declare_parameter("emergency_stop_distance_m", 0.45)
        self.declare_parameter("slow_distance_m", 1.0)
        self.declare_parameter("slow_linear_speed", 0.06)
        self.declare_parameter("safe_turn_speed", 0.25)
        self.declare_parameter("front_angle_deg", 55.0)
        self.declare_parameter("side_angle_deg", 75.0)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("allow_reverse_when_blocked", True)
        self.declare_parameter("reverse_speed_when_blocked", 0.12)
        self.declare_parameter("publish_safety_debug", True)
        self.declare_parameter("debug_log_period_sec", 2.0)

        self._front_stop = float(self.get_parameter("front_stop_distance").value)
        self._emergency_m = float(self.get_parameter("emergency_stop_distance_m").value)
        self._slow_m = float(self.get_parameter("slow_distance_m").value)
        self._slow_lin = float(self.get_parameter("slow_linear_speed").value)
        self._safe_turn = float(self.get_parameter("safe_turn_speed").value)
        self._front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self._side_angle_deg = float(self.get_parameter("side_angle_deg").value)
        self._input_topic = str(self.get_parameter("input_cmd_topic").value)
        self._output_topic = str(self.get_parameter("output_cmd_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self._allow_rev = bool(
            self.get_parameter("allow_reverse_when_blocked").value
        )
        self._reverse_speed = abs(
            float(self.get_parameter("reverse_speed_when_blocked").value)
        )
        self._pub_debug = bool(self.get_parameter("publish_safety_debug").value)
        self._debug_period = float(self.get_parameter("debug_log_period_sec").value)

        self._last_cmd: Optional[Twist] = None
        self._last_scan: Optional[LaserScan] = None

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._pub_blocked = self.create_publisher(Bool, "/safety/blocked_front", 10)
        self._pub_front_min = self.create_publisher(Float32, "/safety/front_min", 10)
        self._pub_obstacle_front = self.create_publisher(Float32, "/obstacle/front_min", 10)
        self._pub_obstacle_state = self.create_publisher(String, "/obstacle/current_state", 10)
        self.create_subscription(Twist, self._input_topic, self._on_cmd, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_timer(0.05, self._publish_safe)
        self._last_debug_log: Optional[Time] = None

        self.get_logger().info(
            f"safety_obstacle_node: {self._input_topic} + {self._scan_topic} -> "
            f"{self._output_topic} (front {self._front_angle_deg} deg, "
            f"emergency<{self._emergency_m}m slow<{self._slow_m}m)"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._publish_safe()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._publish_safe()

    def _scan_fresh(self) -> bool:
        if self._last_scan is None:
            return False
        stamp = Time.from_msg(self._last_scan.header.stamp)
        age = self.get_clock().now() - stamp
        return age <= Duration(seconds=self._scan_timeout)

    def _min_valid_range_in_cone(self, scan: LaserScan, half_angle_deg: float) -> Optional[float]:
        half_rad = math.radians(half_angle_deg)
        best: Optional[float] = None
        n = len(scan.ranges)
        for i in range(n):
            a = scan.angle_min + float(i) * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) > half_rad:
                continue
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            best = r if best is None else min(best, r)
        return best

    def _copy_raw_twist(self, raw: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(raw.linear.x)
        out.linear.y = float(raw.linear.y)
        out.linear.z = float(raw.linear.z)
        out.angular.x = float(raw.angular.x)
        out.angular.y = float(raw.angular.y)
        out.angular.z = float(raw.angular.z)
        return out

    def _maybe_log_debug(
        self,
        raw: Twist | None,
        safe: Twist,
        min_front: Optional[float],
        state: str,
    ) -> None:
        if self._debug_period <= 0.0:
            return
        now = self.get_clock().now()
        if (
            self._last_debug_log is not None
            and (now - self._last_debug_log).nanoseconds < int(self._debug_period * 1e9)
        ):
            return
        self._last_debug_log = now
        rlx = float(raw.linear.x) if raw is not None else float("nan")
        raz = float(raw.angular.z) if raw is not None else float("nan")
        fm = float("nan")
        if min_front is not None and not (math.isnan(min_front) or math.isinf(min_front)):
            fm = float(min_front)
        self.get_logger().info(
            f"safety: state={state} raw_lin={rlx:.4f} raw_ang={raz:.4f} "
            f"safe_lin={safe.linear.x:.4f} safe_ang={safe.angular.z:.4f} front_min={fm} "
        )

    def _publish_safe(self) -> None:
        if self._last_cmd is None:
            return

        stop = Twist()

        def pub_debug(blocked: bool, min_front: Optional[float], state: str) -> None:
            fm = Float32()
            if min_front is None or (
                isinstance(min_front, float) and (math.isnan(min_front) or math.isinf(min_front))
            ):
                fm.data = float("nan")
            else:
                fm.data = float(min_front)

            ost = String()
            if self._last_scan is None or not self._scan_fresh():
                ost.data = "no_scan"
            else:
                ost.data = state
            self._pub_obstacle_front.publish(fm)
            self._pub_obstacle_state.publish(ost)

            if not self._pub_debug:
                return
            b = Bool()
            b.data = blocked
            self._pub_blocked.publish(b)
            self._pub_front_min.publish(fm)

        if self._last_scan is None or not self._scan_fresh():
            # Pass through Nav2 cmd when scan is missing/stale (do not zero /cmd_vel).
            passthrough = self._copy_raw_twist(self._last_cmd)
            pub_debug(False, None, "no_scan")
            self._pub.publish(passthrough)
            self._maybe_log_debug(self._last_cmd, passthrough, None, "no_scan")
            return

        min_front = self._min_valid_range_in_cone(self._last_scan, self._front_angle_deg)
        _ = self._min_valid_range_in_cone(self._last_scan, self._side_angle_deg)
        emergency = (
            min_front is not None and min_front < self._emergency_m
        )
        blocked = min_front is not None and min_front < self._front_stop
        pub_debug(blocked or emergency, min_front, "clear")

        raw = self._last_cmd
        safe = self._copy_raw_twist(raw)
        state = "clear"

        if min_front is not None and min_front < self._slow_m:
            lx = float(raw.linear.x)
            if emergency or blocked:
                state = "stop_turn"
                if lx > 0.0:
                    safe.linear.x = (
                        -self._reverse_speed if self._allow_rev else 0.0
                    )
                else:
                    safe.linear.x = 0.0
                if abs(safe.angular.z) < 0.05:
                    safe.angular.z = self._safe_turn if lx >= 0.0 else -self._safe_turn
                else:
                    safe.angular.z = math.copysign(
                        min(abs(safe.angular.z), self._safe_turn), safe.angular.z
                    )
            else:
                state = "slow"
                if lx > 0.0:
                    safe.linear.x = min(lx, self._slow_lin)
                pub_debug(True, min_front, state)
        elif blocked:
            state = "stop_turn"
            lx = float(raw.linear.x)
            if lx > 0.0:
                safe.linear.x = -self._reverse_speed if self._allow_rev else 0.0
            else:
                safe.linear.x = 0.0

        if state == "clear":
            pub_debug(blocked, min_front, state)

        self._pub.publish(safe)
        self._maybe_log_debug(raw, safe, min_front, state)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetyObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
