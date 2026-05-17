"""Kombiner goal-kommando med scan-basert detour: bremse, sving unna, bue, recover -> /cmd_vel_raw."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class GoalObstacleAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_obstacle_avoidance_node")

        self.declare_parameter("input_cmd_topic", "/cmd_vel_goal")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("scan_timeout_sec", 0.6)

        self.declare_parameter("front_angle_deg", 45.0)
        self.declare_parameter("side_angle_deg", 85.0)
        self.declare_parameter("avoid_enter_distance", 0.90)
        self.declare_parameter("turn_clear_distance", 1.05)
        self.declare_parameter("avoid_clear_distance", 1.20)
        self.declare_parameter("emergency_distance", 0.40)

        self.declare_parameter("brake_time_sec", 0.4)
        self.declare_parameter("min_turn_time_sec", 1.0)
        self.declare_parameter("max_turn_time_sec", 4.0)
        self.declare_parameter("min_arc_time_sec", 1.5)
        self.declare_parameter("max_arc_time_sec", 5.0)
        self.declare_parameter("recover_time_sec", 0.8)

        self.declare_parameter("turn_speed", 0.50)
        self.declare_parameter("arc_turn_speed", 0.30)
        self.declare_parameter("arc_forward_speed", 0.04)
        self.declare_parameter("cmd_angular_limit", 0.55)
        self.declare_parameter("cmd_linear_limit", 0.12)

        self.declare_parameter("side_hysteresis_sec", 1.2)
        self.declare_parameter("side_switch_margin_m", 0.25)

        self.declare_parameter("prefer_last_turn", True)
        self.declare_parameter("publish_debug", True)

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._in_topic = self.get_parameter("input_cmd_topic").get_parameter_value().string_value
        self._out_topic = self.get_parameter("output_cmd_topic").get_parameter_value().string_value
        self._scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        rate = float(self.get_parameter("control_rate_hz").value)
        self._period = 1.0 / rate if rate > 1e-6 else 0.05
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)

        self._front_deg = float(self.get_parameter("front_angle_deg").value)
        self._side_deg = float(self.get_parameter("side_angle_deg").value)
        self._avoid_enter = float(self.get_parameter("avoid_enter_distance").value)
        self._turn_clear = float(self.get_parameter("turn_clear_distance").value)
        self._avoid_clear = float(self.get_parameter("avoid_clear_distance").value)
        self._emergency = float(self.get_parameter("emergency_distance").value)

        self._brake_time = float(self.get_parameter("brake_time_sec").value)
        self._min_turn = float(self.get_parameter("min_turn_time_sec").value)
        self._max_turn = float(self.get_parameter("max_turn_time_sec").value)
        self._min_arc = float(self.get_parameter("min_arc_time_sec").value)
        self._max_arc = float(self.get_parameter("max_arc_time_sec").value)
        self._recover_time = float(self.get_parameter("recover_time_sec").value)

        self._turn_speed = float(self.get_parameter("turn_speed").value)
        self._arc_turn = float(self.get_parameter("arc_turn_speed").value)
        self._arc_forward = float(self.get_parameter("arc_forward_speed").value)
        self._ang_lim = float(self.get_parameter("cmd_angular_limit").value)
        self._lin_lim = float(self.get_parameter("cmd_linear_limit").value)

        self._prefer_last = bool(self.get_parameter("prefer_last_turn").value)

        self._hyst_sec = float(self.get_parameter("side_hysteresis_sec").value)
        self._switch_margin = float(self.get_parameter("side_switch_margin_m").value)
        self._lock_until: Optional[Time] = None
        self._locked_choice: Optional[str] = None

        self._last_goal: Optional[Twist] = None
        self._last_scan: Optional[LaserScan] = None
        self._state = "CLEAR"
        self._phase_start: Optional[Time] = None
        self._chosen_side: Optional[str] = None  # "LEFT" / "RIGHT"
        self._last_side: Optional[str] = None
        self._force_slow_arc = False
        self._last_logged_state: Optional[str] = None

        self._pub = self.create_publisher(Twist, self._out_topic, 10)
        self._pub_state = self.create_publisher(String, "/goal_avoidance/state", 10)
        self._pub_front = self.create_publisher(Float32, "/goal_avoidance/front_min", 10)
        self._pub_left = self.create_publisher(Float32, "/goal_avoidance/left_min", 10)
        self._pub_right = self.create_publisher(Float32, "/goal_avoidance/right_min", 10)
        self._pub_sel = self.create_publisher(String, "/goal_avoidance/selected_turn", 10)
        self._pub_active = self.create_publisher(Bool, "/goal_avoidance/active", 10)

        self.create_subscription(Twist, self._in_topic, self._on_goal_cmd, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_timer(self._period, self._on_timer)

        self.get_logger().info(
            f"goal_obstacle_avoidance (detour): {self._in_topic} + {self._scan_topic} -> {self._out_topic} "
            f"(publish_debug={bool(self.get_parameter('publish_debug').value)})"
        )

    def _on_goal_cmd(self, msg: Twist) -> None:
        self._last_goal = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _scan_fresh(self) -> bool:
        if self._last_scan is None:
            return False
        stamp = Time.from_msg(self._last_scan.header.stamp)
        age = self.get_clock().now() - stamp
        return age <= Duration(seconds=self._scan_timeout)

    def _sector_mins(self, scan: LaserScan) -> tuple[float, float, float]:
        fh = math.radians(self._front_deg)
        sh = math.radians(self._side_deg)
        best_f = math.inf
        best_l = math.inf
        best_r = math.inf
        n = len(scan.ranges)
        for i in range(n):
            a = scan.angle_min + float(i) * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            if abs(a) <= fh:
                best_f = min(best_f, r)
            elif fh < a <= sh:
                best_l = min(best_l, r)
            elif -sh <= a < -fh:
                best_r = min(best_r, r)
        return best_f, best_l, best_r

    def _log_state(self, new_state: str) -> None:
        if new_state != self._last_logged_state:
            self.get_logger().info(f"state -> {new_state}")
            self._last_logged_state = new_state

    def _set_state(self, new_state: str, now: Time) -> None:
        if new_state == "CLEAR" and self._state != "CLEAR":
            self._lock_until = None
            self._locked_choice = None
        self._state = new_state
        self._phase_start = now
        self._log_state(new_state)

    def _elapsed(self, now: Time) -> float:
        if self._phase_start is None:
            self._phase_start = now
        return (now - self._phase_start).nanoseconds * 1e-9

    def _publish_debug(
        self,
        state: str,
        front: float,
        left: float,
        right: float,
        selected: str,
        active: bool,
    ) -> None:
        self._pub_state.publish(String(data=state))
        self._pub_front.publish(Float32(data=float(front)))
        self._pub_left.publish(Float32(data=float(left)))
        self._pub_right.publish(Float32(data=float(right)))
        self._pub_sel.publish(String(data=selected))
        self._pub_active.publish(Bool(data=active))

    def _choose_side(self, left: float, right: float, now: Time) -> str:
        tie_eps = 0.05
        l = left if math.isfinite(left) else float("inf")
        r = right if math.isfinite(right) else float("inf")

        force_left = l > r + self._switch_margin
        force_right = r > l + self._switch_margin
        if (
            self._lock_until is not None
            and self._locked_choice is not None
            and now < self._lock_until
        ):
            if self._locked_choice == "LEFT" and not force_right:
                return "LEFT"
            if self._locked_choice == "RIGHT" and not force_left:
                return "RIGHT"

        if self._prefer_last and self._last_side is not None and abs(l - r) < tie_eps:
            side = self._last_side
        else:
            side = "LEFT" if l > r else "RIGHT"
        self._last_side = side
        self._locked_choice = side
        self._lock_until = now + Duration(seconds=float(self._hyst_sec))
        return side

    def _pass_goal(self, g: Twist, angular_scale: float = 1.0) -> Twist:
        out = Twist()
        out.linear.x = _clamp(max(0.0, float(g.linear.x)), 0.0, self._lin_lim)
        gz = _clamp(float(g.angular.z), -self._ang_lim, self._ang_lim)
        out.angular.z = gz * float(_clamp(angular_scale, 0.0, 1.0))
        return out

    def _emergency_twist(self, left: float, right: float, now: Time) -> tuple[Twist, str]:
        side = self._choose_side(left, right, now)
        self._last_side = side
        ts = _clamp(abs(self._turn_speed), 0.0, self._ang_lim)
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = ts if side == "LEFT" else -ts
        return t, side

    def _on_timer(self) -> None:
        out = Twist()
        now = self.get_clock().now()
        selected = "NONE"
        active = False

        def fin(x: float) -> float:
            return x if math.isfinite(x) else float("nan")

        if self._last_scan is None or not self._scan_fresh():
            self._state = "NO_SCAN"
            self._phase_start = None
            self._chosen_side = None
            self._force_slow_arc = False
            self._lock_until = None
            self._locked_choice = None
            self._log_state("NO_SCAN")
            self._publish_debug("NO_SCAN", float("nan"), float("nan"), float("nan"), selected, False)
            self._pub.publish(out)
            return

        scan = self._last_scan
        front, left, right = self._sector_mins(scan)
        ff, lf, rf = fin(front), fin(left), fin(right)

        # Nød: ingen fremdrift før front > turn_clear
        if math.isfinite(front) and front < self._emergency:
            etw, selected = self._emergency_twist(left, right, now)
            if self._state != "EMERGENCY":
                self._set_state("EMERGENCY", now)
            active = True
            self._publish_debug("EMERGENCY", ff, lf, rf, selected, active)
            self._pub.publish(etw)
            return

        if self._state == "EMERGENCY":
            if math.isfinite(front) and front > self._turn_clear:
                self._set_state("CLEAR", now)
                self._chosen_side = None
                self._force_slow_arc = False
            else:
                etw, selected = self._emergency_twist(left, right, now)
                active = True
                self._publish_debug("EMERGENCY", ff, lf, rf, selected, active)
                self._pub.publish(etw)
                return

        # RECOVER_TO_GOAL
        if self._state == "RECOVER_TO_GOAL":
            active = True
            te = self._elapsed(now)
            if self._last_goal is None:
                self._set_state("CLEAR", now)
                self._chosen_side = None
                self._publish_debug("CLEAR", ff, lf, rf, "NONE", False)
                self._pub.publish(out)
                return
            alpha = min(1.0, te / self._recover_time) if self._recover_time > 1e-6 else 1.0
            out = self._pass_goal(self._last_goal, angular_scale=alpha)
            if te >= self._recover_time:
                self._set_state("CLEAR", now)
                self._chosen_side = None
                self._force_slow_arc = False
            self._publish_debug("RECOVER_TO_GOAL", ff, lf, rf, self._chosen_side or "NONE", active)
            self._pub.publish(out)
            return

        # BRAKE
        if self._state == "BRAKE":
            active = True
            te = self._elapsed(now)
            if te < self._brake_time:
                self._publish_debug("BRAKE", ff, lf, rf, self._chosen_side or "NONE", active)
                self._pub.publish(out)
                return
            if self._chosen_side == "LEFT":
                self._set_state("TURN_AWAY_LEFT", now)
            else:
                self._set_state("TURN_AWAY_RIGHT", now)
            # fortsett til TURN_* samme tikk

        # TURN_AWAY_*
        if self._state == "TURN_AWAY_LEFT":
            active = True
            selected = "LEFT"
            te = self._elapsed(now)
            out.linear.x = 0.0
            ts = _clamp(abs(self._turn_speed), 0.0, self._ang_lim)
            out.angular.z = ts
            force_slow = te >= self._max_turn
            go_arc = force_slow or (
                te >= self._min_turn and math.isfinite(front) and front > self._turn_clear
            )
            if go_arc:
                self._force_slow_arc = force_slow
                self._set_state("ARC_LEFT", now)
            else:
                self._publish_debug("TURN_AWAY_LEFT", ff, lf, rf, selected, active)
                self._pub.publish(out)
                return

        if self._state == "TURN_AWAY_RIGHT":
            active = True
            selected = "RIGHT"
            te = self._elapsed(now)
            out.linear.x = 0.0
            ts = _clamp(abs(self._turn_speed), 0.0, self._ang_lim)
            out.angular.z = -ts
            force_slow = te >= self._max_turn
            go_arc = force_slow or (
                te >= self._min_turn and math.isfinite(front) and front > self._turn_clear
            )
            if go_arc:
                self._force_slow_arc = force_slow
                self._set_state("ARC_RIGHT", now)
            else:
                self._publish_debug("TURN_AWAY_RIGHT", ff, lf, rf, selected, active)
                self._pub.publish(out)
                return

        # ARC_*
        if self._state == "ARC_LEFT":
            active = True
            selected = "LEFT"
            te = self._elapsed(now)
            v = self._arc_forward * (0.5 if self._force_slow_arc else 1.0)
            out.linear.x = _clamp(v, 0.0, self._lin_lim)
            at = _clamp(abs(self._arc_turn), 0.0, self._ang_lim)
            out.angular.z = at
            done = False
            if te >= self._max_arc:
                done = True
            elif te >= self._min_arc and math.isfinite(front) and front > self._avoid_clear:
                done = True
            if done:
                self._force_slow_arc = False
                self._set_state("RECOVER_TO_GOAL", now)
                te = self._elapsed(now)
                alpha = min(1.0, te / self._recover_time) if self._recover_time > 1e-6 else 1.0
                if self._last_goal is not None:
                    out = self._pass_goal(self._last_goal, angular_scale=alpha)
                else:
                    out = Twist()
                self._publish_debug("RECOVER_TO_GOAL", ff, lf, rf, selected, True)
                self._pub.publish(out)
                return
            self._publish_debug("ARC_LEFT", ff, lf, rf, selected, active)
            self._pub.publish(out)
            return

        if self._state == "ARC_RIGHT":
            active = True
            selected = "RIGHT"
            te = self._elapsed(now)
            v = self._arc_forward * (0.5 if self._force_slow_arc else 1.0)
            out.linear.x = _clamp(v, 0.0, self._lin_lim)
            at = _clamp(abs(self._arc_turn), 0.0, self._ang_lim)
            out.angular.z = -at
            done = False
            if te >= self._max_arc:
                done = True
            elif te >= self._min_arc and math.isfinite(front) and front > self._avoid_clear:
                done = True
            if done:
                self._force_slow_arc = False
                self._set_state("RECOVER_TO_GOAL", now)
                te = self._elapsed(now)
                alpha = min(1.0, te / self._recover_time) if self._recover_time > 1e-6 else 1.0
                if self._last_goal is not None:
                    out = self._pass_goal(self._last_goal, angular_scale=alpha)
                else:
                    out = Twist()
                self._publish_debug("RECOVER_TO_GOAL", ff, lf, rf, selected, True)
                self._pub.publish(out)
                return
            self._publish_debug("ARC_RIGHT", ff, lf, rf, selected, active)
            self._pub.publish(out)
            return

        # CLEAR
        if self._last_goal is None:
            self._state = "CLEAR"
            self._log_state("CLEAR")
            self._publish_debug("CLEAR", ff, lf, rf, "NONE", False)
            self._pub.publish(out)
            return

        g = self._last_goal
        if math.isfinite(front) and front < self._avoid_enter:
            side = self._choose_side(left, right, now)
            self._last_side = side
            self._chosen_side = side
            self._force_slow_arc = False
            self._set_state("BRAKE", now)
            active = True
            selected = side
            self._publish_debug("BRAKE", ff, lf, rf, selected, active)
            self._pub.publish(out)
            return

        self._state = "CLEAR"
        self._log_state("CLEAR")
        out = self._pass_goal(g, angular_scale=1.0)
        self._publish_debug("CLEAR", ff, lf, rf, "NONE", False)
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GoalObstacleAvoidanceNode()
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
