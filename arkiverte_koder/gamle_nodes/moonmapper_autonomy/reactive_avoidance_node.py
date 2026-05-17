"""Reaktiv unnamanøver: /scan -> /cmd_vel_raw (låst svingretning per unnamanøver).

Debug:
  /obstacle/front_min|left_min|right_min (Float32): NaN = ingen gyldig måling.
  /obstacle/current_state (std_msgs/String)
  /obstacle/active_turn_direction (std_msgs/String): left|right|none

Parametern max_escape_attempts_same_direction: 0 = aldri bytt retning etter
max_turn-timeout (kun BACKUP + samme sving). >0 = gammel oppførsel (bytt etter N runder).
"""

from __future__ import annotations

import math
import random
from collections import deque
from typing import Deque, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

STOP_NO_SCAN = "STOP_NO_SCAN"
FORWARD = "FORWARD"
SLOW_FORWARD = "SLOW_FORWARD"
STOP_BEFORE_TURN = "STOP_BEFORE_TURN"
BACKUP = "BACKUP"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
ESCAPE_FORWARD = "ESCAPE_FORWARD"


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def valid_distance(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value) and float(value) > 0.0


def _median(vals: List[float]) -> float:
    if not vals:
        return float("inf")
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


class ReactiveAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("reactive_avoidance_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("forward_speed", 0.08)
        self.declare_parameter("slow_speed", 0.04)
        self.declare_parameter("reverse_speed", -0.08)
        self.declare_parameter("turn_speed", 0.08)
        self.declare_parameter("slow_distance", 1.45)
        self.declare_parameter("avoid_distance", 0.90)
        self.declare_parameter("emergency_distance", 0.50)
        self.declare_parameter("clear_distance", 0.95)
        self.declare_parameter("clear_confirm_count", 3)
        self.declare_parameter("stop_before_turn_time_sec", 0.3)
        self.declare_parameter("backup_time_sec", 1.0)
        self.declare_parameter("min_turn_time_sec", 1.4)
        self.declare_parameter("max_turn_time_sec", 6.5)
        # 0 = aldri bytt svingretning ved timeout (kun BACKUP + samme retning; mindre ping-pong).
        self.declare_parameter("max_escape_attempts_same_direction", 0)
        self.declare_parameter("default_turn_direction", "right")
        self.declare_parameter("escape_forward_speed", 0.06)
        self.declare_parameter("escape_forward_time_sec", 1.0)
        self.declare_parameter("front_angle_deg", 45.0)
        self.declare_parameter("side_angle_deg", 75.0)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("front_smooth_samples", 5)
        self.declare_parameter("publish_obstacle_debug", True)

        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._out_topic = str(self.get_parameter("output_cmd_topic").value)
        self._v_fwd = float(self.get_parameter("forward_speed").value)
        self._v_slow = float(self.get_parameter("slow_speed").value)
        self._v_rev = float(self.get_parameter("reverse_speed").value)
        self._w_turn = float(self.get_parameter("turn_speed").value)
        self._slow_d = float(self.get_parameter("slow_distance").value)
        self._avoid_d = float(self.get_parameter("avoid_distance").value)
        self._emerg_d = float(self.get_parameter("emergency_distance").value)
        self._clear_d = float(self.get_parameter("clear_distance").value)
        self._clear_confirm = max(
            1, int(self.get_parameter("clear_confirm_count").value)
        )
        self._t_stop = float(self.get_parameter("stop_before_turn_time_sec").value)
        self._t_backup = float(self.get_parameter("backup_time_sec").value)
        self._t_turn_min = float(self.get_parameter("min_turn_time_sec").value)
        self._t_turn_max = float(self.get_parameter("max_turn_time_sec").value)
        self._max_same_dir = max(
            0, int(self.get_parameter("max_escape_attempts_same_direction").value)
        )
        self._default_turn = str(
            self.get_parameter("default_turn_direction").value
        ).strip().lower()
        self._v_escape = float(self.get_parameter("escape_forward_speed").value)
        self._t_escape = float(self.get_parameter("escape_forward_time_sec").value)
        self._front_ang = float(self.get_parameter("front_angle_deg").value)
        self._side_ang = float(self.get_parameter("side_angle_deg").value)
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        sm = int(self.get_parameter("front_smooth_samples").value)
        self._smooth_n = max(1, sm)  # høyere = roligere front_min (færre falske hinder)
        self._pub_debug_en = bool(
            self.get_parameter("publish_obstacle_debug").value
        )

        hz = float(self.get_parameter("control_rate_hz").value)
        if hz <= 0.0:
            hz = 10.0

        self._fr = math.radians(self._front_ang)
        outer_deg = max(self._side_ang, self._front_ang + 1.0)
        self._outer = math.radians(outer_deg)

        self._last_scan: Optional[LaserScan] = None
        self._state = STOP_NO_SCAN
        self._state_since: Time = self.get_clock().now()
        self._front_hist: Deque[float] = deque(maxlen=self._smooth_n)
        self._next_unknown_left = True

        self._active_turn: Optional[str] = None
        self._clear_streak = 0
        self._same_dir_timeouts = 0

        self._pub = self.create_publisher(Twist, self._out_topic, 10)
        self._pub_f = self.create_publisher(Float32, "/obstacle/front_min", 10)
        self._pub_l = self.create_publisher(Float32, "/obstacle/left_min", 10)
        self._pub_r = self.create_publisher(Float32, "/obstacle/right_min", 10)
        self._pub_state = self.create_publisher(String, "/obstacle/current_state", 10)
        self._pub_active = self.create_publisher(
            String, "/obstacle/active_turn_direction", 10
        )

        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self.create_timer(1.0 / hz, self._control_tick)

        self.get_logger().info(
            f"reactive_avoidance_node: {self._scan_topic} -> {self._out_topic} "
            f"({hz:.1f} Hz, locked-turn escape, emerg<{self._emerg_d} m)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _scan_ok(self) -> bool:
        if self._last_scan is None:
            return False
        stamp = Time.from_msg(self._last_scan.header.stamp)
        age = self.get_clock().now() - stamp
        return age <= Duration(seconds=self._scan_timeout)

    def _min_in_arc(self, scan: LaserScan, lo: float, hi: float) -> Optional[float]:
        best: Optional[float] = None
        n = len(scan.ranges)
        for i in range(n):
            a = _norm_angle(scan.angle_min + float(i) * scan.angle_increment)
            if not (lo < a <= hi):
                continue
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            best = r if best is None else min(best, r)
        return best

    def _min_front(self, scan: LaserScan) -> Optional[float]:
        best: Optional[float] = None
        n = len(scan.ranges)
        for i in range(n):
            a = _norm_angle(scan.angle_min + float(i) * scan.angle_increment)
            if abs(a) > self._fr:
                continue
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            best = r if best is None else min(best, r)
        return best

    def _min_left(self, scan: LaserScan) -> Optional[float]:
        return self._min_in_arc(scan, self._fr, self._outer)

    def _min_right(self, scan: LaserScan) -> Optional[float]:
        return self._min_in_arc(scan, -self._outer, -self._fr)

    def _state_age_sec(self, now: Time) -> float:
        return (now - self._state_since).nanoseconds * 1e-9

    def _set_state(self, s: str, now: Time) -> None:
        if s != self._state:
            self.get_logger().info(f"tilstand: {s}")
            self._state = s
            self._state_since = now
            if s in (TURN_LEFT, TURN_RIGHT):
                self._clear_streak = 0

    def _lock_avoidance_turn(
        self, left_m: Optional[float], right_m: Optional[float]
    ) -> None:
        vl = valid_distance(left_m)
        vr = valid_distance(right_m)
        if vl and vr and left_m is not None and right_m is not None:
            self._active_turn = "left" if left_m >= right_m else "right"
        elif vl and not vr:
            self._active_turn = "left"
        elif vr and not vl:
            self._active_turn = "right"
        elif self._default_turn == "left":
            self._active_turn = "left"
        elif self._default_turn == "right":
            self._active_turn = "right"
        elif self._default_turn == "random":
            self._active_turn = "left" if random.choice((True, False)) else "right"
        else:
            self._active_turn = "left" if self._next_unknown_left else "right"
            self._next_unknown_left = not self._next_unknown_left
        self._same_dir_timeouts = 0
        self.get_logger().info(f"unnamanøver: låst retning={self._active_turn}")

    def _turn_state_for_active(self) -> str:
        assert self._active_turn in ("left", "right")
        return TURN_LEFT if self._active_turn == "left" else TURN_RIGHT

    def _clear_cruise_turn_lock(self) -> None:
        self._active_turn = None
        self._same_dir_timeouts = 0
        self._clear_streak = 0

    def _front_decision(self, front_raw: Optional[float]) -> float:
        if self._front_hist:
            return _median(list(self._front_hist))
        if valid_distance(front_raw):
            return float(front_raw)
        return float("inf")

    def _pub_debug_mins(
        self,
        front: Optional[float],
        left: Optional[float],
        right: Optional[float],
    ) -> None:
        def pack(x: Optional[float]) -> Float32:
            m = Float32()
            if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
                m.data = float("nan")
            else:
                m.data = float(x)
            return m

        if self._pub_debug_en:
            self._pub_f.publish(pack(front))
            self._pub_l.publish(pack(left))
            self._pub_r.publish(pack(right))

    def _publish_state_str(self, s: str) -> None:
        st = String()
        st.data = s
        self._pub_state.publish(st)

    def _publish_active_turn(self) -> None:
        st = String()
        st.data = self._active_turn if self._active_turn else "none"
        self._pub_active.publish(st)

    def _enter_stop_before_turn(
        self, left_m: Optional[float], right_m: Optional[float], now: Time
    ) -> None:
        self._lock_avoidance_turn(left_m, right_m)
        self._set_state(STOP_BEFORE_TURN, now)

    def _update_fsm(
        self,
        f: float,
        left_m: Optional[float],
        right_m: Optional[float],
        now: Time,
    ) -> None:
        st = self._state
        age = self._state_age_sec(now)

        if st == STOP_NO_SCAN:
            self._set_state(FORWARD, now)
            self._clear_cruise_turn_lock()
            return

        if st == FORWARD:
            if f < self._avoid_d:
                self._enter_stop_before_turn(left_m, right_m, now)
            elif f < self._slow_d:
                self._set_state(SLOW_FORWARD, now)
            return

        if st == SLOW_FORWARD:
            if f < self._avoid_d:
                self._enter_stop_before_turn(left_m, right_m, now)
            elif f > self._slow_d:
                self._set_state(FORWARD, now)
            return

        if st == STOP_BEFORE_TURN:
            if age >= self._t_stop:
                self._set_state(BACKUP, now)
            return

        if st == BACKUP:
            if age >= self._t_backup:
                self._set_state(self._turn_state_for_active(), now)
            return

        if st in (TURN_LEFT, TURN_RIGHT):
            if f > self._clear_d:
                self._clear_streak += 1
            else:
                self._clear_streak = 0

            can_leave_turn = (
                age >= self._t_turn_min
                and self._clear_streak >= self._clear_confirm
            )
            if can_leave_turn:
                self._set_state(ESCAPE_FORWARD, now)
                return

            if age >= self._t_turn_max:
                if self._max_same_dir > 0:
                    self._same_dir_timeouts += 1
                    if self._same_dir_timeouts >= self._max_same_dir:
                        self._active_turn = (
                            "right" if self._active_turn == "left" else "left"
                        )
                        self._same_dir_timeouts = 0
                        self.get_logger().info(
                            f"unnamanøver: bytter låst retning til {self._active_turn}"
                        )
                self._set_state(BACKUP, now)
            return

        if st == ESCAPE_FORWARD:
            if f < self._avoid_d:
                self._enter_stop_before_turn(left_m, right_m, now)
                return
            if age >= self._t_escape:
                self._set_state(FORWARD, now)
                self._clear_cruise_turn_lock()

    def _fill_cmd(self, out: Twist) -> None:
        st = self._state
        if st == FORWARD:
            out.linear.x = self._v_fwd
        elif st == SLOW_FORWARD:
            out.linear.x = self._v_slow
        elif st == STOP_BEFORE_TURN:
            pass
        elif st == BACKUP:
            out.linear.x = self._v_rev
        elif st == TURN_LEFT:
            out.angular.z = self._w_turn
        elif st == TURN_RIGHT:
            out.angular.z = -self._w_turn
        elif st == ESCAPE_FORWARD:
            out.linear.x = self._v_escape

    def _control_tick(self) -> None:
        out = Twist()
        now = self.get_clock().now()

        if self._last_scan is None or not self._scan_ok():
            self._front_hist.clear()
            self._set_state(STOP_NO_SCAN, now)
            self._clear_cruise_turn_lock()
            self._publish_state_str(STOP_NO_SCAN)
            self._publish_active_turn()
            self._pub_debug_mins(None, None, None)
            self._pub.publish(out)
            return

        scan = self._last_scan
        front_raw = self._min_front(scan)
        left_m = self._min_left(scan)
        right_m = self._min_right(scan)
        self._pub_debug_mins(front_raw, left_m, right_m)

        if valid_distance(front_raw):
            self._front_hist.append(float(front_raw))

        f = self._front_decision(front_raw)
        self._update_fsm(f, left_m, right_m, now)
        self._fill_cmd(out)

        self._publish_state_str(self._state)
        self._publish_active_turn()
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ReactiveAvoidanceNode()
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
