#!/usr/bin/env python3
"""Coverage / lawnmower: generer sikksakk over rektangel, publiser sekvensielt til /goal_pose."""

from __future__ import annotations

import math
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


class CoverageMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_mission_node")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("goal_state_topic", "/goal_follower/state")
        self.declare_parameter("distance_topic", "/goal_follower/distance_to_goal")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("loop", False)

        self.declare_parameter("area_min_x", -4.0)
        self.declare_parameter("area_max_x", -1.0)
        self.declare_parameter("area_min_y", -0.8)
        self.declare_parameter("area_max_y", 0.8)
        self.declare_parameter("lane_spacing", 0.5)
        self.declare_parameter("start_from_min_y", True)
        self.declare_parameter("start_from_min_x", False)

        self.declare_parameter("yaw_forward", 3.1416)
        self.declare_parameter("yaw_backward", 0.0)

        self.declare_parameter("wait_after_reached_sec", 0.8)
        self.declare_parameter("goal_publish_period_sec", 1.0)
        self.declare_parameter("mission_rate_hz", 10.0)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("max_retries_per_waypoint", 1)

        self.declare_parameter("reached_state_name", "REACHED")
        self.declare_parameter("idle_state_name", "IDLE")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("coverage_reached_tolerance", 0.18)

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self._goal_frame = self.get_parameter("goal_frame").get_parameter_value().string_value
        self._state_topic = self.get_parameter("goal_state_topic").get_parameter_value().string_value
        self._dist_topic = self.get_parameter("distance_topic").get_parameter_value().string_value
        self._auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self._loop = self.get_parameter("loop").get_parameter_value().bool_value

        self._area_min_x = float(self.get_parameter("area_min_x").value)
        self._area_max_x = float(self.get_parameter("area_max_x").value)
        self._area_min_y = float(self.get_parameter("area_min_y").value)
        self._area_max_y = float(self.get_parameter("area_max_y").value)
        self._lane_spacing = float(self.get_parameter("lane_spacing").value)
        self._start_from_min_y = self.get_parameter("start_from_min_y").get_parameter_value().bool_value
        self._start_from_min_x = self.get_parameter("start_from_min_x").get_parameter_value().bool_value

        self._yaw_forward = float(self.get_parameter("yaw_forward").value)
        self._yaw_backward = float(self.get_parameter("yaw_backward").value)

        self._wait_after = float(self.get_parameter("wait_after_reached_sec").value)
        self._goal_period = float(self.get_parameter("goal_publish_period_sec").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._max_retries = int(self.get_parameter("max_retries_per_waypoint").value)
        self._reached_name = self.get_parameter("reached_state_name").get_parameter_value().string_value
        self._debug_pub_enabled = self.get_parameter("publish_debug").get_parameter_value().bool_value

        self._reach_dist_tol = float(self.get_parameter("coverage_reached_tolerance").value)

        rate = float(self.get_parameter("mission_rate_hz").value)
        self._period = 1.0 / rate if rate > 1e-6 else 0.1

        self._mission_state = "IDLE"
        self._waypoints: list[dict[str, Any]] = []
        self._wp_index = 0
        self._timeout_retries = 0
        self._phase_start = self.get_clock().now()
        self._last_goal_pub = self.get_clock().now()
        self._wait_deadline = self.get_clock().now()

        self._follower_state: str = ""
        self._follower_distance: float = 0.0
        self._reached_armed = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_goal = self.create_publisher(PoseStamped, self._goal_topic, qos)
        self._pub_path = self.create_publisher(Path, "/coverage_mission/generated_path", qos)
        self._pub_state = self.create_publisher(String, "/coverage_mission/state", 10)
        self._pub_idx = self.create_publisher(Int32, "/coverage_mission/current_index", 10)
        self._pub_cnt = self.create_publisher(Int32, "/coverage_mission/waypoint_count", 10)
        self._pub_active = self.create_publisher(Bool, "/coverage_mission/active", 10)

        self.create_subscription(String, self._state_topic, self._on_follower_state, 10)
        self.create_subscription(Float32, self._dist_topic, self._on_follower_dist, 10)
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"coverage_mission_node: auto_start={self._auto_start} loop={self._loop} "
            f"område x=[{self._area_min_x},{self._area_max_x}] y=[{self._area_min_y},{self._area_max_y}] "
            f"lane_spacing={self._lane_spacing}"
        )

    def _goto_wait_after_reached(self) -> None:
        if self._mission_state != "WAIT_FOR_REACHED":
            return
        self._mission_state = "WAIT_AFTER_REACHED"
        self._reset_phase()
        self._publish_debug()

    def _on_follower_state(self, msg: String) -> None:
        s = msg.data.strip()
        self._follower_state = s
        if self._mission_state == "WAIT_FOR_REACHED":
            if s != self._reached_name:
                self._reached_armed = True
            elif self._reached_armed and s == self._reached_name:
                self._goto_wait_after_reached()

    def _on_follower_dist(self, msg: Float32) -> None:
        self._follower_distance = float(msg.data)
        if self._mission_state != "WAIT_FOR_REACHED":
            return
        if self._elapsed() < 0.12:
            return
        if self._follower_distance <= self._reach_dist_tol and self._follower_state in (
            "DRIVE_TO_GOAL",
            "ROTATE_TO_GOAL",
            self._reached_name,
        ):
            self._goto_wait_after_reached()

    def _reset_phase(self) -> None:
        self._phase_start = self.get_clock().now()

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    def _publish_debug(self) -> None:
        if not self._debug_pub_enabled:
            return
        self._pub_state.publish(String(data=self._mission_state))
        self._pub_idx.publish(Int32(data=int(self._wp_index)))
        self._pub_cnt.publish(Int32(data=int(len(self._waypoints))))
        active = self._mission_state not in ("IDLE", "DONE", "ERROR")
        self._pub_active.publish(Bool(data=active))

    def _generate_lawnmower(self) -> list[dict[str, Any]]:
        x_lo = min(self._area_min_x, self._area_max_x)
        x_hi = max(self._area_min_x, self._area_max_x)
        y_lo = min(self._area_min_y, self._area_max_y)
        y_hi = max(self._area_min_y, self._area_max_y)
        spacing = max(self._lane_spacing, 1e-6)

        if abs(x_hi - x_lo) < 1e-6 or abs(y_hi - y_lo) < 1e-6:
            self.get_logger().error("Coverage-område har null utstrekning i x eller y.")
            return []

        ys: list[float] = []
        y = y_lo
        eps = 1e-9
        while y <= y_hi + eps:
            ys.append(float(y))
            y += spacing
        if not self._start_from_min_y:
            ys.reverse()

        pts: list[dict[str, Any]] = []

        def append_pt(px: float, py: float, yaw: float) -> None:
            if pts and abs(pts[-1]["x"] - px) < 1e-6 and abs(pts[-1]["y"] - py) < 1e-6:
                return
            pts.append({"x": float(px), "y": float(py), "yaw": float(yaw)})

        yaw_fwd = self._yaw_forward
        yaw_bwd = self._yaw_backward

        for i, y_lane in enumerate(ys):
            if i % 2 == 0:
                xa, xb = (x_hi, x_lo) if not self._start_from_min_x else (x_lo, x_hi)
            else:
                xa, xb = (x_lo, x_hi) if not self._start_from_min_x else (x_hi, x_lo)

            yaw_h = yaw_fwd if xb < xa else yaw_bwd
            append_pt(xa, y_lane, yaw_h)
            append_pt(xb, y_lane, yaw_h)

            if i + 1 < len(ys):
                y_next = ys[i + 1]
                dy = y_next - y_lane
                yaw_v = math.pi / 2.0 if dy > 0.0 else -math.pi / 2.0
                if abs(xb - x_lo) < 1e-6 or abs(xb - x_hi) < 1e-6:
                    append_pt(xb, y_next, yaw_v)
                else:
                    self.get_logger().error(f"Uventet stripe-slutt x={xb} (forventet x_lo eller x_hi).")
                    return []

        return pts

    def _build_path_msg(self) -> Path:
        out = Path()
        out.header.frame_id = self._goal_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for wp in self._waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self._goal_frame
            ps.header.stamp = out.header.stamp
            ps.pose.position.x = float(wp["x"])
            ps.pose.position.y = float(wp["y"])
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = _yaw_to_quaternion(float(wp["yaw"]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            out.poses.append(ps)
        return out

    def _make_pose(self, wp: dict[str, Any]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self._goal_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(wp["x"])
        msg.pose.position.y = float(wp["y"])
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(float(wp["yaw"]))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_goal(self, wp: dict[str, Any], index_1based: int, total: int) -> None:
        pose = self._make_pose(wp)
        self._pub_goal.publish(pose)
        self._last_goal_pub = self.get_clock().now()
        self.get_logger().info(
            f"Coverage waypoint {index_1based}/{total}: x={wp['x']:.3f} y={wp['y']:.3f} yaw={wp['yaw']:.4f}"
        )

    def _start_send_goal_phase(self, now) -> None:
        if self._wp_index >= len(self._waypoints):
            self._mission_state = "DONE"
            self._publish_debug()
            return
        self._mission_state = "SEND_GOAL"
        self._publish_debug()
        wp = self._waypoints[self._wp_index]
        self._publish_goal(wp, self._wp_index + 1, len(self._waypoints))
        self._timeout_retries = 0
        self._reset_phase()
        self._wait_deadline = now + Duration(seconds=float(self._goal_timeout))
        self._mission_state = "WAIT_FOR_REACHED"
        self._reached_armed = False
        self._publish_debug()

    def _tick(self) -> None:
        self._publish_debug()
        now = self.get_clock().now()

        if self._mission_state == "IDLE":
            if self._auto_start:
                self._mission_state = "GENERATE_PATH"
                self._publish_debug()
            else:
                return

        if self._mission_state == "GENERATE_PATH":
            self._waypoints = self._generate_lawnmower()
            self._pub_path.publish(self._build_path_msg())
            if len(self._waypoints) == 0:
                self.get_logger().error(
                    "Coverage-rute er tom (ugyldig område, lane_spacing eller intern feil)."
                )
                self._mission_state = "ERROR"
                self._publish_debug()
                return
            self.get_logger().info(f"Generert lawnmower-rute med {len(self._waypoints)} mål-punkter.")
            self._wp_index = 0
            self._start_send_goal_phase(now)
            return

        if self._mission_state == "WAIT_FOR_REACHED":
            if self._wp_index >= len(self._waypoints):
                self._mission_state = "DONE"
                self._publish_debug()
                return
            wp = self._waypoints[self._wp_index]
            if (now - self._last_goal_pub).nanoseconds * 1e-9 >= self._goal_period:
                self._publish_goal(wp, self._wp_index + 1, len(self._waypoints))

            if self._reached_armed and self._follower_state == self._reached_name:
                self._goto_wait_after_reached()
                return
            if self._elapsed() >= 0.12 and self._follower_distance <= self._reach_dist_tol:
                if self._follower_state in (
                    "DRIVE_TO_GOAL",
                    "ROTATE_TO_GOAL",
                    self._reached_name,
                ):
                    self._goto_wait_after_reached()
                    return

            if now > self._wait_deadline:
                if self._timeout_retries < self._max_retries:
                    self._timeout_retries += 1
                    self.get_logger().warn(
                        f"Goal timeout — prøver coverage-punkt {self._wp_index + 1} på nytt "
                        f"(forsøk {self._timeout_retries}/{self._max_retries})."
                    )
                    self._publish_goal(wp, self._wp_index + 1, len(self._waypoints))
                    self._wait_deadline = now + Duration(seconds=float(self._goal_timeout))
                    self._reached_armed = False
                else:
                    self.get_logger().error(
                        f"Goal timeout for coverage-punkt {self._wp_index + 1} "
                        f"etter {self._max_retries} omforsøk — misjon avbrutt."
                    )
                    self._mission_state = "ERROR"
                    self._publish_debug()
            return

        if self._mission_state == "WAIT_AFTER_REACHED":
            if self._elapsed() < self._wait_after:
                return
            self._mission_state = "NEXT_GOAL"
            self._publish_debug()

        if self._mission_state == "NEXT_GOAL":
            self._wp_index += 1
            if self._wp_index < len(self._waypoints):
                self._start_send_goal_phase(now)
            elif self._loop and len(self._waypoints) > 0:
                self._wp_index = 0
                self._start_send_goal_phase(now)
            else:
                self._mission_state = "DONE"
                self._publish_debug()
            return


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CoverageMissionNode()
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
