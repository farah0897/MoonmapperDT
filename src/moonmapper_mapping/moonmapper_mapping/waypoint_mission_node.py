#!/usr/bin/env python3
"""Enkel waypoint-misjon: les YAML, publiser /goal_pose sekvensielt, vent på goal_follower REACHED."""

from __future__ import annotations

import math
import pathlib
from typing import Any, Optional

import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


class WaypointMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_mission_node")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("goal_state_topic", "/goal_follower/state")
        self.declare_parameter("distance_topic", "/goal_follower/distance_to_goal")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("loop", False)
        self.declare_parameter("reached_state_name", "REACHED")
        self.declare_parameter("idle_state_name", "IDLE")
        self.declare_parameter("wait_after_reached_sec", 1.0)
        self.declare_parameter("goal_publish_period_sec", 1.0)
        self.declare_parameter("mission_rate_hz", 10.0)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("max_retries_per_waypoint", 1)
        self.declare_parameter("publish_debug", True)

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._wp_file = self.get_parameter("waypoints_file").get_parameter_value().string_value
        self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self._goal_frame = self.get_parameter("goal_frame").get_parameter_value().string_value
        self._state_topic = self.get_parameter("goal_state_topic").get_parameter_value().string_value
        self._dist_topic = self.get_parameter("distance_topic").get_parameter_value().string_value
        self._auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self._loop = self.get_parameter("loop").get_parameter_value().bool_value
        self._reached_name = self.get_parameter("reached_state_name").get_parameter_value().string_value
        self._idle_name = self.get_parameter("idle_state_name").get_parameter_value().string_value
        self._wait_after = float(self.get_parameter("wait_after_reached_sec").value)
        self._goal_period = float(self.get_parameter("goal_publish_period_sec").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._max_retries = int(self.get_parameter("max_retries_per_waypoint").value)
        self._debug_pub_enabled = self.get_parameter("publish_debug").get_parameter_value().bool_value
        # Avstand under ca. goal_follower goal_tolerance_m (0.15) + margin — backup hvis REACHED-tikk mistes.
        self._reach_dist_tol = 0.18

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
        self._pub_mission_state = self.create_publisher(String, "/mission/state", 10)
        self._pub_wp_idx = self.create_publisher(Int32, "/mission/current_waypoint_index", 10)
        self._pub_wp_cnt = self.create_publisher(Int32, "/mission/waypoint_count", 10)
        self._pub_active = self.create_publisher(Bool, "/mission/active", 10)

        self.create_subscription(String, self._state_topic, self._on_follower_state, 10)
        self.create_subscription(Float32, self._dist_topic, self._on_follower_dist, 10)
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"waypoint_mission_node: file='{self._wp_file}' auto_start={self._auto_start} loop={self._loop}"
        )

    def _goto_wait_after_reached(self) -> None:
        if self._mission_state != "WAIT_FOR_REACHED":
            return
        self._mission_state = "WAIT_AFTER_REACHED"
        self._reset_phase()
        self._publish_mission_debug()

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

    def _publish_mission_debug(self) -> None:
        if not self._debug_pub_enabled:
            return
        self._pub_mission_state.publish(String(data=self._mission_state))
        self._pub_wp_idx.publish(Int32(data=int(self._wp_index)))
        self._pub_wp_cnt.publish(Int32(data=int(len(self._waypoints))))
        active = self._mission_state not in ("IDLE", "DONE", "ERROR")
        self._pub_active.publish(Bool(data=active))

    def _load_yaml(self) -> bool:
        path = pathlib.Path(self._wp_file).expanduser()
        if not str(self._wp_file).strip():
            self.get_logger().error("waypoints_file er tom — sett parameter waypoints_file.")
            return False
        if not path.is_file():
            self.get_logger().error(f"Waypoints-fil finnes ikke: {path}")
            return False
        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as ex:
            self.get_logger().error(f"Kunne ikke lese YAML: {ex}")
            return False
        if not isinstance(data, dict) or "waypoints" not in data:
            self.get_logger().error("YAML mangler toppnøkkel 'waypoints' (liste).")
            return False
        wps = data["waypoints"]
        if not isinstance(wps, list) or len(wps) == 0:
            self.get_logger().error("waypoints-liste er tom eller ugyldig.")
            return False
        out: list[dict[str, Any]] = []
        for i, w in enumerate(wps):
            if not isinstance(w, dict):
                self.get_logger().error(f"Waypoint {i} er ikke et mapping.")
                return False
            try:
                name = str(w.get("name", f"wp_{i}"))
                x = float(w["x"])
                y = float(w["y"])
                yaw = float(w.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError) as ex:
                self.get_logger().error(f"Waypoint {i} mangler felt eller har ugyldig tall: {ex}")
                return False
            out.append({"name": name, "x": x, "y": y, "yaw": yaw})
        self._waypoints = out
        self.get_logger().info(f"Lastet {len(out)} waypoints fra {path}")
        return True

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
            f"Sending waypoint {index_1based}/{total}: {wp['name']} "
            f"x={wp['x']:.3f} y={wp['y']:.3f} yaw={wp['yaw']:.4f}"
        )

    def _start_send_goal_phase(self, now) -> None:
        """SEND_GOAL (kort i /mission/state), publiser pose, deretter WAIT_FOR_REACHED."""
        if self._wp_index >= len(self._waypoints):
            self._mission_state = "DONE"
            self._publish_mission_debug()
            return
        self._mission_state = "SEND_GOAL"
        self._publish_mission_debug()
        wp = self._waypoints[self._wp_index]
        self._publish_goal(wp, self._wp_index + 1, len(self._waypoints))
        self._timeout_retries = 0
        self._reset_phase()
        self._wait_deadline = now + Duration(seconds=float(self._goal_timeout))
        self._mission_state = "WAIT_FOR_REACHED"
        self._reached_armed = False
        self._publish_mission_debug()

    def _tick(self) -> None:
        self._publish_mission_debug()
        now = self.get_clock().now()

        if self._mission_state == "IDLE":
            if self._auto_start:
                self._mission_state = "LOAD_WAYPOINTS"
                self._publish_mission_debug()
            else:
                return

        if self._mission_state == "LOAD_WAYPOINTS":
            if not self._load_yaml():
                self._mission_state = "ERROR"
                self._publish_mission_debug()
                return
            self._wp_index = 0
            self._mission_state = "LOAD_WAYPOINTS"
            self._publish_mission_debug()
            self._start_send_goal_phase(now)
            return

        if self._mission_state == "WAIT_FOR_REACHED":
            if self._wp_index >= len(self._waypoints):
                self._mission_state = "DONE"
                self._publish_mission_debug()
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
                        f"Goal timeout — prøver waypoint {self._wp_index + 1} på nytt "
                        f"(forsøk {self._timeout_retries}/{self._max_retries})."
                    )
                    self._publish_goal(wp, self._wp_index + 1, len(self._waypoints))
                    self._wait_deadline = now + Duration(seconds=float(self._goal_timeout))
                    self._reached_armed = False
                else:
                    self.get_logger().error(
                        f"Goal timeout for waypoint {self._wp_index + 1} etter {self._max_retries} omforsøk."
                    )
                    self._mission_state = "ERROR"
                    self._publish_mission_debug()
            return

        if self._mission_state == "WAIT_AFTER_REACHED":
            if self._elapsed() < self._wait_after:
                return
            self._mission_state = "NEXT_WAYPOINT"
            self._publish_mission_debug()

        if self._mission_state == "NEXT_WAYPOINT":
            self._wp_index += 1
            if self._wp_index < len(self._waypoints):
                self._start_send_goal_phase(now)
            elif self._loop and len(self._waypoints) > 0:
                self._wp_index = 0
                self._start_send_goal_phase(now)
            else:
                self._mission_state = "DONE"
                self._publish_mission_debug()
            return

        # DONE, ERROR, SEND_GOAL (transient): ingen periodisk logikk


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointMissionNode()
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
