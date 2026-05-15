#!/usr/bin/env python3
"""Logging/metrics for Nav2 static-map replan-around-obstacle test."""

from __future__ import annotations

import hashlib
import math
import sys
from typing import Any, List, Optional, Tuple

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


def _yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _plan_signature(path: Path) -> str:
    if not path.poses:
        return "empty"
    h = hashlib.sha1()
    for p in path.poses[:: max(1, len(path.poses) // 8)]:
        h.update(
            f"{p.pose.position.x:.3f},{p.pose.position.y:.3f},".encode()
        )
    return f"n={len(path.poses)}:{h.hexdigest()[:8]}"


def _count_lethal_near(
    grid: OccupancyGrid, wx: float, wy: float, radius_m: float, lethal_threshold: int = 50
) -> int:
    info = grid.info
    res = float(info.resolution)
    if res <= 0.0:
        return 0
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    oyaw = _yaw_from_quat(info.origin.orientation)
    c, s = math.cos(oyaw), math.sin(oyaw)
    mx = (wx - ox) * c + (wy - oy) * s
    my = -(wx - ox) * s + (wy - oy) * c
    ci = int(mx / res)
    cj = int(my / res)
    r_cells = max(1, int(radius_m / res))
    w, h = int(info.width), int(info.height)
    data = grid.data
    count = 0
    for di in range(-r_cells, r_cells + 1):
        for dj in range(-r_cells, r_cells + 1):
            i, j = ci + di, cj + dj
            if 0 <= i < w and 0 <= j < h:
                v = int(data[i + j * w])
                if v >= lethal_threshold:
                    count += 1
    return count


class Nav2ReplanMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_replan_monitor")
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("autostart_goal", False)
        self.declare_parameter("goal_delay_sec", 12.0)
        self.declare_parameter("log_period_sec", 1.0)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_margin_m", 1.05)
        self.declare_parameter("scan_front_half_deg", 35.0)

        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._goal_yaw = float(self.get_parameter("goal_yaw").value)
        self._autostart = bool(self.get_parameter("autostart_goal").value)
        self._goal_delay = float(self.get_parameter("goal_delay_sec").value)
        self._period = max(0.5, float(self.get_parameter("log_period_sec").value))
        self._base = str(self.get_parameter("base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._margin = float(self.get_parameter("map_margin_m").value)
        self._scan_half = math.radians(float(self.get_parameter("scan_front_half_deg").value))

        self._map_aabb: Optional[Tuple[float, float, float, float]] = None
        self._last_plan_sig = ""
        self._plan_change_count = 0
        self._nav_status = "idle"
        self._goal_sent = False
        self._goal_handle: Any = None
        self._result_future: Any = None

        self._last_scan: Optional[LaserScan] = None
        self._last_plan: Optional[Path] = None
        self._last_local_gc: Optional[OccupancyGrid] = None
        self._last_global_gc: Optional[OccupancyGrid] = None
        self._last_raw: Optional[Twist] = None
        self._last_safe: Optional[Twist] = None
        self._last_front_min: Optional[float] = None
        self._last_safety_state = ""

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos, callback_group=self._cb
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, 10, callback_group=self._cb
        )
        self.create_subscription(Path, "/plan", self._on_plan, 10, callback_group=self._cb)
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._on_local_gc,
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._on_global_gc,
            10,
            callback_group=self._cb,
        )
        self.create_subscription(
            Twist, "/cmd_vel_raw", lambda m: setattr(self, "_last_raw", m), 10, callback_group=self._cb
        )
        self.create_subscription(
            Twist, "/cmd_vel", lambda m: setattr(self, "_last_safe", m), 10, callback_group=self._cb
        )
        self.create_subscription(
            Float32, "/safety/front_min", self._on_front_min, 10, callback_group=self._cb
        )
        self.create_subscription(
            String, "/obstacle/current_state", self._on_obstacle_state, 10, callback_group=self._cb
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb
        )
        self._start_at = self.get_clock().now() + Duration(seconds=self._goal_delay)
        self.create_timer(self._period, self._tick, callback_group=self._cb)

        self.get_logger().info(
            f"replan monitor: goal=({self._goal_x:.2f},{self._goal_y:.2f},yaw={self._goal_yaw:.2f}) "
            f"autostart={self._autostart} delay={self._goal_delay:.1f}s"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        w, h = int(info.width), int(info.height)
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        yaw = _yaw_from_quat(info.origin.orientation)
        corners: List[Tuple[float, float]] = []
        for i in (0, w):
            for j in (0, h):
                mx, my = i * res, j * res
                wx = ox + mx * math.cos(yaw) - my * math.sin(yaw)
                wy = oy + mx * math.sin(yaw) + my * math.cos(yaw)
                corners.append((wx, wy))
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        self._map_aabb = (min(xs), max(xs), min(ys), max(ys))

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _on_plan(self, msg: Path) -> None:
        sig = _plan_signature(msg)
        if sig != self._last_plan_sig:
            if self._last_plan_sig:
                self._plan_change_count += 1
            self._last_plan_sig = sig
        self._last_plan = msg

    def _on_local_gc(self, msg: OccupancyGrid) -> None:
        self._last_local_gc = msg

    def _on_global_gc(self, msg: OccupancyGrid) -> None:
        self._last_global_gc = msg

    def _on_front_min(self, msg: Float32) -> None:
        v = float(msg.data)
        self._last_front_min = None if math.isnan(v) else v

    def _on_obstacle_state(self, msg: String) -> None:
        self._last_safety_state = msg.data

    def _scan_front_min(self) -> Optional[float]:
        scan = self._last_scan
        if scan is None:
            return None
        best: Optional[float] = None
        n = len(scan.ranges)
        for i in range(n):
            a = scan.angle_min + float(i) * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) > self._scan_half:
                continue
            r = float(scan.ranges[i])
            if math.isnan(r) or math.isinf(r):
                continue
            if not (scan.range_min < r < scan.range_max):
                continue
            best = r if best is None else min(best, r)
        return best

    def _robot_xy_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base, Time(), timeout=Duration(seconds=0.15)
            )
            tr = t.transform.translation
            return tr.x, tr.y, _yaw_from_quat(t.transform.rotation)
        except tf2_ros.TransformException:
            return None

    def _in_map_margin(self, x: float, y: float) -> bool:
        if self._map_aabb is None:
            return True
        xmin, xmax, ymin, ymax = self._map_aabb
        m = self._margin
        return (xmin + m) <= x <= (xmax - m) and (ymin + m) <= y <= (ymax - m)

    def _maybe_send_goal(self) -> None:
        if not self._autostart or self._goal_sent:
            return
        if self.get_clock().now() < self._start_at:
            return
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            return
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._goal_x
        pose.pose.position.y = self._goal_y
        half = self._goal_yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_sent = True
        self._nav_status = "sending"
        send = self._nav_client.send_goal_async(goal)
        send.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        self._goal_handle = future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self._nav_status = "rejected"
            self.get_logger().error("NavigateToPose goal rejected")
            return
        self._nav_status = "active"
        self.get_logger().info("NavigateToPose goal accepted")
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        result = future.result()
        status = int(result.status)
        self._nav_status = f"done status={status}"
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("NavigateToPose: Goal succeeded")
        else:
            self.get_logger().warn(
                f"NavigateToPose finished status={status} "
                f"(4=succeeded, 5=canceled, 6=aborted)"
            )

    def _tick(self) -> None:
        if not rclpy.ok():
            return
        self._maybe_send_goal()

        pose = self._robot_xy_yaw()
        scan_fm = self._scan_front_min()
        front = self._last_front_min if self._last_front_min is not None else scan_fm

        local_lethal = global_lethal = -1
        if pose is not None:
            rx, ry, _ = pose
            if self._last_local_gc is not None:
                local_lethal = _count_lethal_near(self._last_local_gc, rx, ry, 0.9)
            if self._last_global_gc is not None:
                global_lethal = _count_lethal_near(self._last_global_gc, rx, ry, 1.2)

        raw = self._last_raw
        safe = self._last_safe
        rlx = float(raw.linear.x) if raw else float("nan")
        rlz = float(raw.angular.z) if raw else float("nan")
        slx = float(safe.linear.x) if safe else float("nan")
        slz = float(safe.angular.z) if safe else float("nan")

        in_bounds = True
        if pose is not None:
            in_bounds = self._in_map_margin(pose[0], pose[1])

        plan_n = len(self._last_plan.poses) if self._last_plan else 0
        pose_s = (
            f"({pose[0]:.3f},{pose[1]:.3f},yaw={math.degrees(pose[2]):.1f}°)"
            if pose
            else "MISSING"
        )
        fm_s = f"{front:.3f}" if front is not None else "nan"

        self.get_logger().info(
            f"replan: pose={pose_s} goal=({self._goal_x:.2f},{self._goal_y:.2f}) "
            f"in_map_margin={in_bounds} scan_front_min={fm_s} safety={self._last_safety_state or '?'} "
            f"raw_v=({rlx:.3f},{rlz:.3f}) safe_v=({slx:.3f},{slz:.3f}) "
            f"plan_poses={plan_n} plan_changes={self._plan_change_count} sig={self._last_plan_sig} "
            f"local_lethal={local_lethal} global_lethal={global_lethal} nav={self._nav_status}"
        )


def main() -> int:
    rclpy.init()
    node: Nav2ReplanMonitorNode | None = None
    try:
        node = Nav2ReplanMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        safe_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
