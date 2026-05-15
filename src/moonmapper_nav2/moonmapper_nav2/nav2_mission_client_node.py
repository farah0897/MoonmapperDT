#!/usr/bin/env python3
"""Action client for Nav2 NavigateThroughPoses (poses in `map` or `odom` per mission_frame)."""

from __future__ import annotations

import math
import os
from typing import Any, List, Optional, Tuple

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _quat_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _occupancy_grid_aabb(msg: OccupancyGrid) -> Tuple[float, float, float, float]:
    info = msg.info
    w, h = int(info.width), int(info.height)
    res = float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    yaw = _quat_yaw(info.origin.orientation)
    corners: List[Tuple[float, float]] = []
    for i in (0, w):
        for j in (0, h):
            mx, my = i * res, j * res
            wx = ox + mx * math.cos(yaw) - my * math.sin(yaw)
            wy = oy + mx * math.sin(yaw) + my * math.cos(yaw)
            corners.append((wx, wy))
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    return min(xs), max(xs), min(ys), max(ys)


def _read_pgm_size(path: str) -> Optional[Tuple[int, int]]:
    try:
        with open(path, "rb") as f:
            line = f.readline()
            if not line.startswith(b"P5"):
                return None
            line = f.readline()
            while line.startswith(b"#") or line.strip() == b"":
                line = f.readline()
            parts = line.split()
            while len(parts) < 2:
                parts += f.readline().split()
            return int(parts[0]), int(parts[1])
    except OSError:
        return None


class Nav2MissionClientNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_mission_client_node")
        self._cb_group = ReentrantCallbackGroup()
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("mission_frame", "map")
        self.declare_parameter("localization_tf_stable_count", 5)
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("autostart", True)
        self.declare_parameter("loop", False)
        self.declare_parameter("goal_timeout_sec", 0.0)
        self.declare_parameter("action_name", "/navigate_through_poses")
        self.declare_parameter("waypoints", [0.0, 0.0, 0.0])
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("navigator_lifecycle_node", "bt_navigator")
        self.declare_parameter("wait_for_localization_tf", True)
        self.declare_parameter("localization_mode", "")
        self.declare_parameter("retry_start_delay_sec", 2.0)
        self.declare_parameter("reject_retry_sec", 2.0)
        self.declare_parameter("max_reject_retries", 0)
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_margin_m", 1.05)
        self.declare_parameter("require_map_bounds", True)
        self.declare_parameter("require_robot_in_safe_before_send", True)
        self.declare_parameter("abort_if_robot_leaves_safe", True)
        self.declare_parameter("safe_pose_check_period_sec", 0.35)

        mf_raw = (
            self.get_parameter("mission_frame").get_parameter_value().string_value.strip().lower()
        )
        self._mission_frame: str = mf_raw if mf_raw in ("map", "odom") else "map"
        fid = self.get_parameter("frame_id").get_parameter_value().string_value.strip()
        if self._mission_frame == "odom":
            self._frame_id = "odom"
        else:
            self._frame_id = fid if fid else "map"

        self._start_delay: float = (
            self.get_parameter("start_delay_sec").get_parameter_value().double_value
        )
        self._autostart: bool = self.get_parameter("autostart").get_parameter_value().bool_value
        self._loop: bool = self.get_parameter("loop").get_parameter_value().bool_value
        gt = self.get_parameter("goal_timeout_sec").get_parameter_value().double_value
        self._goal_timeout_sec: Optional[float] = gt if gt > 0.0 else None
        self._action_name: str = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        self._base_frame: str = self.get_parameter("base_frame").get_parameter_value().string_value
        nav_lc = self.get_parameter("navigator_lifecycle_node").get_parameter_value().string_value
        self._navigator_lc: str = nav_lc.strip().strip("/")
        self._wait_tf: bool = (
            self.get_parameter("wait_for_localization_tf").get_parameter_value().bool_value
        )
        loc_raw = (
            self.get_parameter("localization_mode").get_parameter_value().string_value.strip().lower()
        )
        if loc_raw in ("odom", "amcl"):
            self._localization_mode = loc_raw
        elif self._mission_frame == "odom":
            self._localization_mode = "odom"
        else:
            self._localization_mode = "amcl"
        self._wait_map_odom_tf = self._localization_mode == "amcl" and self._mission_frame == "map"
        self._map_odom_streak = 0
        self._retry_start_delay: float = (
            self.get_parameter("retry_start_delay_sec").get_parameter_value().double_value
        )
        self._reject_retry_sec: float = (
            self.get_parameter("reject_retry_sec").get_parameter_value().double_value
        )
        self._max_reject_retries: int = (
            int(self.get_parameter("max_reject_retries").get_parameter_value().integer_value)
        )
        self._map_yaml_path: str = (
            self.get_parameter("map_yaml_path").get_parameter_value().string_value.strip()
        )
        self._map_margin_m: float = (
            self.get_parameter("map_margin_m").get_parameter_value().double_value
        )
        self._require_bounds: bool = (
            self.get_parameter("require_map_bounds").get_parameter_value().bool_value
        )
        self._require_robot_safe_before_send: bool = (
            self.get_parameter("require_robot_in_safe_before_send").get_parameter_value().bool_value
        )
        self._abort_if_robot_leaves_safe: bool = (
            self.get_parameter("abort_if_robot_leaves_safe").get_parameter_value().bool_value
        )
        self._safe_pose_check_period: float = (
            self.get_parameter("safe_pose_check_period_sec").get_parameter_value().double_value
        )
        self._tf_stable_needed: int = max(
            1,
            int(
                self.get_parameter("localization_tf_stable_count")
                .get_parameter_value()
                .integer_value
            ),
        )
        self._tf_success_streak: int = 0

        if self._mission_frame == "odom":
            self._require_bounds = False
            self._require_robot_safe_before_send = False
            self._abort_if_robot_leaves_safe = False

        self._poses = self._parse_waypoints()

        self._map_aabb: Optional[Tuple[float, float, float, float]] = None
        self._safe_aabb: Optional[Tuple[float, float, float, float]] = None
        self._bounds_logged = False
        self._last_gc: Optional[OccupancyGrid] = None

        self._fb_last_log: Optional[Time] = None
        self._fb_last_poses: Optional[int] = None
        self._fb_last_dist: Optional[float] = None
        self._last_safe_check: Optional[Time] = None
        self._cancel_sent_oob: bool = False

        self._try_bounds_from_yaml()

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos, callback_group=self._cb_group
        )
        gc_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._on_global_costmap,
            gc_qos,
            callback_group=self._cb_group,
        )

        self._client = ActionClient(
            self,
            NavigateThroughPoses,
            self._action_name,
            callback_group=self._cb_group,
        )

        self._mission_state = "idle"
        self._server_ready_announced = False
        self._goal_sent = False
        self._goal_handle: Any = None
        self._result_future: Any = None
        self._delay_until: Optional[Time] = None
        self._goal_deadline: Optional[Time] = None
        self._last_wait_log: Optional[Time] = None
        self._run_gen = 0
        self._next_start_delay_sec: float = self._start_delay
        self._reject_count = 0
        self._get_state_future: Any = None
        lc_srv = f"/{self._navigator_lc}/get_state"
        self._get_state_client = self.create_client(GetState, lc_srv, callback_group=self._cb_group)
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self.get_logger().info(
            f"Nav2 mission client: action={self._action_name}, mission_frame={self._mission_frame}, "
            f"localization_mode={self._localization_mode}, frame_id={self._frame_id}, "
            f"waypoint_count={len(self._poses)}, map_margin_m={self._map_margin_m}, "
            f"require_map_bounds={self._require_bounds}, map_yaml_path={self._map_yaml_path or '(none)'}"
        )
        for i, p in enumerate(self._poses):
            self.get_logger().info(
                f"  waypoint[{i}]: x={p.pose.position.x:.4f} y={p.pose.position.y:.4f} "
                f"yaw={_quat_yaw(p.pose.orientation):.4f} rad"
            )

        self._timer = self.create_timer(0.2, self._tick, callback_group=self._cb_group)
        self._start_srv = self.create_service(
            Trigger,
            "start_mission",
            self._on_start_service,
            callback_group=self._cb_group,
        )
        if not self._autostart:
            self.get_logger().info(
                "autostart=false: call `ros2 service call /nav2_mission_client_node/start_mission "
                "std_srvs/srv/Trigger` to run the mission once."
            )

    def _try_bounds_from_yaml(self) -> None:
        if self._mission_frame == "odom":
            return
        if not self._map_yaml_path or not os.path.isfile(self._map_yaml_path):
            return
        try:
            with open(self._map_yaml_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            if not isinstance(cfg, dict):
                return
            ox = float(cfg["origin"][0])
            oy = float(cfg["origin"][1])
            oyaw = float(cfg["origin"][2]) if len(cfg["origin"]) > 2 else 0.0
            res = float(cfg["resolution"])
            img_name = str(cfg["image"])
            base = os.path.dirname(os.path.abspath(self._map_yaml_path))
            img_path = img_name if os.path.isabs(img_name) else os.path.join(base, img_name)
            wh = _read_pgm_size(img_path)
            if wh is None:
                self.get_logger().warn(f"could not read PGM dimensions from {img_path}")
                return
            w, h = wh
            corners: List[Tuple[float, float]] = []
            for i in (0, w):
                for j in (0, h):
                    mx, my = i * res, j * res
                    c, s = math.cos(oyaw), math.sin(oyaw)
                    wx = ox + mx * c - my * s
                    wy = oy + mx * s + my * c
                    corners.append((wx, wy))
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            self._set_map_bounds(min(xs), max(xs), min(ys), max(ys), source=f"yaml:{self._map_yaml_path}")
        except Exception as e:
            self.get_logger().warn(f"map_yaml_path preload failed: {e}")

    def _set_map_bounds(self, xmin: float, xmax: float, ymin: float, ymax: float, source: str) -> None:
        m = float(self._map_margin_m)
        self._map_aabb = (xmin, xmax, ymin, ymax)
        self._safe_aabb = (xmin + m, xmax - m, ymin + m, ymax - m)
        sx0, sx1, sy0, sy1 = self._safe_aabb
        if sx0 >= sx1 or sy0 >= sy1:
            self.get_logger().error(
                f"safe bounds empty (margin {m} too large for map). map AABB=({xmin:.3f},{xmax:.3f})"
                f"x({ymin:.3f},{ymax:.3f})y — source={source}"
            )
            self._safe_aabb = None
            return
        if not self._bounds_logged:
            self._bounds_logged = True
            self.get_logger().info(
                f"map AABB ({source}): x=[{xmin:.3f},{xmax:.3f}] y=[{ymin:.3f},{ymax:.3f}] "
                f"(margin {m} m) safe: x=[{sx0:.3f},{sx1:.3f}] y=[{sy0:.3f},{sy1:.3f}]"
            )

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._mission_frame == "odom":
            return
        xmin, xmax, ymin, ymax = _occupancy_grid_aabb(msg)
        self._set_map_bounds(xmin, xmax, ymin, ymax, source="/map")

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._last_gc = msg

    def _parse_waypoints(self) -> List[PoseStamped]:
        wp_param = self.get_parameter("waypoints").value
        triples: List[List[float]] = []
        if isinstance(wp_param, list):
            if wp_param and isinstance(wp_param[0], (list, tuple)):
                for row in wp_param:
                    if isinstance(row, (list, tuple)) and len(row) >= 3:
                        triples.append([float(row[0]), float(row[1]), float(row[2])])
                    else:
                        self.get_logger().warn(f"skip invalid waypoint row: {row}")
            else:
                flat = [float(x) for x in wp_param]
                if len(flat) % 3 != 0:
                    self.get_logger().error("waypoints flat list length must be multiple of 3")
                for i in range(0, len(flat) - 2, 3):
                    triples.append([flat[i], flat[i + 1], flat[i + 2]])
        if not triples:
            self.get_logger().error("no valid waypoints; expected list of [x, y, yaw]")
        out: List[PoseStamped] = []
        for x, y, yaw in triples:
            ps = PoseStamped()
            ps.header.frame_id = self._frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(float(yaw))
            out.append(ps)
        return out

    def _waypoints_in_safe_bounds(self) -> bool:
        if not self._require_bounds or self._safe_aabb is None:
            return True
        sx0, sx1, sy0, sy1 = self._safe_aabb
        ok = True
        eps = 1e-4
        for i, p in enumerate(self._poses):
            x, y = p.pose.position.x, p.pose.position.y
            if not (sx0 - eps <= x <= sx1 + eps and sy0 - eps <= y <= sy1 + eps):
                self.get_logger().error(
                    f"waypoint[{i}] ({x:.4f}, {y:.4f}) outside safe bounds "
                    f"x=[{sx0:.3f},{sx1:.3f}] y=[{sy0:.3f},{sy1:.3f}] — NOT sending mission"
                )
                ok = False
        return ok

    def _robot_in_safe_bounds_xy(self, x: float, y: float) -> bool:
        if self._safe_aabb is None:
            return True
        sx0, sx1, sy0, sy1 = self._safe_aabb
        return sx0 <= x <= sx1 and sy0 <= y <= sy1

    def _log_abort_diagnostics(self, status: int, result: Any) -> None:
        self.get_logger().error(f"NavigateThroughPoses terminal diagnostics (status_code={status})")
        if result is not None:
            self.get_logger().error(
                f"  result.error_code={int(result.error_code)} error_msg={result.error_msg!r}"
            )
        try:
            t = self._tf_buffer.lookup_transform(
                self._frame_id, self._base_frame, Time(), timeout=Duration(seconds=0.2)
            )
            rx, ry = t.transform.translation.x, t.transform.translation.y
            inside = self._robot_in_safe_bounds_xy(rx, ry)
            self.get_logger().error(
                f"  last TF {self._frame_id}->{self._base_frame}: x={rx:.3f} y={ry:.3f} "
                f"in_safe_bounds={inside}"
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"  TF lookup failed: {e}")
        if self._map_aabb is not None:
            mx0, mx1, my0, my1 = self._map_aabb
            self.get_logger().error(
                f"  map AABB: x=[{mx0:.3f},{mx1:.3f}] y=[{my0:.3f},{my1:.3f}] "
                f"margin={self._map_margin_m}"
            )
        if self._safe_aabb is not None:
            sx0, sx1, sy0, sy1 = self._safe_aabb
            self.get_logger().error(
                f"  safe AABB: x=[{sx0:.3f},{sx1:.3f}] y=[{sy0:.3f},{sy1:.3f}]"
            )
        gc = self._last_gc
        if gc is not None:
            gx0, gx1, gy0, gy1 = _occupancy_grid_aabb(gc)
            self.get_logger().error(
                f"  last /global_costmap/costmap AABB: x=[{gx0:.3f},{gx1:.3f}] y=[{gy0:.3f},{gy1:.3f}] "
                f"frame={gc.header.frame_id}"
            )
        else:
            self.get_logger().error("  (no /global_costmap/costmap message received yet)")

    def _stamp_poses(self) -> List[PoseStamped]:
        now = self.get_clock().now().to_msg()
        stamped: List[PoseStamped] = []
        for p in self._poses:
            q = PoseStamped()
            q.header.stamp = now
            q.header.frame_id = self._frame_id
            q.pose = p.pose
            stamped.append(q)
        return stamped

    def _on_start_service(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._mission_state not in ("idle", "done", "canceled", "failed"):
            resp.success = False
            resp.message = f"mission busy (state={self._mission_state})"
            return resp
        self._begin_mission_cycle()
        resp.success = True
        resp.message = "mission scheduled"
        return resp

    def _begin_mission_cycle(self) -> None:
        if not self._poses:
            self.get_logger().error("no poses to send; abort")
            self._mission_state = "done"
            return
        self._run_gen += 1
        self._mission_state = "wait_action_server"
        self._goal_sent = False
        self._goal_handle = None
        self._result_future = None
        self._delay_until = None
        self._goal_deadline = None
        self._get_state_future = None
        self._next_start_delay_sec = self._start_delay
        self._reject_count = 0
        self._last_safe_check = None
        self._cancel_sent_oob = False
        self._tf_success_streak = 0

    def _arm_start_delay(self, seconds: float) -> None:
        self._delay_until = self.get_clock().now() + Duration(seconds=seconds)
        self.get_logger().info(f"armed start delay: {seconds:.2f} s before sending goal")

    def _tick(self) -> None:
        if self._mission_state == "idle":
            if self._autostart:
                self._begin_mission_cycle()
            return

        if self._mission_state == "wait_action_server":
            if self._client.server_is_ready():
                if not self._server_ready_announced:
                    self.get_logger().info(
                        f"NavigateThroughPoses action endpoint up: {self._action_name} "
                        f"(waiting for bt_navigator ACTIVE + TF before goal)"
                    )
                    self._server_ready_announced = True
                self._mission_state = "wait_bt_active"
                self._get_state_future = None
                self._last_wait_log = None
            else:
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 5e9:
                    self.get_logger().info(
                        f"still waiting for action server {self._action_name} (Nav2 starting?)"
                    )
                    self._last_wait_log = now
            return

        if self._mission_state == "wait_bt_active":
            if not self._get_state_client.service_is_ready():
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 5e9:
                    self.get_logger().info(
                        f"waiting for lifecycle service /{self._navigator_lc}/get_state …"
                    )
                    self._last_wait_log = now
                return
            if self._get_state_future is None:
                self._get_state_future = self._get_state_client.call_async(GetState.Request())
                return
            if not self._get_state_future.done():
                return
            resp = self._get_state_future.result()
            self._get_state_future = None
            if resp is None:
                return
            sid = int(resp.current_state.id)
            if sid == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(
                    f"bt_navigator is ACTIVE (lifecycle id={sid}); proceeding to localization check"
                )
                if self._wait_tf:
                    self._last_wait_log = None
                    self._tf_success_streak = 0
                    self._map_odom_streak = 0
                    self._mission_state = (
                        "wait_map_odom_tf" if self._wait_map_odom_tf else "wait_map_tf"
                    )
                else:
                    self._after_localization_ready()
            else:
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 3e9:
                    self.get_logger().info(
                        f"waiting for bt_navigator ACTIVE (now lifecycle id={sid}; "
                        f"planner/global_costmap needs {self._frame_id}->{self._base_frame})…"
                    )
                    self._last_wait_log = now
            return

        if self._mission_state == "wait_map_odom_tf":
            try:
                self._tf_buffer.lookup_transform(
                    "map",
                    "odom",
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except tf2_ros.TransformException:
                self._map_odom_streak = 0
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 3e9:
                    self.get_logger().info(
                        "AMCL mode: waiting for TF map->odom (check amcl + initial_x/y/yaw match spawn)"
                    )
                    self._last_wait_log = now
                return
            self._map_odom_streak += 1
            need_mo = min(3, self._tf_stable_needed)
            if self._map_odom_streak < need_mo:
                return
            self.get_logger().info(f"TF OK ({need_mo} consecutive): map -> odom")
            self._map_odom_streak = 0
            self._tf_success_streak = 0
            self._last_wait_log = None
            self._mission_state = "wait_map_tf"
            return

        if self._mission_state == "wait_map_tf":
            try:
                self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except tf2_ros.TransformException:
                self._tf_success_streak = 0
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 3e9:
                    if self._mission_frame == "odom":
                        self.get_logger().info(
                            f"waiting for TF {self._frame_id}->{self._base_frame} "
                            f"(odom test: ensure sim / robot_state_publisher + diff_drive publish odom)"
                        )
                    elif self._localization_mode == "odom":
                        self.get_logger().info(
                            f"waiting for TF map->{self._base_frame} "
                            "(odom static map: identity map->odom + sim odom->base)"
                        )
                    else:
                        self.get_logger().info(
                            "AMCL mode: waiting for TF map->base_footprint "
                            "(set initial_x/y/yaw to Gazebo spawn; no RViz 2D Pose Estimate required)"
                        )
                    self._last_wait_log = now
                return
            self._tf_success_streak += 1
            if self._mission_frame == "odom":
                need = 1
            elif self._localization_mode == "odom":
                need = min(2, self._tf_stable_needed)
            else:
                need = self._tf_stable_needed
            if self._tf_success_streak < need:
                return
            self.get_logger().info(
                f"TF OK ({need} consecutive): {self._frame_id} -> {self._base_frame}"
            )
            self._tf_success_streak = 0
            self._after_localization_ready()
            return

        if self._mission_state == "wait_map_bounds":
            if self._require_bounds and self._safe_aabb is None:
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log).nanoseconds > 5e9:
                    self.get_logger().info(
                        "waiting for /map (or valid map_yaml_path) to compute safe waypoint bounds…"
                    )
                    self._last_wait_log = now
                return
            if not self._waypoints_in_safe_bounds():
                self._mission_state = "failed"
                return
            self._arm_start_delay(self._next_start_delay_sec)
            self._mission_state = "start_delay"
            return

        if self._mission_state == "await_accept":
            return

        if self._mission_state == "reject_cooldown":
            assert self._delay_until is not None
            if self.get_clock().now() < self._delay_until:
                return
            self._get_state_future = None
            self._mission_state = "wait_bt_active"
            return

        if self._mission_state == "start_delay":
            assert self._delay_until is not None
            if self.get_clock().now() < self._delay_until:
                return
            self._send_goal()
            return

        if self._mission_state == "await_result":
            assert self._result_future is not None
            if (
                self._abort_if_robot_leaves_safe
                and self._safe_aabb is not None
                and not self._cancel_sent_oob
            ):
                now = self.get_clock().now()
                period_ns = max(0.05, self._safe_pose_check_period) * 1e9
                if self._last_safe_check is None or (now - self._last_safe_check).nanoseconds >= period_ns:
                    self._last_safe_check = now
                    try:
                        t = self._tf_buffer.lookup_transform(
                            self._frame_id,
                            self._base_frame,
                            Time(),
                            timeout=Duration(seconds=0.1),
                        )
                        rx, ry = t.transform.translation.x, t.transform.translation.y
                        if not self._robot_in_safe_bounds_xy(rx, ry):
                            self.get_logger().error(
                                f"robot ({rx:.3f},{ry:.3f}) left safe map margin — canceling Nav2 goal"
                            )
                            if self._goal_handle is not None:
                                self._goal_handle.cancel_goal_async()
                            self._cancel_sent_oob = True
                    except tf2_ros.TransformException:
                        pass
            if self._result_future.done():
                self._on_result(self._result_future.result())
                return
            if self._goal_deadline is not None and self.get_clock().now() > self._goal_deadline:
                self.get_logger().warn("goal_timeout_sec elapsed; canceling goal")
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                self._mission_state = "canceled"
            return

        if self._mission_state in ("done", "canceled", "failed"):
            if self._loop and self._mission_state == "done":
                self.get_logger().info("loop=true: restarting mission after short pause")
                self._delay_until = self.get_clock().now() + Duration(seconds=2.0)
                self._mission_state = "loop_pause"
            return

        if self._mission_state == "loop_pause":
            assert self._delay_until is not None
            if self.get_clock().now() >= self._delay_until:
                self._begin_mission_cycle()
            return

    def _after_localization_ready(self) -> None:
        if self._require_bounds:
            self._try_bounds_from_yaml()
            if self._safe_aabb is None:
                self._last_wait_log = None
                self._mission_state = "wait_map_bounds"
                return
            if not self._waypoints_in_safe_bounds():
                self._mission_state = "failed"
                return
        self._arm_start_delay(self._next_start_delay_sec)
        self._mission_state = "start_delay"

    def _send_goal(self) -> None:
        if self._require_bounds and self._safe_aabb is not None and not self._waypoints_in_safe_bounds():
            self.get_logger().error("pre-send validation failed; aborting mission")
            self._mission_state = "failed"
            return
        if (
            self._require_robot_safe_before_send
            and self._require_bounds
            and self._safe_aabb is not None
        ):
            try:
                t = self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.3),
                )
                rx, ry = t.transform.translation.x, t.transform.translation.y
                if not self._robot_in_safe_bounds_xy(rx, ry):
                    self.get_logger().error(
                        f"robot ({rx:.3f},{ry:.3f}) is outside safe margin before send — fix 2D Pose Estimate "
                        "(click on the rover in the map where it stands in Gazebo), then "
                        "`ros2 service call /nav2_mission_client_node/start_mission std_srvs/srv/Trigger` "
                        "or restart launch."
                    )
                    self._mission_state = "failed"
                    return
            except tf2_ros.TransformException as e:
                self.get_logger().error(f"cannot verify robot pose before send: {e}")
                self._mission_state = "failed"
                return
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._stamp_poses()
        goal.behavior_tree = ""
        self.get_logger().info(
            f"sending NavigateThroughPoses goal with {len(goal.poses)} poses (behavior_tree='')"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        self._mission_state = "await_accept"
        self._goal_sent = True
        self.get_logger().info("mission goal dispatch async (waiting for accept)")

    def _goal_response_cb(self, future: Any) -> None:
        gh = future.result()
        self._goal_handle = gh
        if not gh.accepted:
            self._reject_count += 1
            self.get_logger().error(
                "NavigateThroughPoses goal rejected (bt_navigator inactive or not ready). "
                "Will retry after localization / lifecycle activation."
            )
            if self._max_reject_retries > 0 and self._reject_count >= self._max_reject_retries:
                self.get_logger().error(
                    f"max_reject_retries={self._max_reject_retries} reached; giving up"
                )
                self._mission_state = "failed"
                return
            self._next_start_delay_sec = self._retry_start_delay
            self._delay_until = self.get_clock().now() + Duration(seconds=self._reject_retry_sec)
            self._mission_state = "reject_cooldown"
            self._last_wait_log = None
            return
        self.get_logger().info("NavigateThroughPoses goal accepted")
        self._last_safe_check = None
        self._cancel_sent_oob = False
        if self._goal_timeout_sec is not None:
            self._goal_deadline = self.get_clock().now() + Duration(seconds=self._goal_timeout_sec)
        else:
            self._goal_deadline = None
        self._result_future = gh.get_result_async()
        self._mission_state = "await_result"

    def _feedback_cb(self, fb: Any) -> None:
        m = fb.feedback
        now = self.get_clock().now()
        poses = int(m.number_of_poses_remaining)
        dist = float(m.distance_remaining)
        poses_ch = self._fb_last_poses is None or poses != self._fb_last_poses
        dist_ch = self._fb_last_dist is None or abs(dist - self._fb_last_dist) >= 0.25
        interesting = poses_ch or dist_ch
        if not interesting:
            return
        time_ok = self._fb_last_log is None or (now - self._fb_last_log).nanoseconds >= 1_000_000_000
        if not time_ok:
            return
        self.get_logger().info(
            f"Nav2 feedback: poses_remaining={poses} distance_remaining={dist:.1f} m "
            f"recoveries={m.number_of_recoveries}"
        )
        self._fb_last_log = now
        self._fb_last_poses = poses
        self._fb_last_dist = dist

    def _on_result(self, wrapped: Any) -> None:
        status = wrapped.status
        result = wrapped.result
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        label = labels.get(status, f"STATUS_{status}")
        self.get_logger().info(f"NavigateThroughPoses finished: {label} (status_code={status})")
        if result is not None and int(result.error_code) != int(NavigateThroughPoses.Result.NONE):
            self.get_logger().warn(
                f"result error_code={result.error_code} error_msg={result.error_msg!r}"
            )
        if status == GoalStatus.STATUS_ABORTED:
            self._log_abort_diagnostics(status, result)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._mission_state = "done"
        elif status == GoalStatus.STATUS_CANCELED:
            self._mission_state = "canceled"
        else:
            self._mission_state = "failed"


def main() -> None:
    rclpy.init()
    node = Nav2MissionClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
