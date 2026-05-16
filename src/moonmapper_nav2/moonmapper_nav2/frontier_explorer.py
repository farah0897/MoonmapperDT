#!/usr/bin/env python3
"""V1 frontier exploration via Nav2 NavigateToPose (safe minimal behavior)."""

from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from moonmapper_nav2.frontier_map_debug import (
    classify_map_cells,
    format_map_debug_header,
    format_map_stats_line,
    local_radius_stats,
    map_value_range_str,
    robot_cell_debug_line,
    sanitize_occ_grid_data,
)
from moonmapper_nav2.frontier_grid import (
    RejectReason,
    ValidatedGoal,
    apply_robot_footprint_clearing,
    build_passable_mask,
    collect_frontier_clusters,
    count_raw_frontier_cells,
    format_reachability_debug_line,
    global_nearest_free_cell,
    resolve_bfs_seed_and_masks,
    score_cluster_distance,
    validate_and_build_goal,
)
from moonmapper_nav2.frontier_utils import world_to_map
from moonmapper_nav2.rclpy_shutdown import is_shutdown_exception, safe_shutdown

MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Advertise types reliably for ros2 topic info / tooling
FRONTIER_TOPIC_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def declare_parameter_if_not_declared(node: Node, name: str, default_value):
    """Declare only if absent (YAML / launch dict may already set parameters)."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    try:
        node.declare_parameter(name, default_value)
    except ParameterAlreadyDeclaredException:
        pass
    return node.get_parameter(name).value


class _St(Enum):
    START = auto()
    WAIT_MAP = auto()
    WAIT_TF = auto()
    SPIN = auto()
    WAIT_NAV2 = auto()
    SELECT = auto()
    SEND = auto()
    NAV = auto()
    PAUSE = auto()
    DONE = auto()


def _yaw_to_q(yaw: float) -> Quaternion:
    h = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


def _norm(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self._cb = ReentrantCallbackGroup()
        self._tick_g = MutuallyExclusiveCallbackGroup()
        d = declare_parameter_if_not_declared
        d(self, "use_sim_time", True)
        d(self, "map_topic", "/map")
        d(self, "map_frame", "map")
        d(self, "base_frame", "base_footprint")
        d(self, "navigate_action", "/navigate_to_pose")
        d(self, "cmd_vel_topic", "/cmd_vel_raw")
        d(self, "exploration_rate_hz", 2.0)
        d(self, "min_frontier_cluster_size", 5)
        d(self, "min_goal_distance", 0.45)
        d(self, "max_goal_distance", 5.0)
        d(self, "max_goal_distance_stage_m", 2.5)
        d(self, "goal_timeout_sec", 120.0)
        d(self, "occupied_threshold", 65)
        d(self, "free_threshold", 0)
        d(self, "unknown_value", -1)
        d(self, "unknown_as_blocked_in_planner_grid", True)
        d(self, "goal_inflation_radius_m", 0.35)
        d(self, "retreat_from_frontier_steps", 3)
        d(self, "blacklist_radius", 0.55)
        d(self, "blacklist_timeout_sec", 120.0)
        d(self, "distance_weight", 1.0)
        d(self, "size_weight", 0.03)
        d(self, "publish_markers", True)
        d(self, "start_delay_sec", 5.0)
        d(self, "nav2_ready_timeout_sec", 120.0)
        d(self, "nav2_debug_log_interval_sec", 2.0)
        d(self, "wait_for_nav2", True)
        d(self, "require_nav_lifecycle_active", True)
        d(self, "require_cmd_vel_subscriber", True)
        d(self, "require_map_publisher_for_ready", False)
        d(self, "wait_log_interval_sec", 12.0)
        d(self, "integrated_initial_spin", False)
        d(self, "initial_spin_min_duration_sec", 1.5)
        d(self, "initial_spin_max_duration_sec", 40.0)
        d(self, "initial_spin_angular_z", 0.28)
        d(self, "initial_spin_target_rad", 6.28)
        d(self, "map_settle_after_spin_sec", 2.5)
        d(self, "nearest_seed_search_radius_m", 0.6)
        d(self, "bfs_seed_search_radius_m", 3.0)
        d(self, "bfs_seed_search_step_m", 0.25)
        d(self, "allow_robot_seed_clearing", True)
        d(self, "robot_seed_clear_radius_m", 0.25)
        d(self, "retry_when_no_frontier", True)
        d(self, "no_frontier_retry_delay_sec", 5.0)
        d(self, "max_no_frontier_retries", 0)
        d(self, "enable_staging_goal", False)
        d(self, "staging_min_distance_m", 0.2)
        d(self, "staging_max_distance_m", 0.8)
        d(self, "staging_clearance_m", 0.08)
        d(self, "staging_fan_angles_deg", [0.0])
        d(self, "staging_fan_distances_m", [0.3])
        d(self, "staging_require_free_value_zero", True)
        d(self, "consecutive_fail_limit", 3)
        d(self, "recovery_pause_sec", 2.0)
        d(self, "max_rescan_cycles", 8)
        d(self, "fallback_radius_min_m", 0.15)
        d(self, "fallback_radius_max_m", 0.8)

        self._logged_map_tf_diag = False
        self._no_frontier_soft_retries = 0
        self._pause_is_no_goal_retry = False
        self._st = _St.START
        self._map: Optional[OccupancyGrid] = None
        self._blacklist: List[Tuple[float, float, float]] = []
        self._send_future: Optional[Future] = None
        self._goal_handle = None
        self._result_future: Optional[Future] = None
        self._last_goal: Optional[Tuple[float, float]] = None
        self._fail_streak = 0
        self._rescan_count = 0
        self._pending_goal: Optional[Tuple[float, float]] = None
        self._did_initial_spin = False
        self._spin_accum = 0.0
        self._spin_t0 = 0.0
        self._last_odom_yaw: Optional[float] = None
        self._map_after_spin = 0.0
        self._goal_deadline = 0.0
        self._start_at = self.get_clock().now() + Duration(
            seconds=float(self.get_parameter("start_delay_sec").value)
        )
        self._nav_wait_t0 = 0.0
        self._nav_log_t0 = 0.0
        self._wait_log_t0 = 0.0
        self._lc_order: Tuple[str, ...] = ("controller_server", "planner_server", "bt_navigator")
        self._lc_clients: Dict[str, object] = {}
        self._lc_idx = 0
        self._lc_fut: Optional[Future] = None

        self._tf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        tf2_ros.TransformListener(self._tf, self, spin_thread=False)

        nav_topic = str(self.get_parameter("navigate_action").value)
        self._nav = ActionClient(self, NavigateToPose, nav_topic, callback_group=self._cb)
        self._cmd = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self._pub_stat = self.create_publisher(String, "/frontier_explorer/status", FRONTIER_TOPIC_QOS)
        self._pub_goal = self.create_publisher(PoseStamped, "/frontier_explorer/current_goal", FRONTIER_TOPIC_QOS)
        self._pub_mk = self.create_publisher(MarkerArray, "/frontier_explorer/markers", 10)

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            MAP_QOS,
            callback_group=self._cb,
        )
        for nm in self._lc_order:
            self._lc_clients[nm] = self.create_client(
                GetState, f"/{nm}/get_state", callback_group=self._cb
            )

        hz = max(0.2, float(self.get_parameter("exploration_rate_hz").value))
        self.create_timer(1.0 / hz, self._tick, callback_group=self._tick_g)
        self._status = ""

        mt = str(self.get_parameter("map_topic").value)
        cv = str(self.get_parameter("cmd_vel_topic").value)
        bf = str(self.get_parameter("base_frame").value)
        mf = str(self.get_parameter("map_frame").value)
        nav = str(self.get_parameter("navigate_action").value)
        lg = self.get_logger()
        lg.info("[frontier_explorer] STARTED (V1)")
        lg.info("[frontier_explorer] Waiting for map / TF / Nav2 via state machine (see /frontier_explorer/status)")
        lg.info(
            "[frontier_explorer] config: "
            f"map_topic={mt} map_frame={mf} base_frame={bf} cmd_vel_topic={cv} navigate_action={nav}"
        )
        self._stat("STARTING")

    def _maybe_log_map_robot_diag(self, rx: float, ry: float) -> None:
        """One-shot debug: map extents vs robot pose (helps Nav2 \"out of costmap\" triage)."""
        if self._logged_map_tf_diag or self._map is None:
            return
        info = self._map.info
        w, h = int(info.width), int(info.height)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        mx, my = world_to_map(rx, ry, ox, oy, res)
        inside = 0 <= mx < w and 0 <= my < h
        self.get_logger().info(
            "[frontier_explorer] map_diag "
            f"origin_xy=({ox:.2f},{oy:.2f}) size_wh=({w},{h}) resolution={res:.4f} | "
            f"robot_world_xy=({rx:.2f},{ry:.2f}) robot_cell_ij=({mx},{my}) "
            f"robot_inside_map={inside}"
        )
        self._logged_map_tf_diag = True

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _stat(self, s: str) -> None:
        if s == self._status:
            return
        self.get_logger().info(f"[frontier_explorer] {s}")
        self._status = s
        self._pub_stat.publish(String(data=s))

    def _pose_map(self) -> Optional[Tuple[float, float, float]]:
        mf = str(self.get_parameter("map_frame").value)
        bf = str(self.get_parameter("base_frame").value)
        try:
            t = self._tf.lookup_transform(mf, bf, Time(), timeout=Duration(seconds=0.25))
            tr = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return tr.x, tr.y, yaw
        except tf2_ros.TransformException:
            return None

    def _tf_ok(self, a: str, b: str) -> bool:
        try:
            self._tf.lookup_transform(a, b, Time(), timeout=Duration(seconds=0.2))
            return True
        except tf2_ros.TransformException:
            return False

    def _odom_yaw(self) -> Optional[float]:
        bf = str(self.get_parameter("base_frame").value)
        try:
            t = self._tf.lookup_transform("odom", bf, Time(), timeout=Duration(seconds=0.15))
            q = t.transform.rotation
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except tf2_ros.TransformException:
            return None

    def _cmd_ok(self) -> bool:
        if not bool(self.get_parameter("require_cmd_vel_subscriber").value):
            return True
        t = str(self.get_parameter("cmd_vel_topic").value)
        return self.count_subscribers(t) >= 1

    def _prune_bl(self) -> None:
        now = time.monotonic()
        to = float(self.get_parameter("blacklist_timeout_sec").value)
        self._blacklist = [(x, y, tt) for x, y, tt in self._blacklist if now - tt < to]

    def _lc_step(self) -> bool:
        if not bool(self.get_parameter("require_nav_lifecycle_active").value):
            return True
        if self._lc_idx >= len(self._lc_order):
            return True
        nm = self._lc_order[self._lc_idx]
        cl = self._lc_clients[nm]
        if not cl.service_is_ready():  # type: ignore[union-attr]
            return False
        if self._lc_fut is None:
            self._lc_fut = cl.call_async(GetState.Request())  # type: ignore[union-attr]
        if not self._lc_fut.done():
            return False
        res = self._lc_fut.result()
        self._lc_fut = None
        ok = res is not None and int(res.current_state.id) == State.PRIMARY_STATE_ACTIVE
        if not ok:
            self._lc_idx = 0
            return False
        self._lc_idx += 1
        return self._lc_idx >= len(self._lc_order)

    def _markers(self, clusters, best: Optional[ValidatedGoal]) -> None:
        if not bool(self.get_parameter("publish_markers").value) or self._map is None:
            return
        info = self._map.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        frame = str(self.get_parameter("map_frame").value)
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        c = Marker()
        c.header.frame_id = frame
        c.header.stamp = now
        c.ns = "clear"
        c.action = Marker.DELETEALL
        arr.markers.append(c)
        pts = Marker()
        pts.header.frame_id = frame
        pts.header.stamp = now
        pts.ns = "frontier"
        pts.id = 1
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        pts.scale.x = res * 0.4
        pts.scale.y = res * 0.4
        pts.color = ColorRGBA(r=0.7, g=0.7, b=0.2, a=0.85)
        for cl in clusters:
            for cx, cy in cl.cells:
                p = Point()
                p.x = ox + (cx + 0.5) * res
                p.y = oy + (cy + 0.5) * res
                p.z = 0.04
                pts.points.append(p)
        arr.markers.append(pts)
        if best is not None:
            a = Marker()
            a.header.frame_id = frame
            a.header.stamp = now
            a.ns = "goal"
            a.id = 2
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.scale.x = 0.35
            a.scale.y = 0.06
            a.scale.z = 0.06
            a.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9)
            a.pose.position.x = best.wx
            a.pose.position.y = best.wy
            a.pose.position.z = 0.05
            a.pose.orientation = _yaw_to_q(best.yaw)
            arr.markers.append(a)
        self._pub_mk.publish(arr)

    def _blacklist_covers(self, wx: float, wy: float) -> bool:
        br = float(self.get_parameter("blacklist_radius").value)
        for bx, by, _t in self._blacklist:
            if math.hypot(wx - bx, wy - by) < br:
                return True
        return False

    def _pick_goal(self) -> Tuple[Optional[ValidatedGoal], str]:
        if self._map is None:
            return None, "no_map"
        pose = self._pose_map()
        if pose is None:
            return None, "no_tf"
        rx, ry, _ = pose
        info = self._map.info
        w, h = int(info.width), int(info.height)
        if w <= 0 or h <= 0:
            return None, "empty"

        lg = self.get_logger()
        hdr = self._map.header
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        occ = int(self.get_parameter("occupied_threshold").value)
        free = int(self.get_parameter("free_threshold").value)
        unk = int(self.get_parameter("unknown_value").value)
        mins = int(self.get_parameter("min_frontier_cluster_size").value)

        data = sanitize_occ_grid_data(self._map.data)
        lg.info(
            format_map_debug_header(
                w, h, res, ox, oy, hdr.frame_id, int(hdr.stamp.sec), int(hdr.stamp.nanosec), 0
            )
        )
        rb_line, rob_ixy, _inside = robot_cell_debug_line(data, w, h, rx, ry, ox, oy, res)
        lg.info(rb_line)
        unk_c, free_c, occ_c, oth_c = classify_map_cells(data, unk, free, occ)
        lg.info(format_map_stats_line(w, h, unk_c, free_c, occ_c, oth_c))
        lg.info(map_value_range_str(data))
        lg.info(
            local_radius_stats(
                data, w, h, rx, ry, ox, oy, res, unk, free, occ, (0.5, 1.0, 2.0, 3.0)
            )
        )

        raw_frontier_cells = count_raw_frontier_cells(data, w, h, unk, free, occ)
        clusters = collect_frontier_clusters(data, w, h, unk, free, occ, mins)
        n_cl = len(clusters)

        passable, _ = build_passable_mask(
            data,
            w,
            h,
            unk,
            occ,
            free,
            bool(self.get_parameter("unknown_as_blocked_in_planner_grid").value),
            float(self.get_parameter("goal_inflation_radius_m").value),
            res,
        )
        p_st = passable
        if bool(self.get_parameter("enable_staging_goal").value):
            p_st, _ = build_passable_mask(
                data,
                w,
                h,
                unk,
                occ,
                free,
                bool(self.get_parameter("unknown_as_blocked_in_planner_grid").value),
                float(self.get_parameter("goal_inflation_radius_m").value)
                + float(self.get_parameter("staging_clearance_m").value),
                res,
            )

        pass_bfs = list(passable)
        pass_st_bfs = list(p_st)
        used_clear = False
        if bool(self.get_parameter("allow_robot_seed_clearing").value):
            cr = float(self.get_parameter("robot_seed_clear_radius_m").value)
            pass_bfs = apply_robot_footprint_clearing(pass_bfs, rx, ry, ox, oy, res, w, h, cr)
            pass_st_bfs = apply_robot_footprint_clearing(pass_st_bfs, rx, ry, ox, oy, res, w, h, cr)
            used_clear = True
            lg.info(
                f"SEED_FALLBACK robot footprint clearing used radius={cr:.2f}m "
                "(local BFS mask only; /map not modified)"
            )

        robot_ixy = world_to_map(rx, ry, ox, oy, res)
        bfs_r = float(self.get_parameter("bfs_seed_search_radius_m").value)
        bfs_step = float(self.get_parameter("bfs_seed_search_step_m").value)
        seed, rm, rs, sd_m, smeth = resolve_bfs_seed_and_masks(
            robot_ixy,
            (rx, ry),
            data,
            w,
            h,
            pass_bfs,
            pass_st_bfs,
            ox,
            oy,
            res,
            unk,
            free,
            occ,
            bfs_r,
            bfs_step,
        )
        lg.info(
            format_reachability_debug_line(
                (rx, ry),
                robot_ixy,
                data,
                w,
                h,
                passable,
                p_st,
                ox,
                oy,
                res,
                occ,
                free,
                unk,
                seed,
                sd_m,
                smeth,
                rm,
                rs,
                1.0,
            )
        )

        if seed is None:
            lg.error("NO_BFS_SEED: no passable BFS seed within configured radii")
            if free_c > 0:
                gf, gd = global_nearest_free_cell(rx, ry, data, w, h, ox, oy, res, unk, free, occ)
                if gf is not None:
                    lg.info(
                        f"nearest_free_global cell={gf} distance_m={gd:.2f} "
                        f"(map has free cells but none reachable as passable_bfs near robot)"
                    )
            elif free_c == 0:
                lg.warning("no_free_cells: MAP_STATS free_count=0")

        if not clusters:
            reason = "no_raw_frontiers" if raw_frontier_cells == 0 else "no_clusters"
            lg.info(
                "FRONTIER_DEBUG "
                f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance=0 "
                f"after_reachable=0 after_blacklist=0 reason={reason}"
            )
            self._markers([], None)
            return None, "no_clusters"

        if seed is None:
            self._markers(clusters, None)
            lg.info(
                "FRONTIER_DEBUG "
                f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance=0 "
                f"after_reachable=0 after_blacklist=0 reason=no_seed_near_robot"
            )
            return None, "no_seed"

        sw = float(self.get_parameter("size_weight").value)
        ranked = sorted(
            clusters,
            key=lambda cl: score_cluster_distance(cl, (rx, ry), ox, oy, res, sw),
        )
        self._prune_bl()
        br = float(self.get_parameter("blacklist_radius").value)
        min_d = float(self.get_parameter("min_goal_distance").value)
        max_d = min(
            float(self.get_parameter("max_goal_distance").value),
            float(self.get_parameter("max_goal_distance_stage_m").value),
        )
        ret = int(self.get_parameter("retreat_from_frontier_steps").value)
        fan_a = self.get_parameter("staging_fan_angles_deg").value
        fan_d = self.get_parameter("staging_fan_distances_m").value
        fa = tuple(float(x) for x in fan_a) if isinstance(fan_a, (list, tuple)) else (0.0,)
        fd = tuple(float(x) for x in fan_d) if isinstance(fan_d, (list, tuple)) else (0.3,)

        after_distance = 0
        after_blacklist = 0
        after_reachable = 0
        n_blk_rej = 0
        n_reach_fail = 0

        for cl in ranked:
            ccx, ccy = cl.centroid_map
            cwx = ox + (ccx + 0.5) * res
            cwy = oy + (ccy + 0.5) * res
            cd = math.hypot(cwx - rx, cwy - ry)
            if cd < min_d or cd > max_d:
                continue
            after_distance += 1
            if self._blacklist_covers(cwx, cwy):
                n_blk_rej += 1
                continue
            after_blacklist += 1

            vg, rj, det = validate_and_build_goal(
                cl,
                data,
                w,
                h,
                (rx, ry),
                robot_ixy,
                ox,
                oy,
                res,
                passable,
                occ,
                free,
                unk,
                self._blacklist,
                br,
                min_d,
                max_d,
                ret,
                float(self.get_parameter("fallback_radius_min_m").value),
                float(self.get_parameter("fallback_radius_max_m").value),
                bool(self.get_parameter("enable_staging_goal").value),
                float(self.get_parameter("staging_min_distance_m").value),
                float(self.get_parameter("staging_max_distance_m").value),
                fa,
                fd,
                bool(self.get_parameter("staging_require_free_value_zero").value),
                p_st,
                rm,
                rs,
            )
            if vg is not None:
                after_reachable += 1
                lg.info(
                    "FRONTIER_DEBUG "
                    f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance={after_distance} "
                    f"after_reachable={after_reachable} after_blacklist={after_blacklist} "
                    f"reason=goal_selected used_seed_clearing={used_clear}"
                )
                self._markers(clusters, vg)
                return vg, ""
            if rj is not None:
                if det == "not_reachable":
                    n_reach_fail += 1
                lg.info(f"reject cluster: {rj.value} {det}")

        fr_reason = "no_reachable_frontier"
        if after_distance == 0:
            fr_reason = "no_frontier_in_distance_band"
        elif after_blacklist == 0 and n_blk_rej > 0:
            fr_reason = "all_blacklisted"
        elif after_reachable == 0 and n_reach_fail > 0:
            fr_reason = "no_reachable_frontier"

        lg.info(
            "FRONTIER_DEBUG "
            f"raw_cells={raw_frontier_cells} clusters={n_cl} after_distance={after_distance} "
            f"after_reachable={after_reachable} after_blacklist={after_blacklist} reason={fr_reason}"
        )
        self._markers(clusters, None)
        return None, "no_valid"

    def _zero(self) -> None:
        try:
            self._cmd.publish(Twist())
        except Exception:
            pass

    def _done(self, msg: str) -> None:
        self._st = _St.DONE
        self._zero()
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._stat(msg)

    def _tick(self) -> None:
        try:
            self._tick_impl()
        except Exception:
            self.get_logger().exception("frontier_explorer tick error")
            self._zero()
            self._done("EXCEPTION")

    def _tick_impl(self) -> None:
        if not rclpy.ok() or self._st == _St.DONE:
            return
        if self.get_clock().now() < self._start_at:
            self._stat("WAIT_START_DELAY")
            return

        if self._st == _St.START:
            mp = str(self.get_parameter("map_topic").value)
            if bool(self.get_parameter("require_map_publisher_for_ready").value):
                if self.count_publishers(mp) < 1:
                    self._throttle_log(f"wait publisher {mp}")
                    self._stat("WAIT_MAP_PUBLISHER")
                    return
            self._st = _St.WAIT_MAP
            return

        if self._st == _St.WAIT_MAP:
            if self._map_after_spin > 0.0:
                self._zero()
                if time.monotonic() < self._map_after_spin:
                    self._stat("WAIT_MAP_AFTER_SPIN")
                    return
                self._map_after_spin = 0.0
                self._begin_nav2()
                return
            if self._map is None or len(self._map.data) == 0:
                self._throttle_log("wait /map data")
                self._stat("WAITING_FOR_MAP")
                return
            w, h = int(self._map.info.width), int(self._map.info.height)
            if w <= 0 or h <= 0:
                self._stat("WAITING_FOR_MAP")
                return
            mf = str(self.get_parameter("map_frame").value)
            bf = str(self.get_parameter("base_frame").value)
            if not self._tf_ok(mf, "odom") or not self._tf_ok("odom", bf):
                self._throttle_log("wait TF map->odom->base")
                self._st = _St.WAIT_TF
                self._stat("WAITING_FOR_TF")
                return
            pm = self._pose_map()
            if pm is not None:
                self._maybe_log_map_robot_diag(pm[0], pm[1])
            if (
                bool(self.get_parameter("integrated_initial_spin").value)
                and not self._did_initial_spin
            ):
                self._st = _St.SPIN
                self._last_odom_yaw = None
                self._spin_accum = 0.0
                self._stat("INITIAL_SPIN")
                return
            self._begin_nav2()
            return

        if self._st == _St.WAIT_TF:
            mf = str(self.get_parameter("map_frame").value)
            bf = str(self.get_parameter("base_frame").value)
            if not self._tf_ok(mf, "odom") or not self._tf_ok("odom", bf):
                self._throttle_log("wait TF map->base_footprint")
                self._stat("WAITING_FOR_TF")
                return
            pm = self._pose_map()
            if pm is not None:
                self._maybe_log_map_robot_diag(pm[0], pm[1])
            if (
                bool(self.get_parameter("integrated_initial_spin").value)
                and not self._did_initial_spin
            ):
                self._st = _St.SPIN
                self._last_odom_yaw = None
                self._spin_accum = 0.0
                self._stat("INITIAL_SPIN")
                return
            self._begin_nav2()
            return

        if self._st == _St.SPIN:
            if not self._cmd_ok():
                t = str(self.get_parameter("cmd_vel_topic").value)
                self._throttle_log(f"waiting for subscriber on {t} before initial spin")
                self._stat("WAIT_CMD_VEL_SUB")
                return
            y = self._odom_yaw()
            nowt = time.monotonic()
            if y is None:
                return
            if self._last_odom_yaw is None:
                self._last_odom_yaw = y
                self._spin_accum = 0.0
                self._spin_t0 = nowt
            dy = _norm(y - self._last_odom_yaw)
            self._spin_accum += abs(dy)
            self._last_odom_yaw = y
            tgt = float(self.get_parameter("initial_spin_target_rad").value)
            mn = float(self.get_parameter("initial_spin_min_duration_sec").value)
            mx = float(self.get_parameter("initial_spin_max_duration_sec").value)
            az = float(self.get_parameter("initial_spin_angular_z").value)
            tw = Twist()
            tw.angular.z = az
            self._cmd.publish(tw)
            if (self._spin_accum >= tgt and (nowt - self._spin_t0) >= mn) or (nowt - self._spin_t0) >= mx:
                self._zero()
                self._did_initial_spin = True
                self._map_after_spin = nowt + float(self.get_parameter("map_settle_after_spin_sec").value)
                self._st = _St.WAIT_MAP
                return

        if self._st == _St.WAIT_NAV2:
            if not bool(self.get_parameter("wait_for_nav2").value):
                self._st = _St.SELECT
                return
            nowm = time.monotonic()
            if self._nav_wait_t0 <= 0.0:
                self._nav_wait_t0 = nowm
            tout = float(self.get_parameter("nav2_ready_timeout_sec").value)
            if nowm - self._nav_wait_t0 > tout:
                self.get_logger().error("Nav2 ready timeout")
                self._zero()
                self._done("NAV2_TIMEOUT")
                return
            iv = float(self.get_parameter("nav2_debug_log_interval_sec").value)
            if nowm - self._nav_log_t0 >= iv:
                self._nav_log_t0 = nowm
                self.get_logger().info(
                    f"wait Nav2 action={self._nav.server_is_ready()} lc_idx={self._lc_idx}"
                )
            if not (self._nav.server_is_ready() or self._nav.wait_for_server(timeout_sec=0.0)):
                self._stat("WAITING_FOR_NAV2")
                self._lc_idx = 0
                self._lc_fut = None
                return
            if bool(self.get_parameter("require_nav_lifecycle_active").value):
                if not self._lc_step():
                    nm = self._lc_order[min(self._lc_idx, len(self._lc_order) - 1)]
                    self._stat("WAITING_FOR_NAV2_LC")
                    self._throttle_log(f"waiting for Nav2 lifecycle active: {nm}")
                    return
            self.get_logger().info("[frontier_explorer] Nav2 reachable and lifecycle ACTIVE — READY")
            self._lc_idx = 0
            self._lc_fut = None
            self._st = _St.SELECT
            self._stat("NAV2_ACTIVE")
            return

        if self._st == _St.SELECT:
            self._stat("SEARCHING_FRONTIER")
            g, why = self._pick_goal()
            if g is None:
                soft_whys = {"no_clusters", "no_seed", "no_valid"}
                retry_nf = bool(self.get_parameter("retry_when_no_frontier").value)
                max_nf = int(self.get_parameter("max_no_frontier_retries").value)
                delay_nf = float(self.get_parameter("no_frontier_retry_delay_sec").value)

                use_soft = retry_nf and why in soft_whys and (
                    max_nf == 0 or self._no_frontier_soft_retries < max_nf
                )

                if use_soft:
                    self._no_frontier_soft_retries += 1
                    self.get_logger().warn(
                        "No goal (%s) — soft retry pause %.1fs (%s/%s)"
                        % (
                            why,
                            delay_nf,
                            self._no_frontier_soft_retries,
                            "∞" if max_nf == 0 else str(max_nf),
                        )
                    )
                    self._fail_streak += 1
                    self._st = _St.PAUSE
                    self._goal_deadline = time.monotonic() + delay_nf
                    self._pause_is_no_goal_retry = True
                    self._stat("SOFT_RETRY_NO_FRONTIER")
                    return

                self._no_frontier_soft_retries = 0
                self._rescan_count += 1
                if self._rescan_count > int(self.get_parameter("max_rescan_cycles").value):
                    self._done("NO_VALID_FRONTIER")
                    return
                self.get_logger().warn(f"No goal ({why}) — recovery pause")
                self._fail_streak += 1
                self._st = _St.PAUSE
                self._goal_deadline = time.monotonic() + float(
                    self.get_parameter("recovery_pause_sec").value
                )
                self._pause_is_no_goal_retry = False
                self._stat("RECOVER_NO_GOAL")
                return
            self._rescan_count = 0
            self._no_frontier_soft_retries = 0
            goal = NavigateToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = str(self.get_parameter("map_frame").value)
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = g.wx
            ps.pose.position.y = g.wy
            ps.pose.position.z = 0.0
            ps.pose.orientation = _yaw_to_q(g.yaw)
            goal.pose = ps
            self._pending_goal = (g.wx, g.wy)
            self._pub_goal.publish(ps)
            self._send_future = self._nav.send_goal_async(goal)
            self._st = _St.SEND
            self._stat("GOAL_SELECTED")
            return

        if self._st == _St.SEND:
            if self._send_future is None or not self._send_future.done():
                return
            gh = self._send_future.result()
            self._send_future = None
            if gh is None or not gh.accepted:
                self._last_goal = self._pending_goal
                self._pending_goal = None
                self._blacklist_goal()
                self._fail_streak += 1
                self._maybe_recover()
                self._stat("GOAL_FAILED")
                return
            self._goal_handle = gh
            self._result_future = gh.get_result_async()
            self._st = _St.NAV
            self._goal_deadline = time.monotonic() + float(
                self.get_parameter("goal_timeout_sec").value
            )
            self._last_goal = self._pending_goal
            self._pending_goal = None
            self._stat("NAVIGATING")
            return

        if self._st == _St.NAV:
            if self._result_future is not None and self._result_future.done():
                wrapped = self._result_future.result()
                self._goal_handle = None
                self._result_future = None
                st = int(wrapped.status) if wrapped is not None else -1
                if st == GoalStatus.STATUS_SUCCEEDED:
                    self._fail_streak = 0
                    self._st = _St.SELECT
                    self._stat("GOAL_SUCCEEDED")
                    return
                self.get_logger().warn(
                    "[frontier_explorer] NavigateToPose ended without success "
                    f"(status={st}); blacklisting goal and retrying"
                )
                self._blacklist_goal()
                self._fail_streak += 1
                self._stat("GOAL_FAILED")
                self._maybe_recover()
                return
            if time.monotonic() > self._goal_deadline:
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._goal_handle = None
                self._result_future = None
                self._blacklist_goal()
                self._fail_streak += 1
                self._maybe_recover()
                self._stat("GOAL_FAILED")
            return

        if self._st == _St.PAUSE:
            self._zero()
            if time.monotonic() < self._goal_deadline:
                return
            self._pause_is_no_goal_retry = False
            self._st = _St.SELECT
            return

    def _blacklist_goal(self) -> None:
        if self._last_goal is None:
            return
        self._blacklist.append((self._last_goal[0], self._last_goal[1], time.monotonic()))

    def _maybe_recover(self) -> None:
        lim = int(self.get_parameter("consecutive_fail_limit").value)
        if self._fail_streak >= lim:
            self._goal_deadline = time.monotonic() + float(
                self.get_parameter("recovery_pause_sec").value
            )
            self._st = _St.PAUSE
            self._pause_is_no_goal_retry = False
            self._stat("RECOVERY_FAIL_STREAK")
            return
        self._st = _St.SELECT

    def _begin_nav2(self) -> None:
        self._nav_wait_t0 = 0.0
        self._nav_log_t0 = 0.0
        self._lc_idx = 0
        self._lc_fut = None
        self._st = _St.WAIT_NAV2

    def _throttle_log(self, msg: str) -> None:
        now = time.monotonic()
        iv = max(5.0, float(self.get_parameter("wait_log_interval_sec").value))
        if now - self._wait_log_t0 >= iv:
            self._wait_log_t0 = now
            self.get_logger().info(msg)

    def shutdown_stop(self) -> None:
        self._done("SHUTDOWN")


def main() -> int:
    rclpy.init()
    node = None
    ex = None
    try:
        node = FrontierExplorer()
        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not is_shutdown_exception(exc):
            raise
    finally:
        if ex is not None:
            try:
                ex.shutdown(timeout_sec=0.5)
            except Exception:
                pass
        if node is not None:
            try:
                node.shutdown_stop()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        safe_shutdown()
    return 0


if __name__ == "__main__":
    main()
