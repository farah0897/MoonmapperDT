"""Logger dekningsrutenett i odom-frame (observerer kun, styrer ikke robot)."""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, Int32, String


class CoverageLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_logger_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("obstacle_state_topic", "/obstacle/current_state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("subscribe_optional_topics", True)
        self.declare_parameter("grid_frame", "odom")
        self.declare_parameter("grid_size_m", 10.0)
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("min_movement_for_distance", 0.005)
        self.declare_parameter("mission_duration_sec", 120.0)
        self.declare_parameter("stuck_check_window_sec", 10.0)
        self.declare_parameter("stuck_min_displacement_m", 0.05)
        # Effektiv «halv sporvidde» for å telle distanse ved ren rotasjon (odom x/y endrer seg lite).
        self.declare_parameter("spin_turn_radius_m", 0.18)
        # true: BEST_EFFORT på odom (matcher ofte ros2_control / Gazebo).
        self.declare_parameter("odom_use_best_effort_qos", True)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._obs_topic = str(self.get_parameter("obstacle_state_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._sub_opt = bool(self.get_parameter("subscribe_optional_topics").value)
        self._frame = str(self.get_parameter("grid_frame").value)
        self._size_m = float(self.get_parameter("grid_size_m").value)
        self._res = float(self.get_parameter("resolution").value)
        if self._res <= 0.0 or self._size_m <= 0.0:
            raise ValueError("coverage_logger: resolution og grid_size_m må være > 0")

        self._w = int(round(self._size_m / self._res))
        self._h = self._w
        self._total_cells = self._w * self._h
        self._cells: List[int] = [0] * self._total_cells
        self._visited_count = 0

        self._min_move = float(self.get_parameter("min_movement_for_distance").value)
        self._mission_dur = float(self.get_parameter("mission_duration_sec").value)
        self._stuck_window = float(self.get_parameter("stuck_check_window_sec").value)
        self._stuck_min_disp = float(self.get_parameter("stuck_min_displacement_m").value)
        self._spin_r = float(self.get_parameter("spin_turn_radius_m").value)
        self._odom_be = bool(self.get_parameter("odom_use_best_effort_qos").value)

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._have_origin = False
        self._prev_x: Optional[float] = None
        self._prev_y: Optional[float] = None
        self._distance_traveled = 0.0
        self._mission_start: Optional[Time] = None
        self._pos_hist: Deque[Tuple[float, float, float]] = deque()
        self._last_odom_stamp: Optional[Time] = None
        self._odom_rx_count = 0
        self._no_odom_warned = False

        self._last_obstacle_state = ""

        hz = float(self.get_parameter("publish_rate_hz").value)
        if hz <= 0.0:
            hz = 1.0

        self._pub_grid = self.create_publisher(OccupancyGrid, "/coverage/grid", 1)
        self._pub_percent = self.create_publisher(Float32, "/coverage/percent", 10)
        self._pub_visited = self.create_publisher(Int32, "/coverage/visited_cells", 10)
        self._pub_dist = self.create_publisher(Float32, "/coverage/distance_traveled", 10)
        self._pub_cell = self.create_publisher(String, "/coverage/current_cell", 10)
        self._pub_status = self.create_publisher(String, "/coverage/status", 10)

        qos_odom = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_qos = qos_odom if self._odom_be else 10
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, odom_qos)
        if self._sub_opt:
            self.create_subscription(String, self._obs_topic, self._on_obstacle, 10)
            self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 10)

        self.create_timer(1.0 / hz, self._publish_coverage)
        self.create_timer(5.0, self._check_odom_flow)

        self.get_logger().info(
            f"coverage_logger: {self._w}x{self._h} celler @ {self._res} m, "
            f"frame={self._frame}, odom={self._odom_topic}"
        )

        # ROS 2 viser ikke topic-type før første publish; én umiddelbar publish hjelper `topic echo`.
        self._publish_coverage()

    def _odom_time(self, msg: Odometry) -> Time:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return self.get_clock().now()
        return Time.from_msg(msg.header.stamp)

    def _check_odom_flow(self) -> None:
        if self._odom_rx_count > 0:
            self._no_odom_warned = True
            return
        if self._no_odom_warned:
            return
        self._no_odom_warned = True
        self.get_logger().warning(
            f"Ingen /odom-mottak etter 5 s på {self._odom_topic}. "
            "Sjekk at sim + cmd_vel_odom_relay kjører, eller sett odom_topic / "
            "odom_use_best_effort_qos."
        )

    def _on_obstacle(self, msg: String) -> None:
        self._last_obstacle_state = msg.data

    def _on_cmd(self, msg: Twist) -> None:
        _ = (msg.linear.x, msg.angular.z)

    def _world_to_ix_iy(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        if not self._have_origin:
            return None
        ix = int(math.floor((wx - self._origin_x) / self._res))
        iy = int(math.floor((wy - self._origin_y) / self._res))
        if 0 <= ix < self._w and 0 <= iy < self._h:
            return ix, iy
        return None

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_rx_count += 1
        now = self.get_clock().now()
        if self._mission_start is None:
            self._mission_start = now

        stamp = self._odom_time(msg)
        dt = 0.0
        if self._last_odom_stamp is not None:
            dt = (stamp - self._last_odom_stamp).nanoseconds * 1e-9
        self._last_odom_stamp = stamp

        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)

        if not self._have_origin:
            half = self._size_m * 0.5
            self._origin_x = px - half
            self._origin_y = py - half
            self._have_origin = True
            self.get_logger().info(
                f"coverage grid origin: ({self._origin_x:.3f}, {self._origin_y:.3f}), "
                f"sentrum start ({px:.3f}, {py:.3f})"
            )

        cell = self._world_to_ix_iy(px, py)
        if cell is not None:
            ix, iy = cell
            idx = iy * self._w + ix
            if self._cells[idx] == 0:
                self._cells[idx] = 100
                self._visited_count += 1

        seg_xy = 0.0
        if self._prev_x is not None and self._prev_y is not None:
            seg_xy = math.hypot(px - self._prev_x, py - self._prev_y)

        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)
        seg_twist = 0.0
        if self._prev_x is not None and 0.0 < dt < 2.0:
            seg_twist = max(abs(vx) * dt, abs(wz) * self._spin_r * dt)

        if self._prev_x is not None:
            if seg_xy >= self._min_move:
                self._distance_traveled += seg_xy
            elif seg_twist > 0.0 and (
                abs(vx) > 0.008 or abs(wz) > 0.04 or seg_xy > 1e-6
            ):
                self._distance_traveled += max(seg_xy, seg_twist)

        self._prev_x, self._prev_y = px, py

        tsec = now.nanoseconds * 1e-9
        self._pos_hist.append((tsec, px, py))
        while self._pos_hist and self._pos_hist[0][0] < tsec - self._stuck_window:
            self._pos_hist.popleft()

    def _compute_status(self) -> str:
        now = self.get_clock().now()
        if self._mission_start is not None:
            elapsed = (now - self._mission_start).nanoseconds * 1e-9
            if elapsed >= self._mission_dur:
                return "FINISHED"

        if len(self._pos_hist) >= 2:
            t0, x0, y0 = self._pos_hist[0]
            t1, x1, y1 = self._pos_hist[-1]
            if (t1 - t0) >= self._stuck_window - 1e-6:
                disp = math.hypot(x1 - x0, y1 - y0)
                if disp < self._stuck_min_disp:
                    return "STUCK_WARNING"
        return "RUNNING"

    def _publish_coverage(self) -> None:
        status = self._compute_status()
        st_msg = String()
        st_msg.data = status
        self._pub_status.publish(st_msg)

        if not self._have_origin:
            pc = String()
            pc.data = "pending"
            self._pub_cell.publish(pc)
            pm = Float32()
            pm.data = 0.0
            self._pub_percent.publish(pm)
            vm = Int32()
            vm.data = 0
            self._pub_visited.publish(vm)
            dm = Float32()
            dm.data = 0.0
            self._pub_dist.publish(dm)
            self._publish_empty_grid()
            return

        if self._prev_x is None:
            self._publish_empty_grid()
            return

        cell = self._world_to_ix_iy(self._prev_x, self._prev_y)
        cmsg = String()
        if cell is None:
            cmsg.data = "out_of_grid"
        else:
            cmsg.data = f"{cell[0]},{cell[1]}"
        self._pub_cell.publish(cmsg)

        pct = 100.0 * float(self._visited_count) / float(max(1, self._total_cells))
        pm = Float32()
        pm.data = float(pct)
        self._pub_percent.publish(pm)

        vm = Int32()
        vm.data = int(self._visited_count)
        self._pub_visited.publish(vm)

        dm = Float32()
        dm.data = float(self._distance_traveled)
        self._pub_dist.publish(dm)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info.resolution = float(self._res)
        grid.info.width = self._w
        grid.info.height = self._h
        grid.info.origin.position.x = float(self._origin_x)
        grid.info.origin.position.y = float(self._origin_y)
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = list(self._cells)
        self._pub_grid.publish(grid)

    def _publish_empty_grid(self) -> None:
        """Publiser tomt kart (alle 0) med kjent oppløsning slik at RViz/topic list ser /coverage/grid."""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info.resolution = float(self._res)
        grid.info.width = self._w
        grid.info.height = self._h
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * self._total_cells
        self._pub_grid.publish(grid)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CoverageLoggerNode()
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
