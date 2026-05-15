"""Enkel goal-follower: /goal_pose -> output_cmd_topic (standard /cmd_vel_raw, eller /cmd_vel_goal).

Strict rotate-then-drive: ingen negativ linear.x, ingen buet kjøring ved stor heading-feil.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class SimpleGoalFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_goal_follower_node")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("rotate_enter_angle_rad", 0.45)
        self.declare_parameter("rotate_exit_angle_rad", 0.22)
        self.declare_parameter("drive_stop_angle_rad", 0.55)
        self.declare_parameter("goal_tolerance_m", 0.15)

        self.declare_parameter("max_forward_speed", 0.12)
        self.declare_parameter("min_forward_speed", 0.025)
        self.declare_parameter("distance_slow_radius", 0.70)
        self.declare_parameter("max_turn_speed", 0.55)
        self.declare_parameter("rotate_angular_speed", 0.55)
        self.declare_parameter("yaw_deadband_rad", 0.04)
        self.declare_parameter("yaw_kp", 0.8)

        self.declare_parameter("base_forward_yaw_offset_rad", math.pi)
        self.declare_parameter("angular_sign", 1.0)
        self.declare_parameter("linear_sign", 1.0)

        self.declare_parameter("command_timeout_sec", 0.5)

        self._fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self._out_topic = self.get_parameter("output_cmd_topic").get_parameter_value().string_value
        rate = float(self.get_parameter("control_rate_hz").value)
        self._period = 1.0 / rate if rate > 1e-6 else 0.1

        self._rotate_enter = float(self.get_parameter("rotate_enter_angle_rad").value)
        self._rotate_exit = float(self.get_parameter("rotate_exit_angle_rad").value)
        self._drive_stop = float(self.get_parameter("drive_stop_angle_rad").value)
        self._goal_tol = float(self.get_parameter("goal_tolerance_m").value)

        self._max_forward = float(self.get_parameter("max_forward_speed").value)
        self._min_forward = float(self.get_parameter("min_forward_speed").value)
        self._slow_radius = float(self.get_parameter("distance_slow_radius").value)
        self._max_turn = float(self.get_parameter("max_turn_speed").value)
        self._rotate_cap = float(self.get_parameter("rotate_angular_speed").value)
        self._yaw_deadband = float(self.get_parameter("yaw_deadband_rad").value)
        self._yaw_kp = float(self.get_parameter("yaw_kp").value)
        self._yaw_offset = float(self.get_parameter("base_forward_yaw_offset_rad").value)
        self._angular_sign = float(self.get_parameter("angular_sign").value)
        self._linear_sign = float(self.get_parameter("linear_sign").value)
        self._tf_timeout = float(self.get_parameter("command_timeout_sec").value)

        if self._rotate_enter < self._rotate_exit:
            self.get_logger().warn(
                "rotate_enter_angle_rad < rotate_exit_angle_rad; "
                "bruk enter >= exit for meningsfull hysterese."
            )

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)

        self._goal: PoseStamped | None = None
        self._state = "IDLE"
        self._last_logged_state: str | None = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, qos)
        self._pub_cmd = self.create_publisher(Twist, self._out_topic, 10)
        self._pub_state = self.create_publisher(String, "/goal_follower/state", 10)
        self._pub_dist = self.create_publisher(Float32, "/goal_follower/distance_to_goal", 10)
        self._pub_yaw_err = self.create_publisher(Float32, "/goal_follower/heading_error", 10)
        self._pub_target_yaw = self.create_publisher(Float32, "/goal_follower/target_yaw", 10)
        self._pub_robot_yaw_raw = self.create_publisher(Float32, "/goal_follower/robot_yaw_raw", 10)
        self._pub_robot_forward_yaw = self.create_publisher(
            Float32, "/goal_follower/robot_forward_yaw", 10
        )
        self._pub_drive_allowed = self.create_publisher(Bool, "/goal_follower/drive_allowed", 10)

        self.create_timer(self._period, self._on_timer)
        self.get_logger().info(
            f"simple_goal_follower (rotate-then-drive): {self._goal_topic} -> {self._out_topic} "
            f"(TF {self._fixed_frame}->{self._base_frame}, yaw_offset={self._yaw_offset:.4f} rad; "
            f"angles enter={self._rotate_enter:.3f} exit={self._rotate_exit:.3f} drive_stop={self._drive_stop:.3f}; "
            f"v_fwd max={self._max_forward:.3f} min={self._min_forward:.3f} slow_r={self._slow_radius:.2f}; "
            f"turn_cap={min(self._max_turn, self._rotate_cap):.3f})"
        )

    def _log_state_change(self, new_state: str) -> None:
        if new_state != self._last_logged_state:
            self.get_logger().info(f"state -> {new_state}")
            self._last_logged_state = new_state

    def _on_goal(self, msg: PoseStamped) -> None:
        try:
            if msg.header.frame_id == self._fixed_frame:
                goal_map = PoseStamped()
                goal_map.header = msg.header
                goal_map.header.frame_id = self._fixed_frame
                goal_map.pose = msg.pose
            else:
                t = self._tf_buffer.lookup_transform(
                    self._fixed_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self._tf_timeout),
                )
                goal_map = do_transform_pose_stamped(msg, t)
                goal_map.header.frame_id = self._fixed_frame
        except TransformException as ex:
            self.get_logger().warn(
                f"Goal transform failed ({msg.header.frame_id}->{self._fixed_frame}): {ex}"
            )
            return

        gx = goal_map.pose.position.x
        gy = goal_map.pose.position.y
        log_extra = ""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._fixed_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            q = tf.transform.rotation
            yaw_raw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            yaw_fwd = _normalize_angle(yaw_raw + self._yaw_offset)
            tw = math.atan2(gy - ry, gx - rx)
            he = _normalize_angle(tw - yaw_fwd)
            dist = math.hypot(gx - rx, gy - ry)
            log_extra = (
                f" robot=({rx:.3f},{ry:.3f}) target_yaw={tw:.3f} yaw_raw={yaw_raw:.3f} "
                f"yaw_fwd={yaw_fwd:.3f} heading_err={he:.3f} dist={dist:.3f}"
            )
        except TransformException:
            log_extra = " (TF ikke klar for detaljlogg)"

        self.get_logger().info(f"New goal map=({gx:.3f},{gy:.3f}){log_extra}")

        self._goal = goal_map
        self._state = "IDLE"

    def _publish_debug(
        self,
        distance: float,
        heading_error: float,
        target_yaw: float,
        robot_yaw_raw: float,
        robot_forward_yaw: float,
        state: str,
        drive_allowed: bool,
    ) -> None:
        self._pub_dist.publish(Float32(data=float(distance)))
        self._pub_yaw_err.publish(Float32(data=float(heading_error)))
        self._pub_target_yaw.publish(Float32(data=float(target_yaw)))
        self._pub_robot_yaw_raw.publish(Float32(data=float(robot_yaw_raw)))
        self._pub_robot_forward_yaw.publish(Float32(data=float(robot_forward_yaw)))
        self._pub_state.publish(String(data=state))
        self._pub_drive_allowed.publish(Bool(data=drive_allowed))

    def _angular_cmd(self, heading_error: float, rotate_mode: bool) -> float:
        if abs(heading_error) < self._yaw_deadband:
            return 0.0
        lim = min(self._max_turn, self._rotate_cap) if rotate_mode else self._max_turn
        return _clamp(self._yaw_kp * heading_error, -lim, lim)

    def _drive_linear_speed(self, distance: float) -> float:
        if self._slow_radius > 1e-6:
            scale = min(1.0, distance / self._slow_radius)
        else:
            scale = 1.0
        speed = self._max_forward * scale
        return _clamp(speed, self._min_forward, self._max_forward)

    def _finalize_and_publish(
        self,
        twist: Twist,
        distance: float,
        heading_error: float,
        target_yaw: float,
        robot_yaw_raw: float,
        robot_forward_yaw: float,
        state: str,
    ) -> None:
        twist.linear.x = float(twist.linear.x * self._linear_sign)
        twist.angular.z = float(twist.angular.z * self._angular_sign)
        if twist.linear.x < 0.0:
            twist.linear.x = 0.0
        drive_allowed = state == "DRIVE_TO_GOAL"
        self._pub_cmd.publish(twist)
        self._publish_debug(
            float(distance),
            float(heading_error),
            float(target_yaw),
            float(robot_yaw_raw),
            float(robot_forward_yaw),
            state,
            drive_allowed,
        )
        self._log_state_change(state)

    def _on_timer(self) -> None:
        twist = Twist()
        if self._goal is None:
            self._state = "IDLE"
            self._pub_cmd.publish(twist)
            self._publish_debug(0.0, 0.0, 0.0, 0.0, 0.0, self._state, False)
            self._log_state_change(self._state)
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                self._fixed_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF {self._fixed_frame}->{self._base_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            self._state = "WAITING_FOR_TF"
            self._pub_cmd.publish(twist)
            self._publish_debug(0.0, 0.0, 0.0, 0.0, 0.0, self._state, False)
            self._log_state_change(self._state)
            return

        if self._state == "WAITING_FOR_TF":
            self._state = "IDLE"

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        robot_yaw_raw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        robot_forward_yaw = _normalize_angle(robot_yaw_raw + self._yaw_offset)

        gx = self._goal.pose.position.x
        gy = self._goal.pose.position.y

        dx = gx - rx
        dy = gy - ry
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        heading_error = _normalize_angle(target_yaw - robot_forward_yaw)
        abs_he = abs(heading_error)

        if distance < self._goal_tol:
            self._goal = None
            self._state = "REACHED"
            self._pub_cmd.publish(twist)
            self._publish_debug(
                float(distance),
                float(heading_error),
                float(target_yaw),
                float(robot_yaw_raw),
                float(robot_forward_yaw),
                self._state,
                False,
            )
            self._log_state_change(self._state)
            return

        if self._state == "IDLE":
            if abs_he > self._rotate_exit:
                self._state = "ROTATE_TO_GOAL"
            else:
                self._state = "DRIVE_TO_GOAL"

        if self._state == "ROTATE_TO_GOAL":
            if abs_he < self._rotate_exit:
                self._state = "DRIVE_TO_GOAL"
            else:
                twist.linear.x = 0.0
                twist.angular.z = self._angular_cmd(heading_error, True)
                self._finalize_and_publish(
                    twist,
                    distance,
                    heading_error,
                    target_yaw,
                    robot_yaw_raw,
                    robot_forward_yaw,
                    "ROTATE_TO_GOAL",
                )
                return

        # DRIVE_TO_GOAL (evt. nettopp gått ut av ROTATE i samme tikk)
        if abs_he > self._drive_stop:
            self._state = "ROTATE_TO_GOAL"
            twist.linear.x = 0.0
            twist.angular.z = self._angular_cmd(heading_error, True)
            self._finalize_and_publish(
                twist,
                distance,
                heading_error,
                target_yaw,
                robot_yaw_raw,
                robot_forward_yaw,
                self._state,
            )
            return

        twist.linear.x = float(self._drive_linear_speed(distance))
        twist.angular.z = self._angular_cmd(heading_error, False)
        self._state = "DRIVE_TO_GOAL"
        self._finalize_and_publish(
            twist,
            distance,
            heading_error,
            target_yaw,
            robot_yaw_raw,
            robot_forward_yaw,
            self._state,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleGoalFollowerNode()
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
