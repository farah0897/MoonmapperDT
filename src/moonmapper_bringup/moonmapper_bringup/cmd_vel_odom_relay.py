"""In-process relay for cmd_vel + odom (rover-konvensjon i sim).

Matcher den fysiske rover-en ``dynamixel_driver.py`` slik:

    /cmd_vel (Twist)               -> /diff_drive_controller/cmd_vel
                                       (publiseres som TwistStamped fordi
                                       Jazzy ros2_controllers v4.x sin
                                       diff_drive_controller utelukkende
                                       abonnerer paa TwistStamped)
    /diff_drive_controller/odom    -> /odom  (valgfritt; se publish_odom_relay)

Alternativ (TwistStamped inn, f.eks. smoothed kommando):
    /cmd_vel_smoothed (TwistStamped) -> /diff_drive_controller/cmd_vel (TwistStamped)
    og valgfritt speil av Twist -> /cmd_vel (for diagnostikk/teleop-konvensjon).

``use_smoothed_twist_stamped_input`` settes fra launch naar kun TwistStamped
skal mates til diff_drive (uten Twist paa /cmd_vel).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


class CmdVelOdomRelay(Node):
    """Liten Twist/Odometry-relay som binder rover-topics til Gazebo-controller."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_odom_relay")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("use_smoothed_twist_stamped_input", False)
        self.declare_parameter("twist_stamped_topic", "/cmd_vel_smoothed")
        self.declare_parameter("publish_twist_cmd_vel_mirror", True)
        # False nar robot_localization EKF bruker /diff_drive_controller/odom og
        # publiserer /odometry/filtered (unnga dobbel /odom-kilde).
        self.declare_parameter("publish_odom_relay", True)

        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._use_smoothed = (
            self.get_parameter("use_smoothed_twist_stamped_input")
            .get_parameter_value()
            .bool_value
        )
        self._stamped_topic = (
            self.get_parameter("twist_stamped_topic").get_parameter_value().string_value
        )
        self._mirror_twist = (
            self.get_parameter("publish_twist_cmd_vel_mirror")
            .get_parameter_value()
            .bool_value
        )
        self._publish_odom_relay = (
            self.get_parameter("publish_odom_relay")
            .get_parameter_value()
            .bool_value
        )

        self._pub_cmd = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10,
        )
        self._pub_cmd_vel_mirror = None
        if self._use_smoothed and self._mirror_twist:
            self._pub_cmd_vel_mirror = self.create_publisher(Twist, "/cmd_vel", 10)

        if self._use_smoothed:
            self.create_subscription(
                TwistStamped,
                self._stamped_topic,
                self._on_cmd_vel_smoothed,
                10,
            )
            self.get_logger().info(
                "cmd_vel_odom_relay klar: %s (TwistStamped) -> "
                "/diff_drive_controller/cmd_vel (TwistStamped, frame_id=%s)%s."
                % (
                    self._stamped_topic,
                    self._frame_id,
                    "; speiler Twist -> /cmd_vel" if self._pub_cmd_vel_mirror else "",
                ),
            )
        else:
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().info(
                "cmd_vel_odom_relay klar: /cmd_vel (Twist) -> "
                "/diff_drive_controller/cmd_vel (TwistStamped, frame_id=%s); "
                "/diff_drive_controller/odom -> /odom." % self._frame_id,
            )

        if self._publish_odom_relay:
            self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.create_subscription(
                Odometry, "/diff_drive_controller/odom", self._on_odom, 10,
            )
        else:
            self._pub_odom = None
            self.get_logger().info(
                "cmd_vel_odom_relay: publish_odom_relay=false — ingen /odom-speil "
                "(bruk f.eks. /odometry/filtered fra EKF).",
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = msg
        self._pub_cmd.publish(stamped)

    def _on_cmd_vel_smoothed(self, msg: TwistStamped) -> None:
        out = TwistStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        out.twist = msg.twist
        self._pub_cmd.publish(out)
        if self._pub_cmd_vel_mirror is not None:
            self._pub_cmd_vel_mirror.publish(msg.twist)

    def _on_odom(self, msg: Odometry) -> None:
        if self._pub_odom is not None:
            self._pub_odom.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
