#!/usr/bin/env python3
"""unity_cmd_vel_node.py

Ros2_control-erstatning for Unity-simulering.

Topologi:
    /cmd_vel (geometry_msgs/Twist)
        -> differential-drive-kinematikk
        -> /unity/wheel_velocities (std_msgs/Float64MultiArray)
             [wheel_l1, wheel_l2, wheel_l3, wheel_r1, wheel_r2, wheel_r3]
             i radianer/sekund.

Unity-siden abonnerer paa /unity/wheel_velocities og setter
ArticulationBody.jointVelocity (eller XDrive target) pr. hjul.

Hvorfor ikke bare sende Twist rett til Unity?
    * Diff-drive-matematikken holdes paa ROS-siden, saa Unity-scenen
      ikke trenger aa kjenne roverens geometri.
    * Seks separate hjul-verdier gir enklere integrering mot
      fremtidig uavhengig 6WD-kontroll uten aa endre Unity-siden.

Parametre (yaml via ros2 run / launch):
    wheel_separation:     Avstand mellom venstre og hoeyre hjulaksling (m)
    wheel_radius:         Hjulradius (m)
    max_linear_velocity:  Maks |v| paa Twist.linear.x (m/s)
    max_angular_velocity: Maks |w| paa Twist.angular.z (rad/s)
    publish_rate:         Hvor ofte vi publiserer siste kommando (Hz)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class UnityCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("unity_cmd_vel_bridge")

        self.declare_parameter("wheel_separation", 0.030)
        self.declare_parameter("wheel_radius", 0.0073)
        self.declare_parameter("max_linear_velocity", 0.05)
        self.declare_parameter("max_angular_velocity", 0.2)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("use_stamped", False)

        self._wheel_sep = float(
            self.get_parameter("wheel_separation").value)
        self._wheel_radius = float(
            self.get_parameter("wheel_radius").value)
        self._v_max = float(
            self.get_parameter("max_linear_velocity").value)
        self._w_max = float(
            self.get_parameter("max_angular_velocity").value)
        self._rate = float(
            self.get_parameter("publish_rate").value)
        self._timeout = float(
            self.get_parameter("cmd_vel_timeout").value)
        self._use_stamped = bool(
            self.get_parameter("use_stamped").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # DDS tillater ikke to abonnementer paa samme topic-navn med
        # ulike typer, saa vi velger presist én type basert paa
        # `use_stamped`-parameteren. Maa matche hva teleop publiserer.
        if self._use_stamped:
            self._sub = self.create_subscription(
                TwistStamped, "/cmd_vel", self._on_cmd_vel_stamped, qos)
            self.get_logger().info("Abonnerer paa /cmd_vel (TwistStamped)")
        else:
            self._sub = self.create_subscription(
                Twist, "/cmd_vel", self._on_cmd_vel, qos)
            self.get_logger().info("Abonnerer paa /cmd_vel (Twist)")

        self._pub = self.create_publisher(
            Float64MultiArray, "/unity/wheel_velocities", qos)

        self._last_cmd_time = self.get_clock().now()
        self._v = 0.0
        self._w = 0.0

        self._timer = self.create_timer(1.0 / self._rate, self._on_timer)

        self.get_logger().info(
            f"UnityCmdVelBridge startet: "
            f"wheel_separation={self._wheel_sep} m, "
            f"wheel_radius={self._wheel_radius} m, "
            f"rate={self._rate} Hz"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._v = max(-self._v_max, min(self._v_max, msg.linear.x))
        self._w = max(-self._w_max, min(self._w_max, msg.angular.z))
        self._last_cmd_time = self.get_clock().now()

    def _on_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        # Vi ignorerer frame_id og bruker twist direkte. For korrekt
        # frame-haandtering (f.eks. hvis teleop publiserer i en roterende
        # ramme), ville man transformert via tf2. For teleop i base_link
        # er det ikke noedvendig.
        self._on_cmd_vel(msg.twist)

    def _on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self._timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self._v, self._w

        # Standard differential-drive-kinematikk:
        # v_left  = v - w * L/2
        # v_right = v + w * L/2
        # omega_wheel = v_wheel / r
        v_left = v - w * self._wheel_sep * 0.5
        v_right = v + w * self._wheel_sep * 0.5

        if self._wheel_radius < 1e-9:
            # sikkerhet: unngaa divisjon paa 0 hvis noen glemmer parameter
            return

        omega_l = v_left / self._wheel_radius
        omega_r = v_right / self._wheel_radius

        out = Float64MultiArray()
        out.layout.dim.append(MultiArrayDimension(
            label="wheel_velocities",
            size=6,
            stride=6,
        ))
        out.layout.data_offset = 0
        # Rekkefoelge: [l1, l2, l3, r1, r2, r3] (alle i rad/s)
        out.data = [omega_l, omega_l, omega_l, omega_r, omega_r, omega_r]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnityCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
