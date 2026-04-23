"""
MoonMapper rover driver plugin for webots_ros2_driver.
Subscribes to /cmd_vel, drives 6 wheels (skid-steer), publishes /odom and TF.
"""
import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import tf2_ros

# Skid-steer kinematics (simplified rocker-bogie / 6-wheel differential)
WHEEL_RADIUS = 0.08
HALF_TRACK = 0.175  # half of wheel separation (m)


class MoonmapperDriver:
    """Webots plugin: cmd_vel -> wheel motors, odom + TF publisher."""

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._timestep = int(self._robot.getBasicTimeStep())

        # Get 6 wheel motors (left: front, center, rear | right: front, center, rear)
        self._left_motors = [
            self._robot.getDevice("left_front_motor"),
            self._robot.getDevice("left_center_motor"),
            self._robot.getDevice("left_rear_motor"),
        ]
        self._right_motors = [
            self._robot.getDevice("right_front_motor"),
            self._robot.getDevice("right_center_motor"),
            self._robot.getDevice("right_rear_motor"),
        ]

        for m in self._left_motors + self._right_motors:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self._target_twist = Twist()

        # ROS 2 node (reuse existing rclpy context if any)
        try:
            rclpy.init(args=None)
        except Exception:
            pass
        self._node = Node("moonmapper_driver")

        self._node.create_subscription(
            Twist, "cmd_vel", self._cmd_vel_callback, 1
        )

        self._odom_pub = self._node.create_publisher(
            Odometry, "odom", 10
        )
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self._node)

        # Odometry state (integrated from cmd_vel for sim; replace with wheel encoders on real hardware)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self._robot.getTime()

    def _cmd_vel_callback(self, msg):
        self._target_twist = msg

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

        t = self._robot.getTime()
        dt = t - self._last_time
        self._last_time = t

        forward = self._target_twist.linear.x
        angular = self._target_twist.angular.z

        # Skid-steer: left and right wheel velocities
        left_vel = (forward - angular * HALF_TRACK) / WHEEL_RADIUS
        right_vel = (forward + angular * HALF_TRACK) / WHEEL_RADIUS

        for m in self._left_motors:
            m.setVelocity(left_vel)
        for m in self._right_motors:
            m.setVelocity(right_vel)

        # Integrate odometry (simplified; on real HW use wheel encoders)
        self._x += (forward * math.cos(self._theta) - 0) * dt
        self._y += (forward * math.sin(self._theta) + 0) * dt
        self._theta += angular * dt

        # Publish odom and TF
        odom = Odometry()
        odom.header.stamp = self._node.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        qx = 0.0
        qy = 0.0
        qz = math.sin(self._theta / 2)
        qw = math.cos(self._theta / 2)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = forward
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular
        self._odom_pub.publish(odom)

        t_msg = TransformStamped()
        t_msg.header.stamp = odom.header.stamp
        t_msg.header.frame_id = "odom"
        t_msg.child_frame_id = "base_link"
        t_msg.transform.translation.x = self._x
        t_msg.transform.translation.y = self._y
        t_msg.transform.translation.z = 0.0
        t_msg.transform.rotation.x = qx
        t_msg.transform.rotation.y = qy
        t_msg.transform.rotation.z = qz
        t_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t_msg)
