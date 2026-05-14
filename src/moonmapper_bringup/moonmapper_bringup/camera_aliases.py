"""Camera topic + frame_id aliaser for MoonMapper-simulasjonen.

Kilder matcher ``ros_gz_bridge.yaml`` (/depth_camera/image, depth_image, ...).

Speiler den fysiske rover-ens RealSense D435-konvensjon (se
``rover/simulation/Simulation Guide.md`` og ``rover/setup/startup.md``) med
``use_sim_time:=true``.

Mapping (sim -> rover):

    /depth_camera/image        (depth_camera_optical_frame)
        -> /camera/camera/color/image_raw         (camera_color_optical_frame)
    /depth_camera/depth_image  (depth_camera_optical_frame)
        -> /camera/camera/depth/image_rect_raw    (camera_depth_optical_frame)
    /depth_camera/camera_info  (depth_camera_optical_frame)
        -> /camera/camera/color/camera_info       (camera_color_optical_frame)
    /depth_camera/points       (depth_camera_optical_frame)
        -> /camera/camera/depth/color/points      (camera_depth_optical_frame)

Statisk TF for camera_link og _optical_frame-aliasene legges i
``moonmapper_description`` (gazebo_rover) / bringup.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class CameraAliases(Node):
    """Republiserer Gazebo-depth-kamera-topics med rover-konvensjon."""

    def __init__(self) -> None:
        super().__init__("camera_aliases")

        self.declare_parameter("color_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_frame", "camera_depth_optical_frame")
        # Gazebo/ros_gz_bridge kan gi header.stamp som ikke matcher ROS /clock (sim).
        # Da feiler collision_monitor (kilde vs node-tid) og diagnostikk viser enorm «alder».
        # True: sett utgående header.stamp = denne nodens ROS-klokke (krever use_sim_time + /clock i sim).
        self.declare_parameter("refresh_header_stamp", True)
        self.color_frame = (
            self.get_parameter("color_frame").get_parameter_value().string_value
        )
        self.depth_frame = (
            self.get_parameter("depth_frame").get_parameter_value().string_value
        )
        self._refresh_stamp = (
            self.get_parameter("refresh_header_stamp").get_parameter_value().bool_value
        )

        self._pub_color = self.create_publisher(
            Image, "/camera/camera/color/image_raw", SENSOR_QOS,
        )
        self._pub_depth = self.create_publisher(
            Image, "/camera/camera/depth/image_rect_raw", SENSOR_QOS,
        )
        self._pub_info = self.create_publisher(
            CameraInfo, "/camera/camera/color/camera_info", SENSOR_QOS,
        )
        self._pub_points = self.create_publisher(
            PointCloud2, "/camera/camera/depth/color/points", SENSOR_QOS,
        )

        self.create_subscription(
            Image, "/depth_camera/image", self._on_color, SENSOR_QOS,
        )
        self.create_subscription(
            Image, "/depth_camera/depth_image", self._on_depth, SENSOR_QOS,
        )
        self.create_subscription(
            CameraInfo, "/depth_camera/camera_info", self._on_info, SENSOR_QOS,
        )
        self.create_subscription(
            PointCloud2, "/depth_camera/points", self._on_points, SENSOR_QOS,
        )

        self.get_logger().info(
            "camera_aliases klar: republiserer /depth_camera/* som "
            "/camera/camera/* med rover-frame-id ('%s' / '%s'); refresh_header_stamp=%s."
            % (self.color_frame, self.depth_frame, self._refresh_stamp),
        )

    def _stamp_now(self, msg: Image | CameraInfo | PointCloud2) -> None:
        if self._refresh_stamp:
            msg.header.stamp = self.get_clock().now().to_msg()

    def _on_color(self, msg: Image) -> None:
        self._stamp_now(msg)
        msg.header.frame_id = self.color_frame
        self._pub_color.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        self._stamp_now(msg)
        msg.header.frame_id = self.depth_frame
        self._pub_depth.publish(msg)

    def _on_info(self, msg: CameraInfo) -> None:
        self._stamp_now(msg)
        msg.header.frame_id = self.color_frame
        self._pub_info.publish(msg)

    def _on_points(self, msg: PointCloud2) -> None:
        self._stamp_now(msg)
        msg.header.frame_id = self.depth_frame
        self._pub_points.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraAliases()
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
