#!/usr/bin/env python3

"""ROS2 localization node for the iPlow Robot assignment."""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from localization_math import geodetic_to_enu, quaternion_to_yaw


class LocalizationNode(Node):
    """Estimate robot pose from GPS position and heading."""

    def __init__(self) -> None:
        super().__init__("localization_node")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.reference_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.current_position_enu: Optional[
            Tuple[float, float, float]
        ] = None

        self.current_yaw: Optional[float] = None

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/robot_pose",
            10,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.publish_timer = self.create_timer(
            0.1,
            self.publish_localization,
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            self.gps_callback,
            sensor_qos,
        )

        self.heading_subscription = self.create_subscription(
            Imu,
            "/navheading",
            self.heading_callback,
            sensor_qos,
        )

        self.get_logger().info("Localization node initialized.")

    def gps_callback(self, msg: NavSatFix) -> None:
        """Update local ENU position from GPS."""
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warning(
                "Ignoring GPS message without a valid fix."
            )
            return

        if not all(
            math.isfinite(value)
            for value in (
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
        ):
            self.get_logger().warning(
                "Ignoring GPS fix with non-finite coordinates."
            )
            return
        if self.reference_position is None:
            self.reference_position = (
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )

            self.get_logger().info(
                "GPS reference initialized at "
                f"lat={msg.latitude:.8f}, "
                f"lon={msg.longitude:.8f}, "
                f"alt={msg.altitude:.3f}"
            )

        ref_lat, ref_lon, ref_alt = self.reference_position

        self.current_position_enu = geodetic_to_enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            ref_lat,
            ref_lon,
            ref_alt,
        )

    def heading_callback(self, msg: Imu) -> None:
        """Update robot yaw from heading quaternion."""
        orientation = msg.orientation

        self.current_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def publish_localization(self) -> None:
        """Publish the latest localization estimate."""
        if (
            self.current_position_enu is None
            or self.current_yaw is None
        ):
            return

        east, north, up = self.current_position_enu
        yaw = self.current_yaw

        half_yaw = yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "odom"

        pose_msg.pose.position.x = east
        pose_msg.pose.position.y = north
        pose_msg.pose.position.z = up

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_publisher.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base"

        odom_msg.pose.pose = pose_msg.pose

        self.odom_publisher.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base"

        transform.transform.translation.x = east
        transform.transform.translation.y = north
        transform.transform.translation.z = up

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """Run the localization node."""
    rclpy.init(args=args)

    node = LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()