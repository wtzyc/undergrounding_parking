"""Publish odometry messages as the odom -> base_footprint TF."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/model/my_orange_car/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._broadcaster = TransformBroadcaster(self)
        topic = str(self.get_parameter('odom_topic').value)
        self._subscription = self.create_subscription(
            Odometry,
            topic,
            self._on_odometry,
            20,
        )
        self.get_logger().info(
            f'Publishing {self._odom_frame} -> {self._base_frame} from {topic}'
        )

    def _on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if not math.isfinite(norm) or norm < 1e-9:
            self.get_logger().error('收到无效里程计姿态：四元数长度为 0')
            return

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = orientation
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
