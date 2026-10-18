"""Runtime checks for the sensor and TF inputs required by slam_toolbox."""

from __future__ import annotations

import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class SlamDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__('slam_diagnostics')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/model/my_orange_car/odometry')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('startup_timeout_sec', 10.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        self._started_at = time.monotonic()
        self._reported = False
        self._received_scan = False
        self._received_odom = False
        self._received_map = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            10,
        )
        self._timer = self.create_timer(
            max(0.5, min(timeout / 2.0, 2.0)),
            self._check,
        )

    def _on_scan(self, _message: LaserScan) -> None:
        self._received_scan = True

    def _on_odom(self, _message: Odometry) -> None:
        self._received_odom = True

    def _on_map(self, _message: OccupancyGrid) -> None:
        self._received_map = True

    def _check(self) -> None:
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        if time.monotonic() - self._started_at < timeout:
            return

        problems = []
        if not self._received_scan:
            problems.append('没有收到激光雷达话题')
        if not self._received_odom:
            problems.append('没有收到里程计话题')
        if not self._received_map:
            problems.append('没有收到 SLAM 地图话题 /map')
        for target, source, label in (
            (self._odom_frame, self._base_frame, 'odom 到 base_footprint'),
            (self._map_frame, self._odom_frame, 'map 到 odom'),
        ):
            try:
                self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            except TransformException:
                problems.append(f'缺少 TF：{label}')

        if problems and not self._reported:
            self.get_logger().error('SLAM 输入检查失败：' + '；'.join(problems))
            self._reported = True
        elif not problems and not self._reported:
            self.get_logger().info('SLAM 输入检查通过：scan、odom、map 和 TF 均可用')
            self._reported = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
