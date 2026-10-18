import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage


class DataCollector(Node):
    def __init__(self) -> None:
        super().__init__('data_collector')
        self.declare_parameter('camera_topic', '/camera_front/image')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/model/my_orange_car/odometry')
        self.declare_parameter('truth_pose_topic', '/world/luxury_parking_lot_expanded/pose/info')
        self.declare_parameter('output_dir', str(Path.home() / 'ros2_ws' / 'dataset' / 'raw'))
        self.declare_parameter('vehicle_model_name', 'my_orange_car')
        self.declare_parameter('save_interval_sec', 0.5)

        self._output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self._image_dir = self._output_dir / 'images'
        self._metadata_dir = self._output_dir / 'metadata'
        self._image_dir.mkdir(parents=True, exist_ok=True)
        self._metadata_dir.mkdir(parents=True, exist_ok=True)

        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._latest_image: Image | None = None
        self._truth_pose: dict | None = None
        self._odom_pose: dict | None = None
        self._frame_index = self._next_frame_index()
        self._last_save_time = 0.0

        self._image_sub = self.create_subscription(Image, self.get_parameter('camera_topic').value, self._on_image, 10)
        self._scan_sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self._odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self._truth_sub = self.create_subscription(
            TFMessage,
            self.get_parameter('truth_pose_topic').value,
            self._on_truth_pose,
            10,
        )

        self.get_logger().info(f'data_collector saving images only to {self._output_dir}')

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._odom_pose = {
            'x': float(position.x),
            'y': float(position.y),
            'z': float(position.z),
            'yaw': self._yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
            'stamp': self._stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
        }

    def _on_truth_pose(self, msg: TFMessage) -> None:
        vehicle_name = str(self.get_parameter('vehicle_model_name').value)
        best = None
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child == vehicle_name or child.endswith('/' + vehicle_name):
                best = transform
                break
            if vehicle_name in child:
                best = transform
        if best is None:
            return

        translation = best.transform.translation
        rotation = best.transform.rotation
        self._truth_pose = {
            'x': float(translation.x),
            'y': float(translation.y),
            'z': float(translation.z),
            'yaw': self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            'stamp': self._stamp_to_float(best.header.stamp),
            'frame_id': best.header.frame_id,
            'child_frame_id': best.child_frame_id,
        }

    def _on_image(self, msg: Image) -> None:
        self._latest_image = msg
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_save_time < float(self.get_parameter('save_interval_sec').value):
            return
        self._last_save_time = now
        self._save_sample(msg)

    def _save_sample(self, msg: Image) -> None:
        try:
            frame = self._image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'failed to decode image: {exc}')
            return

        stem = f'frame_{self._frame_index:06d}'
        image_path = self._image_dir / f'{stem}.jpg'
        metadata_path = self._metadata_dir / f'{stem}.json'
        cv2.imwrite(str(image_path), frame)
        metadata = {
            'image': str(image_path),
            'camera_topic': self.get_parameter('camera_topic').value,
            'image_stamp': self._stamp_to_float(msg.header.stamp),
            'scan_stamp': self._stamp_to_float(self._latest_scan.header.stamp) if self._latest_scan is not None else None,
            'odom_stamp': self._stamp_to_float(self._latest_odom.header.stamp) if self._latest_odom is not None else None,
            'truth_pose': self._truth_pose,
            'odom_pose': self._odom_pose,
            'scan_summary': self._scan_summary(self._latest_scan),
            'odom': self._odom_summary(self._latest_odom),
        }
        with metadata_path.open('w', encoding='utf-8') as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)
        self._frame_index += 1

    def _next_frame_index(self) -> int:
        existing = sorted(self._image_dir.glob('frame_*.jpg'))
        if not existing:
            return 0
        try:
            return int(existing[-1].stem.split('_')[-1]) + 1
        except ValueError:
            return len(existing)

    @staticmethod
    def _sector_min(scan: LaserScan, min_deg: float, max_deg: float) -> float:
        values = []
        for index, value in enumerate(scan.ranges):
            angle = math.degrees(scan.angle_min + index * scan.angle_increment)
            if angle < min_deg or angle > max_deg:
                continue
            distance = float(value)
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                values.append(distance)
        if not values:
            return float(scan.range_max)
        return min(values)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        array = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ('rgb8', 'bgr8'):
            frame = array.reshape((msg.height, msg.width, 3))
            if encoding == 'rgb8':
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        if encoding in ('rgba8', 'bgra8'):
            frame = array.reshape((msg.height, msg.width, 4))
            if encoding == 'rgba8':
                return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        raise ValueError(f'unsupported encoding: {msg.encoding}')

    @staticmethod
    def _scan_summary(scan: LaserScan | None) -> dict | None:
        if scan is None:
            return None
        ranges = [float(value) for value in scan.ranges if math.isfinite(float(value))]
        return {
            'frame_id': scan.header.frame_id,
            'range_min': float(scan.range_min),
            'range_max': float(scan.range_max),
            'valid_count': len(ranges),
            'front_min_m': DataCollector._sector_min(scan, -25.0, 25.0),
            'left_min_m': DataCollector._sector_min(scan, 25.0, 95.0),
            'right_min_m': DataCollector._sector_min(scan, -95.0, -25.0),
        }

    @staticmethod
    def _odom_summary(odom: Odometry | None) -> dict | None:
        if odom is None:
            return None
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        return {
            'frame_id': odom.header.frame_id,
            'child_frame_id': odom.child_frame_id,
            'position': {
                'x': float(position.x),
                'y': float(position.y),
                'z': float(position.z),
            },
            'orientation': {
                'x': float(orientation.x),
                'y': float(orientation.y),
                'z': float(orientation.z),
                'w': float(orientation.w),
            },
        }

    @staticmethod
    def _stamp_to_float(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main() -> None:
    rclpy.init()
    node = DataCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
