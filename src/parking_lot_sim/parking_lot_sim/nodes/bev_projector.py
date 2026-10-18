"""ROS 2 node that publishes a god-camera ground-plane BEV image."""

from __future__ import annotations

import json
from typing import Any

import cv2
import numpy as np
from parking_lot_sim.bev import BEVConfig, BEVError, BEVProjector, CameraExtrinsics
from parking_lot_sim.bev import CameraIntrinsics
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class BevProjectorNode(Node):
    def __init__(self) -> None:
        super().__init__('bev_projector')
        self.declare_parameter('camera_topic', '/camera_god/image')
        self.declare_parameter('camera_info_topic', '/camera_god/camera_info')
        self.declare_parameter('bev_topic', '/camera_god/bev_image')
        self.declare_parameter('debug_topic', '/camera_god/bev_debug_image')
        self.declare_parameter('status_topic', '/camera_god/bev_status')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('x_min', -8.0)
        self.declare_parameter('x_max', 8.0)
        self.declare_parameter('y_min', -8.0)
        self.declare_parameter('y_max', 8.0)
        self.declare_parameter('resolution_m_per_pixel', 0.02)
        self.declare_parameter('input_timeout_sec', 1.0)

        self._camera_topic = str(self.get_parameter('camera_topic').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._projector: BEVProjector | None = None
        self._camera_info_frame = ''
        self._last_image_stamp: float | None = None

        self._bev_publisher = self.create_publisher(
            Image,
            self.get_parameter('bev_topic').value,
            1,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            self.get_parameter('debug_topic').value,
            1,
        )
        self._status_publisher = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self._camera_info_subscriber = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_camera_info,
            image_qos,
        )
        self._image_subscriber = self.create_subscription(
            Image,
            self._camera_topic,
            self._on_image,
            image_qos,
        )
        timeout = max(0.1, float(self.get_parameter('input_timeout_sec').value))
        self._input_timer = self.create_timer(timeout / 2.0, self._check_input)
        self.get_logger().info(
            f'bev_projector ready on {self._camera_topic}; output frame {self._base_frame}'
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        try:
            intrinsics = CameraIntrinsics.from_camera_info(message)
            config = self._bev_config()
            if (
                self._projector is None
                or self._projector.intrinsics != intrinsics
                or self._projector.config != config
            ):
                self._projector = BEVProjector(
                    intrinsics,
                    CameraExtrinsics.from_base_rpy(
                        x=0.0,
                        y=0.0,
                        z=15.0,
                        pitch=1.5707963267948966,
                        frame_id='camera_god',
                    ),
                    config,
                )
            self._camera_info_frame = message.header.frame_id
            self._publish_status(
                'ready',
                stamp=self._stamp_to_float(message.header.stamp),
                source_frame=self._camera_info_frame,
            )
        except (BEVError, TypeError, ValueError) as exc:
            self._projector = None
            self._publish_status(
                'invalid_camera_info',
                stamp=self._stamp_to_float(message.header.stamp),
                source_frame=message.header.frame_id,
                reason=str(exc),
            )
            self.get_logger().error(f'invalid camera info: {exc}')

    def _on_image(self, message: Image) -> None:
        stamp = self._stamp_to_float(message.header.stamp)
        self._last_image_stamp = stamp
        if self._projector is None:
            self._publish_status(
                'waiting_for_camera_info',
                stamp=stamp,
                source_frame=message.header.frame_id,
                reason='no valid CameraInfo has been received',
            )
            return
        try:
            frame = self._image_to_bgr(message)
            if (
                frame.shape[1] != self._projector.intrinsics.width
                or frame.shape[0] != self._projector.intrinsics.height
            ):
                raise BEVError('image dimensions do not match CameraInfo')
            warped = self._projector.warp(
                frame,
                stamp=stamp,
                source_frame=message.header.frame_id or self._camera_info_frame,
            )
            self._bev_publisher.publish(
                self._numpy_to_image(
                    warped.image,
                    message,
                    self._base_frame,
                )
            )
            if self._debug_publisher.get_subscription_count() > 0:
                debug = self._projector.debug_overlay(frame)
                self._debug_publisher.publish(
                    self._numpy_to_image(
                        debug,
                        message,
                        message.header.frame_id or self._camera_info_frame,
                    )
                )
            self._publish_json(warped.metadata)
        except (BEVError, ValueError, cv2.error) as exc:
            self._publish_status(
                'processing_error',
                stamp=stamp,
                source_frame=message.header.frame_id,
                reason=str(exc),
            )
            self.get_logger().error(f'BEV processing failed: {exc}')

    def _check_input(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = max(0.1, float(self.get_parameter('input_timeout_sec').value))
        if self._last_image_stamp is None:
            self._publish_status(
                'waiting_for_image',
                stamp=now,
                source_frame=self._camera_info_frame,
                reason='no god-camera image has been received',
            )
            return
        if now - self._last_image_stamp > timeout:
            self._publish_status(
                'camera_input_stale',
                stamp=now,
                source_frame=self._camera_info_frame,
                reason=f'no image received for {now - self._last_image_stamp:.3f} seconds',
            )

    def _bev_config(self) -> BEVConfig:
        return BEVConfig(
            x_min=float(self.get_parameter('x_min').value),
            x_max=float(self.get_parameter('x_max').value),
            y_min=float(self.get_parameter('y_min').value),
            y_max=float(self.get_parameter('y_max').value),
            resolution_m_per_pixel=float(
                self.get_parameter('resolution_m_per_pixel').value
            ),
            frame_id=self._base_frame,
            ground_z=float(self.get_parameter('ground_z').value),
        )

    def _publish_status(
        self,
        status: str,
        *,
        stamp: float | None,
        source_frame: str,
        reason: str = '',
    ) -> None:
        if self._projector is not None:
            payload = self._projector.status(
                status,
                stamp=stamp,
                source_frame=source_frame,
                reason=reason,
            )
        else:
            payload = {
                'status': status,
                'reason': reason,
                'stamp': stamp,
                'source_frame': source_frame,
                'frame_id': self._base_frame,
            }
        self._publish_json(payload)

    def _publish_json(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self._status_publisher.publish(message)

    @staticmethod
    def _stamp_to_float(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    @staticmethod
    def _image_to_bgr(message: Image) -> np.ndarray:
        encoding = message.encoding.lower()
        if encoding in ('mono8',):
            channels = 1
        elif encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif encoding in ('rgba8', 'bgra8'):
            channels = 4
        else:
            raise ValueError(f'unsupported image encoding: {message.encoding}')
        row_bytes = int(message.width) * channels
        step = int(message.step) if message.step else row_bytes
        data = np.frombuffer(message.data, dtype=np.uint8)
        if data.size < int(message.height) * step or step < row_bytes:
            raise ValueError('image data is shorter than the declared dimensions')
        rows = data[:int(message.height) * step].reshape((int(message.height), step))
        frame = rows[:, :row_bytes].reshape((int(message.height), int(message.width), channels))
        if encoding == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    @staticmethod
    def _numpy_to_image(
        frame: np.ndarray,
        source: Image,
        frame_id: str,
    ) -> Image:
        if frame.ndim == 2:
            encoding = 'mono8'
            channels = 1
        elif frame.ndim == 3 and frame.shape[2] == 3:
            encoding = 'bgr8'
            channels = 3
        else:
            raise ValueError('BEV output must be mono or BGR')
        message = Image()
        message.header.stamp = source.header.stamp
        message.header.frame_id = frame_id
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = int(frame.shape[1] * channels)
        message.data = np.ascontiguousarray(frame).tobytes()
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BevProjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
