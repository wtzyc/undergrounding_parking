#!/usr/bin/env python3
"""Save individual BEV frames to the offline VLM input directory."""

import argparse
from pathlib import Path
import re

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class BevImageSaver(Node):
    def __init__(self, topic: str, output_dir: Path) -> None:
        super().__init__('bev_image_saver')
        self.topic = topic
        self.output_dir = output_dir.expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_image: Image | None = None
        self.frame_sequence = 0
        self.last_saved_sequence = 0

        # The running BEV publisher was observed to deliver frames to a reliable
        # subscriber, while the sensor-data best-effort subscription got none.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image, self.topic, self._on_image, image_qos
        )
        self.service = self.create_service(
            Trigger, '~/save', self._on_save
        )
        self.get_logger().info(
            f'Listening to {self.topic}; save with '
            f"ros2 service call /bev_image_saver/save std_srvs/srv/Trigger '{{}}'"
        )
        self.get_logger().info(f'Images will be written to {self.output_dir}')

    def _on_image(self, message: Image) -> None:
        self.latest_image = message
        self.frame_sequence += 1

    def _next_path(self) -> Path:
        highest = 0
        pattern = re.compile(r'^bev_capture_(\d+)\.png$')
        for path in self.output_dir.iterdir():
            match = pattern.match(path.name)
            if match:
                highest = max(highest, int(match.group(1)))
        return self.output_dir / f'bev_capture_{highest + 1:03d}.png'

    def _on_save(self, _request: Trigger.Request, response: Trigger.Response):
        if self.latest_image is None:
            response.success = False
            response.message = f'No image received yet on {self.topic}.'
            return response
        if self.frame_sequence == self.last_saved_sequence:
            response.success = False
            response.message = 'No new BEV frame since the previous save; wait briefly and retry.'
            return response

        try:
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except CvBridgeError as exc:
            response.success = False
            response.message = f'Could not convert BEV image: {exc}'
            return response

        path = self._next_path()
        if not cv2.imwrite(str(path), image):
            response.success = False
            response.message = f'Failed to write image: {path}'
            return response

        self.last_saved_sequence = self.frame_sequence
        response.success = True
        response.message = f'Saved {path} ({image.shape[1]}x{image.shape[0]}).'
        self.get_logger().info(response.message)
        return response


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera_god/bev_image')
    parser.add_argument(
        '--output-dir',
        default=str(Path(__file__).resolve().parent / 'images'),
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = BevImageSaver(args.topic, Path(args.output_dir))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
