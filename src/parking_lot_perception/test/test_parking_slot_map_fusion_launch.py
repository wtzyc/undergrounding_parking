import json
import os
import socket
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import rclpy
from geometry_msgs.msg import TransformStamped
from launch_ros.actions import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def generate_test_description():
    global semantic_map_path, ros_domain_id
    temp_dir = tempfile.TemporaryDirectory(prefix='parking_slot_fusion_test_')
    semantic_map_path = Path(temp_dir.name) / 'semantic_map.json'
    ros_domain_id = str(42 + os.getpid() % 100)
    os.environ['ROS_DOMAIN_ID'] = ros_domain_id
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
        global ros_network_available
        ros_network_available = True
    except OSError:
        ros_network_available = False
    if not ros_network_available:
        return launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
        ]), {'temp_dir': temp_dir}

    fusion = Node(
        package='parking_lot_perception',
        executable='parking_slot_map_fusion',
        parameters=[{'map_path': str(semantic_map_path), 'confirm_hits': 10}],
        additional_env={'ROS_DOMAIN_ID': ros_domain_id},
        output='screen',
    )
    return launch.LaunchDescription([
        fusion,
        launch_testing.actions.ReadyToTest(),
    ]), {'temp_dir': temp_dir}


class TestFusionMapPersistenceWithoutTf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not ros_network_available:
            raise unittest.SkipTest(
                'ROS 2 DDS integration test requires local UDP sockets, unavailable in this sandbox')
        rclpy.init()
        cls.node = rclpy.create_node('parking_slot_fusion_external_test')
        cls.messages = []
        cls.publisher = cls.node.create_publisher(
            String, '/semantic/parking_slot_candidates', 10)
        cls.pillar_publisher = cls.node.create_publisher(
            String, '/semantic/pillar_candidates', 10)
        cls.subscription = cls.node.create_subscription(
            String,
            '/semantic/parking_slots_map',
            cls.messages.append,
            10,
        )
        cls.marker_messages = []
        cls.marker_subscription = cls.node.create_subscription(
            MarkerArray,
            '/semantic/parking_slot_markers',
            cls.marker_messages.append,
            10,
        )
        cls.pillar_marker_messages = []
        cls.pillar_marker_subscription = cls.node.create_subscription(
            MarkerArray,
            '/semantic/pillar_landmark_markers',
            cls.pillar_marker_messages.append,
            10,
        )
        cls.road_node_marker_messages = []
        cls.road_node_marker_subscription = cls.node.create_subscription(
            MarkerArray,
            '/semantic/virtual_road_node_markers',
            cls.road_node_marker_messages.append,
            10,
        )
        cls.save_client = cls.node.create_client(
            Trigger, '/semantic/parking_slots_map/save')
        cls._spin_until(lambda: cls.save_client.service_is_ready(), timeout=10.0)
        cls._spin_until(lambda: bool(cls.messages), timeout=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin_until(cls, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if condition():
                return
        raise AssertionError('ROS graph condition was not reached before timeout')

    def test_candidate_without_tf_is_not_added_and_save_service_persists_empty_map(self):
        self._spin_until(lambda: self.publisher.get_subscription_count() > 0, 5.0)
        for _ in range(10):
            stamp = self.node.get_clock().now().nanoseconds / 1e9
            candidate = {
                'schema_version': 'parking-slot-observations-v2',
                'candidate_only': True,
                'frame_id': 'base_footprint',
                'stamp': stamp,
                'candidates': [{
                    'id': 'candidate_1',
                    'partial_or_occluded': False,
                    'x': 2.0,
                    'y': 1.0,
                    'yaw': 0.0,
                    'length': 5.8,
                    'width': 2.8,
                    'confidence': 0.9,
                }],
            }
            message = String()
            message.data = json.dumps(candidate)
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        request = Trigger.Request()
        future = self.save_client.call_async(request)
        self._spin_until(lambda: future.done(), timeout=5.0)
        response = future.result()
        self.assertTrue(response.success, response.message)
        self.assertTrue(semantic_map_path.exists())

        saved = json.loads(semantic_map_path.read_text(encoding='utf-8'))
        self.assertEqual(saved['frame_id'], 'map')
        self.assertEqual(saved['landmarks'], [])
        self.assertEqual(saved['road_nodes'], [])
        self.assertEqual(saved['edges'], [])

    def test_fusion_publishes_unified_map_schema(self):
        initial = json.loads(self.messages[0].data)
        self.assertIn('version', initial)
        self.assertEqual(initial['frame_id'], 'map')
        self.assertIn('landmarks', initial)
        self.assertEqual(initial['road_nodes'], [])
        self.assertEqual(initial['edges'], [])

    def test_confirmed_parking_slot_is_published_as_rviz_markers(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(self.node)
        broadcaster.sendTransform(transform)

        self._spin_until(lambda: self.publisher.get_subscription_count() > 0, 5.0)
        map_message_count = len(self.messages)
        for _ in range(10):
            candidate = {
                'schema_version': 'parking-slot-observations-v2',
                'candidate_only': True,
                'frame_id': 'base_footprint',
                'stamp': self.node.get_clock().now().nanoseconds / 1e9,
                'candidates': [{
                    'id': 'visible_slot',
                    'partial_or_occluded': True,
                    'x': 2.0,
                    'y': 1.0,
                    'yaw': 0.25,
                    'length': 5.8,
                    'width': 2.8,
                    'confidence': 0.9,
                }],
            }
            message = String()
            message.data = json.dumps(candidate)
            self.publisher.publish(message)
            self._spin_until(lambda: len(self.messages) > map_message_count, 5.0)
            map_message_count = len(self.messages)

        fused = json.loads(self.messages[-1].data)
        self.assertEqual(len(fused['landmarks']), 1)
        self.assertEqual(fused['landmarks'][0]['type'], 'parking_slot')
        self.assertEqual(len(fused['road_nodes']), 1)
        self.assertEqual(fused['road_nodes'][0]['node_type'], 'parking_anchor')
        self.assertEqual(fused['road_nodes'][0]['anchor_id'], 'parking_slot_1')
        self._spin_until(
            lambda: any(
                marker.action == Marker.ADD and marker.type == Marker.TEXT_VIEW_FACING
                for array in self.marker_messages for marker in array.markers
            ),
            5.0,
        )
        latest = self.marker_messages[-1]
        added = [marker for marker in latest.markers if marker.action == Marker.ADD]
        self.assertEqual(len(added), 3)
        self.assertTrue(all(marker.header.frame_id == 'map' for marker in added))
        self.assertTrue(any(marker.type == Marker.LINE_STRIP for marker in added))
        self.assertTrue(any(marker.type == Marker.SPHERE for marker in added))
        self.assertTrue(any(marker.type == Marker.TEXT_VIEW_FACING for marker in added))
        self._spin_until(
            lambda: any(
                marker.action == Marker.ADD and marker.ns == 'virtual_road_node'
                for array in self.road_node_marker_messages for marker in array.markers
            ),
            5.0,
        )

    def test_confirmed_pillar_is_kept_in_map_and_save_file(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(self.node)
        broadcaster.sendTransform(transform)

        self._spin_until(lambda: self.pillar_publisher.get_subscription_count() > 0, 5.0)
        map_message_count = len(self.messages)
        for _ in range(10):
            candidate = {
                'schema_version': 'pillar-candidates-v1',
                'candidate_only': True,
                'frame_id': 'base_footprint',
                'source_frame': 'test_lidar',
                'stamp': self.node.get_clock().now().nanoseconds / 1e9,
                'status': 'ok',
                'reason': '',
                'candidates': [{
                    'id': 'pillar-1',
                    'x': 4.0,
                    'y': -2.0,
                    'observed_width_m': 1.1,
                    'confidence': 0.9,
                    'partial_or_occluded': False,
                }],
            }
            message = String()
            message.data = json.dumps(candidate)
            self.pillar_publisher.publish(message)
            self._spin_until(lambda: len(self.messages) > map_message_count, 5.0)
            map_message_count = len(self.messages)

        fused = json.loads(self.messages[-1].data)
        pillars = [landmark for landmark in fused['landmarks'] if landmark['type'] == 'pillar']
        self.assertEqual(len(pillars), 1)
        self.assertEqual(pillars[0]['id'], 'pillar_1')
        self.assertEqual(pillars[0]['status'], 'confirmed')
        self.assertEqual(pillars[0]['observation_count'], 10)
        self.assertAlmostEqual(pillars[0]['pose']['x'], 4.0)
        self.assertAlmostEqual(pillars[0]['pose']['y'], -2.0)

        no_detection = {
            'schema_version': 'pillar-candidates-v1',
            'candidate_only': True,
            'frame_id': 'base_footprint',
            'source_frame': 'test_lidar',
            'stamp': self.node.get_clock().now().nanoseconds / 1e9,
            'status': 'no_candidates',
            'reason': 'no_cluster_matched_pillar_geometry',
            'candidates': [],
        }
        message = String()
        message.data = json.dumps(no_detection)
        self.pillar_publisher.publish(message)
        self._spin_until(lambda: len(self.messages) > map_message_count, 5.0)
        remaining = json.loads(self.messages[-1].data)
        self.assertEqual(
            [landmark['id'] for landmark in remaining['landmarks'] if landmark['type'] == 'pillar'],
            ['pillar_1'],
        )

        future = self.save_client.call_async(Trigger.Request())
        self._spin_until(lambda: future.done(), 5.0)
        self.assertTrue(future.result().success, future.result().message)
        saved = json.loads(semantic_map_path.read_text(encoding='utf-8'))
        self.assertEqual(
            [landmark['id'] for landmark in saved['landmarks'] if landmark['type'] == 'pillar'],
            ['pillar_1'],
        )
        self.assertEqual(len(saved['road_nodes']), 4)
        self.assertTrue(all(
            node['node_type'] == 'pillar_anchor' for node in saved['road_nodes']))
        self._spin_until(
            lambda: any(
                marker.action == Marker.ADD and marker.ns == 'pillar_landmark'
                for array in self.pillar_marker_messages for marker in array.markers
            ),
            5.0,
        )
        self.assertTrue(any(
            marker.header.frame_id == 'map' and marker.ns == 'pillar_landmark'
            for array in self.pillar_marker_messages for marker in array.markers
        ))
        self._spin_until(
            lambda: any(
                marker.action == Marker.ADD and marker.ns == 'virtual_road_node'
                for array in self.road_node_marker_messages for marker in array.markers
            ),
            5.0,
        )
