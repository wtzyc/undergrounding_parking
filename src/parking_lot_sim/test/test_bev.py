from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from parking_lot_sim.bev import BEVConfig, BEVError, BEVProjector
from parking_lot_sim.bev import CameraExtrinsics, CameraIntrinsics
import pytest


def _projector():
    intrinsics = CameraIntrinsics(
        width=100,
        height=100,
        fx=50.0,
        fy=50.0,
        cx=50.0,
        cy=50.0,
    )
    extrinsics = CameraExtrinsics(
        x=0.0,
        y=0.0,
        z=1.0,
        rotation_camera_to_base=(
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
        ),
    )
    config = BEVConfig(
        x_min=0.0,
        x_max=4.0,
        y_min=-2.0,
        y_max=2.0,
        resolution_m_per_pixel=0.1,
    )
    return BEVProjector(intrinsics, extrinsics, config)


def test_ground_projection_uses_vehicle_coordinates():
    projector = _projector()

    assert projector.pixel_to_ground(50.0, 75.0) == pytest.approx((2.0, 0.0))
    assert projector.pixel_to_ground(60.0, 75.0) == pytest.approx((2.0, -0.4))
    assert projector.ground_to_pixel(2.0, 0.0) == pytest.approx((50.0, 75.0))
    assert projector.ground_to_bev(2.0, 0.0) == pytest.approx((19.0, 19.0))
    assert projector.bev_to_ground(19.0, 19.0) == pytest.approx((2.0, 0.0))
    assert projector.ground_to_bev(0.0, -2.0) == pytest.approx((39.0, 39.0))
    assert projector.bev_to_ground(39.0, 39.0) == pytest.approx((0.0, -2.0))


def test_invalid_rays_and_bounds_return_no_coordinate():
    projector = _projector()

    assert projector.pixel_to_ground(-1.0, 50.0) is None
    assert projector.pixel_to_ground(50.0, 50.0) is None
    assert projector.ground_to_pixel(-1.0, 0.0) is None
    assert projector.ground_to_bev(5.0, 0.0) is None
    assert projector.bev_to_ground(-1.0, 0.0) is None


def test_warp_contains_metadata_mask_and_projected_ground_marker():
    projector = _projector()
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(image, (50, 75), 3, (0, 0, 255), -1)
    cv2.circle(image, (80, 75), 3, (0, 255, 0), -1)

    result = projector.warp(image, stamp=12.5, source_frame='camera_god')
    debug = projector.debug_overlay(image)

    assert result.image.shape == (40, 40, 3)
    assert result.valid_mask.shape == (40, 40)
    assert result.metadata['stamp'] == 12.5
    assert result.metadata['frame_id'] == 'base_footprint'
    assert result.metadata['source_frame'] == 'camera_god'
    assert result.metadata['image_rotation_deg'] == 180
    assert result.metadata['pixel_origin_base'] == {'x': 3.9, 'y': 1.9}
    assert result.metadata['pixel_axes'] == {'row': '-x', 'column': '-y'}
    assert result.metadata['valid_pixel_ratio'] > 0.0
    assert int(result.image[20, 20, 2]) > 0
    assert int(result.image[19, 31, 1]) > 0
    assert np.count_nonzero(debug) > 0


def test_invalid_camera_parameters_are_rejected():
    with pytest.raises(BEVError, match='fx and fy'):
        CameraIntrinsics(100, 100, 0.0, 50.0, 50.0, 50.0)

    with pytest.raises(BEVError, match='orthonormal'):
        CameraExtrinsics(
            0.0,
            0.0,
            1.0,
            (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0),
        )

    with pytest.raises(BEVError, match='positive area'):
        BEVConfig(1.0, 1.0, -1.0, 1.0, 0.1)


def test_from_base_rpy_builds_a_valid_optical_orientation():
    extrinsics = CameraExtrinsics.from_base_rpy(0.0, 0.0, 1.0)
    projector = BEVProjector(
        CameraIntrinsics(20, 20, 10.0, 10.0, 10.0, 10.0),
        extrinsics,
        BEVConfig(0.0, 2.0, -1.0, 1.0, 0.1),
    )

    assert projector.pixel_to_ground(10.0, 15.0) == pytest.approx((2.0, 0.0))


def test_bev_node_defaults_to_the_god_camera():
    node_file = Path(__file__).parents[1] / 'parking_lot_sim' / 'nodes' / 'bev_projector.py'
    text = node_file.read_text(encoding='utf-8')
    assert "camera_topic', '/camera_god/image'" in text
    assert "camera_info_topic', '/camera_god/camera_info'" in text
    assert 'camera_god' in text
    assert '1.5707963267948966' in text
    assert 'camera_x' not in text


def test_simulation_launch_exposes_optional_bev_node():
    launch_file = Path(__file__).parents[1] / 'launch' / 'parking_lot.launch.py'
    text = launch_file.read_text(encoding='utf-8')
    assert 'enable_bev_projector' in text
    assert "executable='bev_projector'" in text
    assert '/camera_god/image@sensor_msgs/msg/Image' in text
    assert '/camera_god/camera_info@sensor_msgs/msg/CameraInfo' in text


def test_auxiliary_camera_bridge_is_only_started_for_data_collection_or_recording():
    launch_file = Path(__file__).parents[1] / 'launch' / 'parking_lot.launch.py'
    text = launch_file.read_text(encoding='utf-8')

    assert 'front_camera_bridge' in text
    assert 'enable_data_collector' in text
    assert 'enable_rosbag_recording' in text
    assert 'condition=IfCondition(enable_front_camera)' in text
    for unused_topic in (
        '/camera/image',
        '/camera/camera_info',
        '/camera_left/image',
        '/camera_left/camera_info',
        '/camera_right/image',
        '/camera_right/camera_info',
        '/camera_rear/image',
        '/camera_rear/camera_info',
    ):
        assert unused_topic not in text


def test_robot_model_sleeps_unused_cameras_and_runs_god_camera_at_10_hz():
    model_file = Path(__file__).parents[2] / 'models' / 'orange_audibot' / 'model.sdf'
    root = ET.parse(model_file).getroot()
    sensors = {sensor.attrib['name']: sensor for sensor in root.findall('.//sensor')}

    assert sensors['cam_god_sensor'].findtext('always_on') == '1'
    assert sensors['cam_god_sensor'].findtext('update_rate') == '10'
    for sensor_name in (
        'camera_sensor',
        'cam_left_sensor',
        'cam_right_sensor',
        'cam_front_sensor',
        'cam_rear_sensor',
    ):
        assert sensors[sensor_name].findtext('always_on') == '0'


def test_simulation_has_a_headless_low_load_mode():
    launch_file = Path(__file__).parents[1] / 'launch' / 'parking_lot.launch.py'
    text = launch_file.read_text(encoding='utf-8')

    assert "'headless'" in text
    assert '-s --headless-rendering' in text
    assert 'condition=IfCondition(headless)' in text


def test_world_uses_bounded_physics_and_sensor_rendering_load():
    world_file = Path(__file__).parents[2] / 'world' / 'track_world.sdf'
    world_root = ET.parse(world_file).getroot()
    physics = world_root.find('.//physics')
    assert physics is not None
    assert physics.findtext('max_step_size') == '0.005'
    assert physics.findtext('real_time_update_rate') == '200'

    model_file = Path(__file__).parents[2] / 'models' / 'orange_audibot' / 'model.sdf'
    model_root = ET.parse(model_file).getroot()
    lidar = model_root.find(".//sensor[@name='lidar_sensor']")
    assert lidar is not None
    assert lidar.findtext('visualize') == 'false'
