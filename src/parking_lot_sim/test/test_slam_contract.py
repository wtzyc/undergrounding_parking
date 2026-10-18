from pathlib import Path

import parking_lot_sim.slam_config as slam_config
import pytest


def test_missing_slam_toolbox_has_an_actionable_error(monkeypatch):
    def missing_package(_name):
        raise slam_config.PackageNotFoundError('slam_toolbox')

    monkeypatch.setattr(slam_config, 'get_package_prefix', missing_package)

    with pytest.raises(RuntimeError, match='ros-jazzy-slam-toolbox'):
        slam_config.validate_slam_toolbox()


def test_mapping_config_keeps_the_simulation_frames_and_scan_topic():
    config = Path(__file__).parents[1] / 'config' / 'slam_toolbox_mapping.yaml'
    text = config.read_text(encoding='utf-8')

    assert 'mode: mapping' in text
    assert 'map_frame: map' in text
    assert 'odom_frame: odom' in text
    assert 'base_frame: base_footprint' in text
    assert 'scan_topic: /scan' in text
    assert 'use_map_saver: true' in text


def test_mapping_launch_uses_the_gazebo_lidar_frame():
    launch_file = Path(__file__).parents[1] / 'launch' / 'slam_mapping.launch.py'
    text = launch_file.read_text(encoding='utf-8')

    assert "'scan_frame'" in text
    assert "'my_orange_car/lidar_link/lidar_sensor'" in text


def test_online_rviz_config_contains_map_scan_and_tf_displays():
    config = Path(__file__).parents[1] / 'rviz' / 'parking_lot_mapping.rviz'
    text = config.read_text(encoding='utf-8')

    assert 'Fixed Frame: map' in text
    assert 'Value: /map' in text
    assert 'Value: /scan' in text
    assert 'Class: rviz_default_plugins/TF' in text
    assert 'Class: rviz_default_plugins/MarkerArray' in text
    assert 'Value: /semantic/parking_slot_markers' in text
