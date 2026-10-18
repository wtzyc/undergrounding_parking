"""Configuration and dependency checks for the 2-D SLAM launch entry."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    PackageNotFoundError,
)


SLAM_TOOLBOX_PACKAGE = 'slam_toolbox'
SLAM_TOOLBOX_EXECUTABLE = 'async_slam_toolbox_node'


def validate_slam_toolbox() -> Path:
    """Return the installed slam_toolbox prefix or raise a useful error."""
    try:
        prefix = Path(get_package_prefix(SLAM_TOOLBOX_PACKAGE))
    except PackageNotFoundError as exc:
        raise RuntimeError(
            '未找到 slam_toolbox。请先安装 ROS 2 Jazzy 依赖：'
            'sudo apt install ros-jazzy-slam-toolbox'
        ) from exc

    executable = prefix / 'lib' / SLAM_TOOLBOX_PACKAGE / SLAM_TOOLBOX_EXECUTABLE
    if not executable.exists():
        raise RuntimeError(
            f'已找到 slam_toolbox，但缺少可执行文件：{executable}。'
            '请重新安装 ros-jazzy-slam-toolbox。'
        )
    return prefix
