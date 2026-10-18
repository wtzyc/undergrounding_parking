import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('parking_lot_navigation')
    default_params = os.path.join(
        package_share,
        'config',
        'nav2_ackermann.yaml',
    )
    default_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_ackermann.xml',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    common_parameters = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]
    common_arguments = ['--ros-args', '--log-level', log_level]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common_parameters,
            arguments=common_arguments,
            remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common_parameters,
            arguments=common_arguments,
            remappings=tf_remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common_parameters,
            arguments=common_arguments,
            remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=common_parameters,
            arguments=common_arguments,
            remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=common_parameters,
            arguments=common_arguments,
            remappings=tf_remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=common_parameters + [{
                'default_nav_to_pose_bt_xml': default_bt,
            }],
            arguments=common_arguments,
            remappings=tf_remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
            }],
            arguments=common_arguments,
        ),
    ])
