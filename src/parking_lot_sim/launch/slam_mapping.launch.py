import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition

from parking_lot_sim.slam_config import validate_slam_toolbox


def generate_launch_description():
    validate_slam_toolbox()
    package_share = get_package_share_directory('parking_lot_sim')
    default_params = os.path.join(
        package_share,
        'config',
        'slam_toolbox_mapping.yaml',
    )

    scan_topic = LaunchConfiguration('scan_topic')
    scan_frame = LaunchConfiguration('scan_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics')
    enable_rviz = LaunchConfiguration('enable_rviz')
    autostart = LaunchConfiguration('autostart')
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'parking_lot_mapping.rviz',
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'scan_topic': scan_topic,
                'use_lifecycle_manager': False,
            },
        ],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[parking_lot_sim] slam_toolbox is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic consumed by slam_toolbox',
        ),
        DeclareLaunchArgument(
            'scan_frame',
            default_value='my_orange_car/lidar_link/lidar_sensor',
            description='Frame id reported by the Gazebo LaserScan message',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/model/my_orange_car/odometry',
            description='Odometry topic converted to odom -> base_footprint TF',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params,
            description='slam_toolbox mapping parameter file',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Configure and activate slam_toolbox automatically',
        ),
        DeclareLaunchArgument('enable_diagnostics', default_value='true'),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Open RViz2 with the online mapping displays',
        ),
        Node(
            package='parking_lot_sim',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            parameters=[{
                'odom_topic': odom_topic,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_static_tf',
            arguments=[
                '--x', '2.5',
                '--y', '0',
                '--z', '1.6',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', scan_frame,
            ],
            output='screen',
        ),
        slam_node,
        configure_event,
        activate_event,
        Node(
            package='parking_lot_sim',
            executable='slam_diagnostics',
            name='slam_diagnostics',
            parameters=[
                {
                    'scan_topic': scan_topic,
                    'odom_topic': odom_topic,
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
            output='screen',
            condition=IfCondition(enable_diagnostics),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping_view',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen',
            condition=IfCondition(enable_rviz),
        ),
    ])
