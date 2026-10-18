import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

from parking_lot_sim.recording import (
    DEFAULT_RECORD_TOPICS,
    ensure_fresh_recording_destination,
    write_recording_manifest,
)


def generate_launch_description():
    workspace_path = os.path.expanduser('~/ros2_ws')

    # Gazebo 模型目录
    default_models_path = os.path.join(
        workspace_path,
        'src',
        'models',
    )

    # 停车场世界文件
    default_world_file = os.path.join(
        workspace_path,
        'src',
        'world',
        'track_world.sdf',
    )
    default_dataset_dir = os.path.join(
        workspace_path,
        'dataset',
        'raw',
    )
    default_semantic_map_path = os.path.join(
        workspace_path,
        'runtime',
        'maps',
        'parking_lot_semantic_map.json',
    )

    models_path = LaunchConfiguration('models_path')
    world_file = LaunchConfiguration('world_file')
    dataset_dir = LaunchConfiguration('dataset_dir')
    collection_save_interval = LaunchConfiguration('collection_save_interval')
    enable_data_collector = LaunchConfiguration('enable_data_collector')
    enable_bev_projector = LaunchConfiguration('enable_bev_projector')
    enable_parking_slot_detector = LaunchConfiguration('enable_parking_slot_detector')
    enable_parking_slot_debug_view = LaunchConfiguration(
        'enable_parking_slot_debug_view'
    )
    enable_parking_slot_map_fusion = LaunchConfiguration(
        'enable_parking_slot_map_fusion'
    )
    enable_pillar_detector = LaunchConfiguration('enable_pillar_detector')
    parking_slot_length_m = LaunchConfiguration('parking_slot_length_m')
    parking_slot_width_m = LaunchConfiguration('parking_slot_width_m')
    enable_rosbag_recording = LaunchConfiguration('enable_rosbag_recording')
    bag_output_dir = LaunchConfiguration('bag_output_dir')
    semantic_map_path = LaunchConfiguration('semantic_map_path')
    slot_lane_extension_m = LaunchConfiguration('slot_lane_extension_m')
    pillar_extension_m = LaunchConfiguration('pillar_extension_m')
    lane_center_x = LaunchConfiguration('lane_center_x')
    lane_center_y = LaunchConfiguration('lane_center_y')
    headless = LaunchConfiguration('headless')
    enable_front_camera = PythonExpression([
        "str('", enable_data_collector, "').lower() == 'true' or ",
        "str('", enable_rosbag_recording, "').lower() == 'true'",
    ])

    def prepare_manifest(context):
        if context.perform_substitution(enable_rosbag_recording).lower() != 'true':
            return []
        ensure_fresh_recording_destination(
            context.perform_substitution(bag_output_dir)
        )
        write_recording_manifest(
            context.perform_substitution(bag_output_dir),
            world_file=context.perform_substitution(world_file),
            models_path=context.perform_substitution(models_path),
            topics=DEFAULT_RECORD_TOPICS,
            collection_save_interval=context.perform_substitution(
                collection_save_interval
            ),
        )
        return []

    # 启动 Gazebo Sim
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 2 ', world_file],
        }.items(),
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering -v 2 ', world_file],
        }.items(),
        condition=IfCondition(headless),
    )

    # Gazebo Transport 与 ROS 2 话题桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Gazebo -> ROS 2
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/camera_god/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_god/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/my_orange_car/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/my_orange_car/joint_states'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/luxury_parking_lot_expanded/pose/info'
            '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # ROS 2 -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )
    front_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        condition=IfCondition(enable_front_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'models_path',
            default_value=default_models_path,
        ),

        DeclareLaunchArgument(
            'world_file',
            default_value=default_world_file,
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server and sensor renderer, without the 3D GUI',
        ),
        DeclareLaunchArgument(
            'dataset_dir',
            default_value=default_dataset_dir,
        ),
        DeclareLaunchArgument(
            'collection_save_interval',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'enable_data_collector',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'enable_bev_projector',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_parking_slot_detector',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_parking_slot_debug_view',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'parking_slot_length_m',
            default_value='5.8',
        ),
        DeclareLaunchArgument(
            'parking_slot_width_m',
            default_value='2.8',
        ),
        DeclareLaunchArgument(
            'enable_parking_slot_map_fusion',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_pillar_detector',
            default_value='true',
            description='Publish vehicle-relative LaserScan pillar candidates',
        ),
        DeclareLaunchArgument(
            'semantic_map_path',
            default_value=default_semantic_map_path,
            description='JSON path used to load/save the semantic parking-slot map',
        ),
        DeclareLaunchArgument(
            'slot_lane_extension_m',
            default_value='2.07',
            description='Distance from the parking-slot boundary into the lane for its virtual node (1.8 m increased by 15 percent)',
        ),
        DeclareLaunchArgument(
            'pillar_extension_m',
            default_value='2.0',
            description='Distance from a pillar for each clear-direction virtual node',
        ),
        DeclareLaunchArgument('lane_center_x', default_value='0.0'),
        DeclareLaunchArgument('lane_center_y', default_value='0.0'),
        DeclareLaunchArgument(
            'road_node_max_neighbor_distance_m',
            default_value='5.0',
            description='Maximum distance for connecting virtual road-node neighbors',
        ),
        DeclareLaunchArgument(
            'pillar_slot_adjacency_max_distance_m',
            default_value='8.0',
            description='Maximum distance for removing pillar directions that point into an adjacent parking slot',
        ),
        DeclareLaunchArgument(
            'enable_rosbag_recording',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'bag_output_dir',
            default_value=os.path.join(
                workspace_path,
                'dataset',
                'bags',
                'latest',
            ),
            description='rosbag2 output directory; use a fresh path for each run',
        ),

        # 添加模型目录，同时保留系统原有资源路径
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                models_path,
                os.pathsep,
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH',
                    default_value='',
                ),
            ],
        ),

        gz_sim_gui,
        gz_sim_headless,
        bridge,
        front_camera_bridge,
        OpaqueFunction(function=prepare_manifest),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '-o',
                bag_output_dir,
                *DEFAULT_RECORD_TOPICS,
            ],
            output='screen',
            condition=IfCondition(enable_rosbag_recording),
        ),
        Node(
            package='parking_lot_sim',
            executable='data_collector',
            parameters=[{
                'output_dir': dataset_dir,
                'truth_pose_topic': '/world/luxury_parking_lot_expanded/pose/info',
                'save_interval_sec': collection_save_interval,
            }],
            output='screen',
            condition=IfCondition(enable_data_collector),
        ),
        Node(
            package='parking_lot_sim',
            executable='bev_projector',
            output='screen',
            condition=IfCondition(enable_bev_projector),
        ),
        Node(
            package='parking_lot_perception',
            executable='parking_slot_bev_node',
            parameters=[{
                'expected_length_m': parking_slot_length_m,
                'expected_width_m': parking_slot_width_m,
                'resolution_m_per_pixel': 0.02,
                'x_min': -8.0,
                'y_min': -8.0,
                'base_frame': 'base_footprint',
            }],
            output='screen',
            condition=IfCondition(enable_parking_slot_detector),
        ),
        Node(
            package='parking_lot_perception',
            executable='parking_slot_map_fusion',
            parameters=[{
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'map_path': semantic_map_path,
                'tf_tolerance_sec': 0.1,
                'match_distance_m': 0.5,
                'match_angle_deg': 15.0,
                'duplicate_merge_distance_m': 1.2,
                'duplicate_merge_angle_deg': 10.0,
                'duplicate_dimension_tolerance_m': 0.6,
                'confirm_hits': 10,
                'stability_position_tolerance_m': 0.15,
                'stability_angle_tolerance_deg': 5.0,
                'stability_dimension_tolerance_m': 0.15,
                'max_missed_frames': 5,
                'pillar_side_length_m': 1.2,
                'pillar_min_confidence': 0.35,
                'pillar_same_frame_merge_distance_m': 0.6,
                'pillar_duplicate_merge_distance_m': 0.8,
                'pillar_marker_topic': '/semantic/pillar_landmark_markers',
                'road_node_marker_topic': '/semantic/virtual_road_node_markers',
                'slot_lane_extension_m': slot_lane_extension_m,
                'pillar_extension_m': pillar_extension_m,
                'lane_center_x': lane_center_x,
                'lane_center_y': lane_center_y,
                'pillar_slot_adjacency_max_distance_m': LaunchConfiguration(
                    'pillar_slot_adjacency_max_distance_m'),
                'road_node_obstacle_clearance_m': 0.10,
                'road_node_duplicate_distance_m': 0.25,
                'road_node_max_neighbor_distance_m': LaunchConfiguration(
                    'road_node_max_neighbor_distance_m'),
            }],
            output='screen',
            condition=IfCondition(enable_parking_slot_map_fusion),
        ),
        Node(
            package='parking_lot_perception',
            executable='pillar_scan_detector_node',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/scan',
                'output_topic': '/semantic/pillar_candidates',
                'marker_topic': '/semantic/pillar_candidate_markers',
                'base_frame': 'base_footprint',
                'expected_pillar_width_m': 1.2,
                'min_observed_width_m': 0.3,
                'max_observed_width_m': 2.1,
                'cluster_gap_m': 0.15,
                'angular_gap_scale': 1.2,
                'min_cluster_points': 3,
            }],
            output='screen',
            condition=IfCondition(enable_pillar_detector),
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/camera_god/parking_slot_debug_image'],
            output='screen',
            condition=IfCondition(enable_parking_slot_debug_view),
        ),
    ])
