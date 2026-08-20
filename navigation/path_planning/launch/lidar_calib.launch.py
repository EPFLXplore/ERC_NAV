"""Camera-free LiDAR calibration bring-up.

Launch order:
  1. Manual rover stack: motors, odometry, Ouster, and point-cloud corrector.
  2. Local traversability costmap after the LiDAR has initialized.
  3. Nav2 after the hardware and costmap have had time to start.
  4. LiDAR calibrator, which waits for GLIM before commanding the 3 m goal.

GLIM is intentionally launched separately because it runs in its own container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_frame_id = LaunchConfiguration('goal_frame_id')
    travel_distance_m = LaunchConfiguration('travel_distance_m')
    sample_count = LaunchConfiguration('sample_count')
    settle_time_sec = LaunchConfiguration('settle_time_sec')
    calibration_file = LaunchConfiguration('calibration_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_file = LaunchConfiguration('map_file')

    hardware_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('wheels_control'), 'launch', 'manual_stack.launch.py',
        ])),
    )

    # Calibration runs without ArUco localization, which normally owns map -> odom.
    # Keep the Nav2 TF tree connected while the rover performs the straight test drive.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='calibration_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    gradient_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gradient_costmap'), 'launch', 'gradient_costmap.launch.py',
        ])),
        launch_arguments={'gradient_mode': 'local'}.items(),
    )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('path_planning'), 'launch', 'navigation_launch.py',
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'false',
            'params_file': nav2_params_file,
            'map': map_file,
        }.items(),
    )

    calibrator = Node(
        package='path_planning',
        executable='lidar_calibrator.py',
        name='lidar_calibrator',
        output='screen',
        parameters=[{
            'goal_x': ParameterValue(goal_x, value_type=float),
            'goal_y': ParameterValue(goal_y, value_type=float),
            'goal_frame_id': goal_frame_id,
            'travel_distance_m': ParameterValue(travel_distance_m, value_type=float),
            'sample_count': ParameterValue(sample_count, value_type=int),
            'settle_time_sec': ParameterValue(settle_time_sec, value_type=float),
            'calibration_file': calibration_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_frame_id', default_value='map'),
        DeclareLaunchArgument('travel_distance_m', default_value='3.0'),
        DeclareLaunchArgument('sample_count', default_value='50'),
        DeclareLaunchArgument('settle_time_sec', default_value='2.0'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/home/xplore/dev_ws/src/sensors/lidar/config/lidar_calibration.yaml',
            description='Source YAML to update after successful calibration.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=(
                '/home/xplore/dev_ws/src/navigation/path_planning/config/'
                'nav2_params_real_2026_with_global_map.yaml'),
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=(
                '/home/xplore/dev_ws/src/navigation/path_planning/saved_maps/2026/'
                'local_inflated_raw.yaml'),
        ),
        hardware_stack,
        map_to_odom,
        TimerAction(period=5.0, actions=[gradient_costmap]),
        TimerAction(period=12.0, actions=[navigation_stack]),
        TimerAction(period=13.0, actions=[calibrator]),
    ])
