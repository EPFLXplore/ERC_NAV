import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_aruco_with_lidar')
    # rviz_config_file = os.path.join(package_dir, 'rviz', 'dual_cam_setup.rviz')

    # rviz = LaunchConfiguration('rviz', default='false')

    tolerance_deg = LaunchConfiguration('tolerance_deg')
    tolerance_radius = LaunchConfiguration('tolerance_radius')
    hauteur_z_min = LaunchConfiguration('hauteur_z_min')
    hauteur_z_max = LaunchConfiguration('hauteur_z_max')

    distance_threshold_inliers = LaunchConfiguration('distance_threshold_inliers')
    max_iterations = LaunchConfiguration('max_iterations')
    t = LaunchConfiguration('t')
    min_inliers = LaunchConfiguration('min_inliers')
    max_lines = LaunchConfiguration('max_lines')
    max_distance_from_aruco = LaunchConfiguration('max_distance_from_aruco')
    angular_tolerance_deg = LaunchConfiguration('angular_tolerance_deg')


    return LaunchDescription([
        # lidar filter
        DeclareLaunchArgument('tolerance_deg', default_value='15.0'),
        DeclareLaunchArgument('tolerance_radius', default_value='1.5'), #1.5
        DeclareLaunchArgument('hauteur_z_min', default_value='-0.7'),
        DeclareLaunchArgument('hauteur_z_max', default_value='1.0'),
        DeclareLaunchArgument('distance_threshold_inliers', default_value='0.05'),
        DeclareLaunchArgument('max_iterations', default_value='100'),
        DeclareLaunchArgument('t', default_value='0.25'),
        DeclareLaunchArgument('min_inliers', default_value='10'),
        DeclareLaunchArgument('max_lines', default_value='3'),
        # before ransac
        DeclareLaunchArgument('max_distance_from_aruco', default_value='0.3'),
        DeclareLaunchArgument('angular_tolerance_deg', default_value='10.0'),

        Node(
            package='ros2_aruco_with_lidar',
            executable='detect_cube',
            name='detect_cube',
            output='screen',
            parameters=[{
                'distance_threshold_inliers': distance_threshold_inliers,
                'max_iterations': max_iterations,
                't': t,
                'min_inliers': min_inliers,
                'max_lines': max_lines,
                'max_distance_from_aruco': max_distance_from_aruco,
                'angular_tolerance_deg': angular_tolerance_deg,
            }],
        ),
  
        Node(
            package='ros2_aruco_with_lidar',
            executable='lidar_phi_filter_node',
            name='lidar_phi_filter_node',
            output='screen',
            parameters=[{
                'tolerance_deg': tolerance_deg,
                'tolerance_radius': tolerance_radius,
                'hauteur_z_min': hauteur_z_min,
                'hauteur_z_max': hauteur_z_max,
            }],
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        #     condition=IfCondition(rviz)
        # ),

        # Node(
        #     package='ros2_aruco',
        #     executable='plot_arucos',
        #     name='plot_arucos',
        #     output='screen',
        # ),

    ])
