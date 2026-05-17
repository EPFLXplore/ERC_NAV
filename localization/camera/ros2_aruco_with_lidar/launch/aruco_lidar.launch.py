import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def _aruco_params_yaml_path():
    """Same resolution as ros2_aruco_cpp/launch/aruco.launch.py (install or symlink-install)."""
    share = get_package_share_directory('ros2_aruco_cpp')
    in_install = os.path.join(share, 'config', 'aruco_params.yaml')
    cpp_launch = os.path.join(share, 'launch', 'aruco.launch.py')
    launch_dir = os.path.dirname(os.path.realpath(cpp_launch)) if os.path.isfile(cpp_launch) else ''
    next_to_cpp_launch = (
        os.path.normpath(os.path.join(launch_dir, '..', 'config', 'aruco_params.yaml'))
        if launch_dir
        else ''
    )
    for p in (in_install, next_to_cpp_launch):
        if p and os.path.isfile(p):
            return p
    raise FileNotFoundError(
        'aruco_params.yaml not found under ros2_aruco_cpp. Tried:\n'
        f'  {in_install}\n'
        f'  {next_to_cpp_launch or "(ros2_aruco_cpp launch not found)"}\n'
        'Rebuild: colcon build --packages-select ros2_aruco_cpp && source install/setup.bash'
    )


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_aruco_with_lidar')
    config = _aruco_params_yaml_path()
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
        DeclareLaunchArgument('tolerance_deg', default_value='5.0'),
        DeclareLaunchArgument('tolerance_radius', default_value='0.8'), #1.5
        DeclareLaunchArgument('hauteur_z_min', default_value='-0.5'),
        DeclareLaunchArgument('hauteur_z_max', default_value='1.0'),
        DeclareLaunchArgument('distance_threshold_inliers', default_value='0.05'),
        DeclareLaunchArgument('max_iterations', default_value='20'),
        DeclareLaunchArgument('t', default_value='0.25'),
        DeclareLaunchArgument('min_inliers', default_value='30'),
        DeclareLaunchArgument('max_lines', default_value='3'),
        # before ransac
        DeclareLaunchArgument('max_distance_from_aruco', default_value='1.3'),
        DeclareLaunchArgument('angular_tolerance_deg', default_value='100.0'),

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
                # Matches lidar_phi_filter_node publisher aruco_markers_lidar_assoc
                'aruco_topic': 'aruco_markers_lidar_assoc',
            }],
        ),
  
        Node(
            package='ros2_aruco_with_lidar',
            executable='lidar_phi_filter_node',
            name='lidar_phi_filter_node',
            output='screen',
            parameters=[
                config,
                {
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
