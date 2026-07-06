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
    cpp_share = get_package_share_directory('ros2_aruco_cpp')
    camera_config_dir = os.path.join(cpp_share, 'config')
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
    max_planes = LaunchConfiguration('max_planes')
    plane_group_centroid_max_m = LaunchConfiguration('plane_group_centroid_max_m')
    face_dimension_tolerance_m = LaunchConfiguration('face_dimension_tolerance_m')
    side_perpendicular_dot_max = LaunchConfiguration('side_perpendicular_dot_max')
    top_vertical_dot_min = LaunchConfiguration('top_vertical_dot_min')
    face_min_short_frac = LaunchConfiguration('face_min_short_frac')
    side_min_long_frac = LaunchConfiguration('side_min_long_frac')
    top_min_long_frac = LaunchConfiguration('top_min_long_frac')
    face_score_tolerance_multiplier = LaunchConfiguration('face_score_tolerance_multiplier')
    max_face_diagonal_multiplier = LaunchConfiguration('max_face_diagonal_multiplier')
    max_center_error_m = LaunchConfiguration('max_center_error_m')
    accept_best_plane_fallback = LaunchConfiguration('accept_best_plane_fallback')
    process_rate_hz = LaunchConfiguration('process_rate_hz')
    marker_lifetime_sec = LaunchConfiguration('marker_lifetime_sec')
    max_distance_from_aruco = LaunchConfiguration('max_distance_from_aruco')
    angular_tolerance_deg = LaunchConfiguration('angular_tolerance_deg')



    return LaunchDescription([
        # ---- ArUco camera detection (from ros2_aruco_cpp) ----
        Node(
            package='ros2_aruco_cpp',
            executable='multiview_aruco_node',
            name='aruco_node',
            parameters=[config, {
                'camera_config_dir': camera_config_dir,
            }],
            output='screen',
        ),

        # ---- Pose estimation / triangulation (from ros2_aruco_cpp) ----
        Node(
            package='ros2_aruco_cpp',
            executable='pose_estimator_lidar_node',
            name='pose_estimation_node_with_lidar',
            parameters=[config],
            output='screen',
        ),

        # ---- Lidar filter params ----
        # lidar filter
        DeclareLaunchArgument('tolerance_deg', default_value='5.0'),
        DeclareLaunchArgument('tolerance_radius', default_value='2.0'), #1.5 HAS TO BE A TYPE DOUBLE
        DeclareLaunchArgument('hauteur_z_min', default_value='0.0'),
        DeclareLaunchArgument('hauteur_z_max', default_value='0.75'),
        DeclareLaunchArgument('distance_threshold_inliers', default_value='0.05'),
        DeclareLaunchArgument('max_iterations', default_value='300'),
        DeclareLaunchArgument('t', default_value='0.25'),
        DeclareLaunchArgument('min_inliers', default_value='20'),
        DeclareLaunchArgument('max_lines', default_value='3'),
        DeclareLaunchArgument('max_planes', default_value='3'),
        DeclareLaunchArgument('plane_group_centroid_max_m', default_value='0.80'),
        DeclareLaunchArgument('face_dimension_tolerance_m', default_value='0.25'),
        DeclareLaunchArgument('side_perpendicular_dot_max', default_value='0.75'),
        DeclareLaunchArgument('top_vertical_dot_min', default_value='0.35'),
        DeclareLaunchArgument('face_min_short_frac', default_value='0.20'),
        DeclareLaunchArgument('side_min_long_frac', default_value='0.20'),
        DeclareLaunchArgument('top_min_long_frac', default_value='0.20'),
        DeclareLaunchArgument('face_score_tolerance_multiplier', default_value='5.0'),
        DeclareLaunchArgument('max_face_diagonal_multiplier', default_value='1.1'),
        DeclareLaunchArgument('max_center_error_m', default_value='3.0'),
        DeclareLaunchArgument('accept_best_plane_fallback', default_value='true'),
        DeclareLaunchArgument('process_rate_hz', default_value='13.0'),
        DeclareLaunchArgument('marker_lifetime_sec', default_value='1.0'),
        # before ransac
        DeclareLaunchArgument('max_distance_from_aruco', default_value='1.0'), #has to be a type double
        DeclareLaunchArgument('angular_tolerance_deg', default_value='20.0'),

        Node(
            package='ros2_aruco_with_lidar',
            executable='detect_cube',
            name='detect_cube',
            output='screen',
            remappings=[
                ('/perception/lidar_cube_markers', '/cube_markers'),
            ],
            parameters=[{
                'distance_threshold_inliers': distance_threshold_inliers,
                'max_iterations': max_iterations,
                't': t,
                'min_inliers': min_inliers,
                'max_lines': max_lines,
                'max_planes': max_planes,
                'plane_group_centroid_max_m': plane_group_centroid_max_m,
                'face_dimension_tolerance_m': face_dimension_tolerance_m,
                'side_perpendicular_dot_max': side_perpendicular_dot_max,
                'top_vertical_dot_min': top_vertical_dot_min,
                'face_min_short_frac': face_min_short_frac,
                'side_min_long_frac': side_min_long_frac,
                'top_min_long_frac': top_min_long_frac,
                'face_score_tolerance_multiplier': face_score_tolerance_multiplier,
                'max_face_diagonal_multiplier': max_face_diagonal_multiplier,
                'max_center_error_m': max_center_error_m,
                'accept_best_plane_fallback': accept_best_plane_fallback,
                'process_rate_hz': process_rate_hz,
                'marker_lifetime_sec': marker_lifetime_sec,
                'max_distance_from_aruco': max_distance_from_aruco,
                'angular_tolerance_deg': angular_tolerance_deg,
                # Matches lidar_phi_filter_node association publisher.
                'aruco_topic': '/perception/aruco_markers_for_lidar_association',
            }],
        ),
  
        Node(
            package='ros2_aruco_with_lidar',
            executable='lidar_phi_filter_node',
            name='lidar_phi_filter_node',
            output='screen',
            remappings=[
                ('/perception/camera_forbidden_sector_cube_markers', '/cube_markers_phi'),
            ],
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
