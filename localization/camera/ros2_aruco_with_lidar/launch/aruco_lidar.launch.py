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
    use_camera_aruco_position = LaunchConfiguration('use_camera_aruco_position')
    camera_cone_half_angle_deg = LaunchConfiguration('camera_cone_half_angle_deg')
    camera_cone_depth_tolerance_m = LaunchConfiguration('camera_cone_depth_tolerance_m')
    camera_cone_depth_tolerance_ratio = LaunchConfiguration('camera_cone_depth_tolerance_ratio')
    association_ambiguity_margin_m = LaunchConfiguration('association_ambiguity_margin_m')
    auto_recovery_enabled = LaunchConfiguration('auto_recovery_enabled')
    recovery_enter_disagreement_m = LaunchConfiguration('recovery_enter_disagreement_m')
    recovery_exit_disagreement_m = LaunchConfiguration('recovery_exit_disagreement_m')
    recovery_enter_hold_sec = LaunchConfiguration('recovery_enter_hold_sec')
    recovery_exit_hold_sec = LaunchConfiguration('recovery_exit_hold_sec')
    recovery_min_trigger_tags = LaunchConfiguration('recovery_min_trigger_tags')
    recovery_min_lidar_tags = LaunchConfiguration('recovery_min_lidar_tags')

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

        # ---- Global map->odom KF ----
        # Owns map->odom after pose_estimator_lidar_node finishes Phase 2 and
        # hands it the seed on the latched /map_odom_init, then corrects it
        # from the post-init /aruco_rover_pos solutions.  Start order does not
        # matter: the seed topic is transient_local.
        Node(
            package='global_nav_kf',
            executable='global_nav_kf_2d_node',
            name='global_nav_kf_2d',
            parameters=[{
                'broadcast_rate_hz': 20.0,
                # ^ [Hz] map->odom TF rate. Matches the 50 ms republish_tf rate
                #   it replaces in pose_estimator_lidar_node.
                'meas_sigma_xy_m': 0.25,
                # ^ [m] trust in one /aruco_rover_pos solution. High: TF barely
                #   moves, aruco drift correction is slow. Low: TF chases every
                #   solver solution, map->odom gets jittery.
                'meas_sigma_yaw_deg': 5.0,
                # ^ [deg] same for heading; keep near the solver's
                #   bearing_sigma_deg in aruco_params.yaml.
                'process_sigma_xy_m_per_s': 0.02,
                # ^ [m/s] assumed odom drift rate. High: filter forgets fast,
                #   accepts big corrections. Low: filter is stiff, a real drift
                #   takes many measurements to correct.
                'process_sigma_yaw_deg_per_s': 0.5,
                # ^ [deg/s] same for yaw drift (mostly gyro bias).
                'mahalanobis_gate_chi2': 7.815,
                # ^ chi2 outlier gate on 3 DOF: 7.815 = 95%, 11.34 = 99%.
                #   High: mislabeled cubes get in. Low: legitimate corrections
                #   are rejected.
                'max_consecutive_rejects': 10,
                # ^ after this many rejections in a row, P is inflated x4 so a
                #   mis-seeded filter can recover instead of rejecting forever;
                #   inflation is permitted only while the recovery supervisor
                #   has verified and enabled camera-guided recovery.
                'inflate_only_in_camera_recovery': True,
                'stationary_speed_mps': 0.05,
                # ^ [m/s] at/above this the full process noise applies; below
                #   it Q is scaled down proportionally. Odom drifts with
                #   distance travelled, so a parked rover should not be allowed
                #   to wander map->odom. Set near the EKF's velocity noise
                #   floor: too high and Q stays throttled while genuinely
                #   creeping, too low and it never engages.
                'stationary_yaw_rate_dps': 3.0,
                # ^ [deg/s] same threshold for turning in place, which slips
                #   wheels and drifts odom without any linear velocity.
                'stationary_q_scale': 0.05,
                # ^ floor on the Q scale when fully stopped. Low: map->odom is
                #   held nearly frozen while parked (good at waypoints). Too
                #   low: P stops growing, the Mahalanobis gate tightens, and
                #   real corrections get rejected on setting off again.
                'twist_timeout_sec': 0.5,
                # ^ [s] older than this, /fused_nav_ekf_odom is ignored and
                #   full Q is used. Never assume "stationary" from missing
                #   data, that would make the filter silently overconfident.
            }],
            output='screen',
        ),

        # ---- Lidar filter params ----
        # lidar filter (lidar_phi_filter_node: crops the raw cloud around the camera aruco landmarks)
        DeclareLaunchArgument('tolerance_deg', default_value='5.0'),
        # ^ UNUSED: declared/loaded by lidar_phi_filter_node but never used in the filter (legacy phi-sector width)
        DeclareLaunchArgument('tolerance_radius', default_value='1.2'), #1.5 HAS TO BE A TYPE DOUBLE
        # ^ [m] map-guided mode only: keep lidar points within this radius of each landmark
        DeclareLaunchArgument('hauteur_z_min', default_value='0.1'),
        # ^ [m, map in landmark mode; base_link in camera mode] z floor. High: cuts more ground but also the cube bottom (fewer plane inliers). Low: keeps ground points -> ground can be fit as a "face"
        DeclareLaunchArgument('hauteur_z_max', default_value='1.50'),
        # ^ [m, map in landmark mode; base_link in camera mode] z ceiling. High: keeps clutter above the cube (people, structures). Low: risks cutting the cube top (cube is ~0.32 m tall)
        DeclareLaunchArgument('use_camera_aruco_position', default_value='false'),
        # ^ false: search around the ID-associated map landmark; true: search around the camera-measured ArUco center without depending on map/odometry accuracy
        DeclareLaunchArgument('camera_cone_half_angle_deg', default_value='10.0'),
        # ^ [deg] recovery cone half-angle around the camera bearing; lateral half-width grows as range*tan(angle)
        DeclareLaunchArgument('camera_cone_depth_tolerance_m', default_value='1.0'),
        # ^ [m] minimum recovery cone range tolerance on either side of the camera-estimated depth
        DeclareLaunchArgument('camera_cone_depth_tolerance_ratio', default_value='0.50'),
        # ^ recovery range tolerance also grows with distance: max(fixed tolerance, ratio*camera range)
        DeclareLaunchArgument('association_ambiguity_margin_m', default_value='0.05'),
        # ^ [m] discard points/centers whose nearest ArUco ID is not better than the second-nearest by this margin; prevents ID swaps when search zones overlap
        DeclareLaunchArgument('auto_recovery_enabled', default_value='true'),
        # ^ Automatically switch to camera-guided LiDAR search after sustained, multi-tag evidence that map->odom projects landmarks incorrectly
        DeclareLaunchArgument('recovery_enter_disagreement_m', default_value='0.8'),
        # ^ [m] camera-vs-map landmark disagreement required to consider one tag evidence of localization loss
        DeclareLaunchArgument('recovery_exit_disagreement_m', default_value='0.35'),
        # ^ [m] tighter agreement required before returning to map-guided search (hysteresis)
        DeclareLaunchArgument('recovery_enter_hold_sec', default_value='0.6'),
        # ^ [s] continuous qualified failure evidence required before entering camera recovery
        DeclareLaunchArgument('recovery_exit_hold_sec', default_value='2.0'),
        # ^ [s] continuous LiDAR/solver success and camera-map agreement required before returning to map guidance
        DeclareLaunchArgument('recovery_min_trigger_tags', default_value='2'),
        # ^ independent tags that must show a consistent displacement; a single wrong ID cannot trigger global recovery
        DeclareLaunchArgument('recovery_min_lidar_tags', default_value='2'),
        # ^ unique LiDAR cube IDs required together with a solver-approved rover pose before testing map guidance again
        DeclareLaunchArgument('distance_threshold_inliers', default_value='0.05'),
        # ^ [m] RANSAC plane inlier distance. High: rough/curved clutter counts as planes, face dims inflate. Low: crisp planes only, sparse/noisy scans may not reach min_inliers
        DeclareLaunchArgument('max_iterations', default_value='300'),
        # ^ RANSAC iterations per plane. High: more consistent best-plane fit, more CPU per tag. Low: faster but can settle on a suboptimal plane
        DeclareLaunchArgument('t', default_value='0.25'),
        # ^ UNUSED: legacy face width [m] of the old 2D-line detector; the plane path uses cube_width_m instead
        DeclareLaunchArgument('min_inliers', default_value='20'),
        # ^ min points per plane (a range-based dynamic floor also applies). High: only dense/close detections, far cube missed. Low: accepts sparse planes -> more false positives
        DeclareLaunchArgument('max_lines', default_value='3'),
        # ^ UNUSED: legacy limit of the old 2D-line detector
        DeclareLaunchArgument('max_planes', default_value='3'),
        # ^ RANSAC plane extractions per tag. High: digs past ground/walls to find the cube faces (more CPU). Low: only dominant planes, cube missed if clutter is fit first
        DeclareLaunchArgument('plane_group_centroid_max_m', default_value='0.80'),
        # ^ [m] max centroid distance to group a 2nd side face / top with the 1st. High: planes from different objects merged into one cube. Low: valid 2nd face rejected -> center estimated from a single face (less accurate)
        DeclareLaunchArgument('face_dimension_tolerance_m', default_value='0.1'),
        # ^ [m] allowed excess over nominal face dims (also scales the accept score). High: oversized/odd planes accepted as faces. Low: strict size match, partial or noisy faces rejected
        DeclareLaunchArgument('side_perpendicular_dot_max', default_value='0.75'),
        # ^ max |dot| between two side-face normals to pair them (0 = strictly perpendicular). High: near-parallel planes can pair as "two faces". Low: true face pairs dropped when normals are noisy
        DeclareLaunchArgument('top_vertical_dot_min', default_value='0.35'),
        # ^ min |normal.z| for a plane to count as top face. High: only near-horizontal planes are tops. Low: tilted side-ish planes get classified as top
        DeclareLaunchArgument('face_min_short_frac', default_value='0.20'),
        # ^ min short dim as fraction of cube width. High: must see most of the face width. Low: thin slivers accepted as faces
        DeclareLaunchArgument('side_min_long_frac', default_value='0.20'),
        # ^ min side-face long dim as fraction of cube height. High: must see most of the face height. Low: small patches accepted as side faces
        DeclareLaunchArgument('top_min_long_frac', default_value='0.20'),
        # ^ min top-face long dim as fraction of cube width. High: must see most of the top. Low: small patches accepted as top faces
        DeclareLaunchArgument('face_score_tolerance_multiplier', default_value='5.0'),
        # ^ multiplies face_dimension_tolerance_m for the size-score accept gate. High: looser, wrong-sized planes pass. Low: strict, partial views rejected
        DeclareLaunchArgument('max_face_diagonal_multiplier', default_value='1.1'),
        # ^ max plane long dim vs face diagonal (<=0 disables). High: big planes (walls, ground patches) pass as faces. Low: slightly oversized real faces rejected
        DeclareLaunchArgument('max_center_error_m', default_value='3.0'),
        # ^ [m] max gap between computed cube center and expected map landmark (<=0 = auto). High: tolerates localization drift but accepts wrong objects. Low: strict gate, true detections rejected when localization drifts
        DeclareLaunchArgument('accept_best_plane_fallback', default_value='false'),
        # ^ if no plausible face found, take the biggest extracted plane as a side. true: almost always outputs a detection (clutter can become the "cube"). false: silent unless a real face matched
        DeclareLaunchArgument('process_rate_hz', default_value='13.0'),
        # ^ processing throttle. High: fresher detections, more CPU. Low: saves CPU, laggier detections
        DeclareLaunchArgument('marker_lifetime_sec', default_value='1.0'),
        # ^ [s] rviz marker persistence. High: markers linger after the cube moved (ghosting). Low: markers flicker between detections
        # before ransac
        DeclareLaunchArgument('max_distance_from_aruco', default_value='1.0'), #has to be a type double
        # ^ [m] radial band half-width around the expected landmark range (pre-RANSAC point gate). High: more clutter admitted into RANSAC. Low: cube points dropped if the range estimate is off
        DeclareLaunchArgument('angular_tolerance_deg', default_value='20.0'),
        # ^ [deg] angular gate around the expected bearing (pre-RANSAC). High: wide sector, neighbouring objects included. Low: narrow sector, cube missed if bearing estimate is off

        Node(
            package='ros2_aruco_with_lidar',
            executable='aruco_lidar_recovery_manager',
            name='aruco_lidar_recovery_manager',
            output='screen',
            parameters=[{
                'auto_recovery_enabled': auto_recovery_enabled,
                'use_camera_aruco_position': use_camera_aruco_position,
                'enter_disagreement_m': recovery_enter_disagreement_m,
                'exit_disagreement_m': recovery_exit_disagreement_m,
                'enter_hold_sec': recovery_enter_hold_sec,
                'exit_hold_sec': recovery_exit_hold_sec,
                'min_trigger_tags': recovery_min_trigger_tags,
                'min_recovery_lidar_tags': recovery_min_lidar_tags,
            }],
        ),

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
                'use_camera_aruco_position': use_camera_aruco_position,
                'camera_cone_half_angle_deg': camera_cone_half_angle_deg,
                'camera_cone_depth_tolerance_m': camera_cone_depth_tolerance_m,
                'camera_cone_depth_tolerance_ratio': camera_cone_depth_tolerance_ratio,
                'association_ambiguity_margin_m': association_ambiguity_margin_m,
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
                'use_camera_aruco_position': use_camera_aruco_position,
                'camera_cone_half_angle_deg': camera_cone_half_angle_deg,
                'camera_cone_depth_tolerance_m': camera_cone_depth_tolerance_m,
                'camera_cone_depth_tolerance_ratio': camera_cone_depth_tolerance_ratio,
                'association_ambiguity_margin_m': association_ambiguity_margin_m,
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
