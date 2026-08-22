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
    cross_id_ambiguity_margin_m = LaunchConfiguration('cross_id_ambiguity_margin_m')
    auto_recovery_enabled = LaunchConfiguration('auto_recovery_enabled')
    recovery_exit_disagreement_m = LaunchConfiguration('recovery_exit_disagreement_m')
    recovery_enter_hold_sec = LaunchConfiguration('recovery_enter_hold_sec')
    recovery_exit_hold_sec = LaunchConfiguration('recovery_exit_hold_sec')
    recovery_min_trigger_tags = LaunchConfiguration('recovery_min_trigger_tags')
    recovery_min_lidar_tags = LaunchConfiguration('recovery_min_lidar_tags')
    recovery_camera_observation_ttl_sec = LaunchConfiguration(
        'recovery_camera_observation_ttl_sec')
    recovery_lidar_success_ttl_sec = LaunchConfiguration(
        'recovery_lidar_success_ttl_sec')
    recovery_pose_solution_ttl_sec = LaunchConfiguration(
        'recovery_pose_solution_ttl_sec')
    recovery_min_observable_range_m = LaunchConfiguration(
        'recovery_min_observable_range_m')
    recovery_max_observable_range_m = LaunchConfiguration(
        'recovery_max_observable_range_m')
    recovery_forbidden_angle_min_deg = LaunchConfiguration(
        'recovery_forbidden_angle_min_deg')
    recovery_forbidden_angle_max_deg = LaunchConfiguration(
        'recovery_forbidden_angle_max_deg')
    phase2_yaw_gate_deg = LaunchConfiguration('phase2_yaw_gate_deg')
    phase2_backup_yaw_gate_deg = LaunchConfiguration(
        'phase2_backup_yaw_gate_deg')
    init_cube_timeout_sec = LaunchConfiguration('init_cube_timeout_sec')

    distance_threshold_inliers = LaunchConfiguration('distance_threshold_inliers')
    max_iterations = LaunchConfiguration('max_iterations')
    ransac_bounded_plane_enable = LaunchConfiguration('ransac_bounded_plane_enable')
    ransac_max_plane_short_extent_m = LaunchConfiguration('ransac_max_plane_short_extent_m')
    ransac_max_plane_long_extent_m = LaunchConfiguration('ransac_max_plane_long_extent_m')
    ransac_core_radius_margin = LaunchConfiguration('ransac_core_radius_margin')
    ransac_max_outside_fraction = LaunchConfiguration('ransac_max_outside_fraction')
    ransac_bbox_angle_step_deg = LaunchConfiguration('ransac_bbox_angle_step_deg')
    ransac_embedded_veto_enable = LaunchConfiguration('ransac_embedded_veto_enable')
    ransac_embedded_min_bin_points = LaunchConfiguration('ransac_embedded_min_bin_points')
    ransac_embedded_min_occupied_bins = LaunchConfiguration('ransac_embedded_min_occupied_bins')
    ransac_context_max_points = LaunchConfiguration('ransac_context_max_points')
    ransac_require_vertical_long_axis = LaunchConfiguration('ransac_require_vertical_long_axis')
    ransac_vertical_long_axis_max_deg = LaunchConfiguration('ransac_vertical_long_axis_max_deg')
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
        # Consumed by the pose-estimator Node below, therefore these must be
        # declared before that Node action is evaluated.
        DeclareLaunchArgument('phase2_yaw_gate_deg', default_value='20.0'),
        # ^ [deg] Phase-2 marker yaw may deviate this far from the camera-derived
        # Phase-1 yaw. Increase: accepts more heading error but can admit a cube
        # assigned to the wrong landmark. Decrease: rejects bad associations but
        # can drop valid cubes when the Phase-1 yaw is noisy.
        DeclareLaunchArgument(
            'phase2_backup_yaw_gate_deg', default_value='45.0'),
        # ^ [deg] Phase-2 gate when Phase 1 used the manual backup yaw.
        #   Increase: accepts less accurate manual alignment but can associate a
        #   cube to the wrong landmark. Decrease: is safer but rejects more
        #   legitimate cubes after imperfect manual alignment.
        DeclareLaunchArgument('init_cube_timeout_sec', default_value='30.0'),
        # ^ [s] if the cube pipeline never delivers enough usable detections,
        # Phase 2 completes anyway (partial samples, or the Phase-1 transform
        # unchanged) and hands map->odom to the KF. Increase: gives LiDAR more
        # time to produce a better seed but delays navigation. Decrease: hands
        # over sooner with less cube evidence. 0 disables the fallback and waits.

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
            parameters=[config, {
                'phase2_yaw_gate_deg': phase2_yaw_gate_deg,
                'phase2_backup_yaw_gate_deg': phase2_backup_yaw_gate_deg,
                'init_cube_timeout_sec': init_cube_timeout_sec,
            }],
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
                #   it replaces in pose_estimator_lidar_node. Increase: smoother
                #   TF updates but more CPU/network traffic. Decrease: lower load
                #   but a coarser transform stream.
                'meas_sigma_xy_m': 0.25,
                # ^ [m] FALLBACK only: /aruco_rover_pos now carries its own
                #   per-solution covariance (solution_sigma_xy_m in
                #   pose_estimator_lidar_node, scaled by the marker count), and
                #   that value is used whenever it is present. Tune the trust in
                #   a solver solution there, not here. If this fallback is used,
                #   increase it to trust a solution less; decrease it to trust it
                #   more and accept faster/noisier corrections.
                'meas_sigma_yaw_deg': 5.0,
                # ^ [deg] same fallback for heading: increase trusts a fallback
                #   yaw less; decrease corrects heading faster but more noisily.
                'process_sigma_xy_m_per_s': 0.02,
                # ^ [m/s] assumed odom drift rate. High: filter forgets fast,
                #   accepts big corrections. Low: filter is stiff, a real drift
                #   takes many measurements to correct.
                'process_sigma_yaw_deg_per_s': 0.5,
                # ^ [deg/s] same for yaw drift (mostly gyro bias).
                'mahalanobis_gate_chi2': 16.27,
                # ^ chi2 outlier gate on 3 DOF: 7.815 = 95%, 11.34 = 99%,
                #   16.27 = 99.9%. High: mislabeled cubes get in. Low:
                #   legitimate corrections are rejected.
                'max_consecutive_rejects': 10,
                # ^ after this many rejections in a row, P is inflated x4 so a
                #   mis-seeded filter can recover instead of rejecting forever;
                #   inflation is permitted only while the recovery supervisor
                #   has verified and enabled camera-guided recovery. Increase:
                #   waits longer before loosening the filter. Decrease: recovers
                #   sooner but becomes more permissive after a short bad streak.
                'inflate_only_in_camera_recovery': True,
                # ^ true permits covariance inflation only during verified
                #   CAMERA_RECOVERY; false permits it in all states. Boolean:
                #   there is no increase/decrease direction.
            }],
            output='screen',
        ),

        # ---- LiDAR crop: parameters shared by every recovery state -----------
        # lidar_phi_filter_node crops the raw cloud around the active ArUco
        # association.  The frame is map in MAP_GUIDED and base_link in the two
        # camera-guided states.
        DeclareLaunchArgument('hauteur_z_min', default_value='-0.4'),
        # ^ [m] z floor. Increase: removes more ground but can remove the cube
        #   bottom and reduce plane inliers. Decrease: preserves the cube bottom
        #   but admits more ground that can be fitted as a false face.
        DeclareLaunchArgument('hauteur_z_max', default_value='1.50'),
        # ^ [m] z ceiling. Increase: admits more overhead clutter. Decrease:
        #   rejects more clutter but can cut off the cube top (~0.32 m tall).
        DeclareLaunchArgument('association_ambiguity_margin_m', default_value='0.05'),
        # ^ [m] nearest-ID margin required by both crop paths. Increase: rejects
        #   more overlap/possible ID swaps, but drops valid close landmarks.
        #   Decrease: retains more points, but makes ID swaps more likely.

        # ---- MAP_GUIDED only -----------------------------------------------
        # The active association is the decoded ID's configured map landmark.
        DeclareLaunchArgument('tolerance_deg', default_value='5.0'),
        # ^ UNUSED legacy phi-sector width. Changing it has no runtime effect.
        DeclareLaunchArgument('tolerance_radius', default_value='1.0'),
        # ^ [m] landmark crop radius and REACQUIRE_MAP validation radius.
        #   Increase: tolerates more map->odom error, but admits nearby clutter
        #   and weakens the return-to-map test. Decrease: gives a cleaner crop
        #   and stricter validation, but can miss a displaced real cube.
        DeclareLaunchArgument('max_distance_from_aruco', default_value='1.0'),
        # ^ [m] pre-RANSAC radial band around the expected landmark range.
        #   Increase: admits more clutter. Decrease: rejects clutter but drops
        #   cube points when the expected range is inaccurate.
        DeclareLaunchArgument('angular_tolerance_deg', default_value='20.0'),
        # ^ [deg] pre-RANSAC bearing gate around the expected landmark.
        #   Increase: widens the sector and admits neighbours. Decrease: makes
        #   the crop cleaner but can miss a cube with bearing error.

        # ---- CAMERA_RECOVERY and REACQUIRE_MAP -----------------------------
        # Both states keep the camera-guided cone active. REACQUIRE_MAP uses it
        # while it tests whether map-guided filtering is safe again.
        DeclareLaunchArgument('use_camera_aruco_position', default_value='false'),
        # ^ Initial state only: false starts MAP_GUIDED; true starts
        #   CAMERA_RECOVERY. The recovery manager changes the mode at runtime.
        DeclareLaunchArgument('camera_cone_half_angle_deg', default_value='10.0'),
        # ^ [deg] cone half-angle around the camera bearing. Increase: wider
        #   lateral search, more recovery tolerance and clutter. Decrease:
        #   cleaner cone, but misses cubes when the camera bearing is noisy.
        DeclareLaunchArgument('camera_cone_depth_tolerance_m', default_value='0.6'),
        # ^ [m] minimum cone range tolerance. Increase: accepts larger camera
        #   depth error and more clutter. Decrease: improves rejection but can
        #   drop the cube when its depth estimate is biased.
        DeclareLaunchArgument('camera_cone_depth_tolerance_ratio', default_value='0.50'),
        # ^ dimensionless distance-scaled cone tolerance: max(fixed, ratio *
        #   camera range). Increase: cone grows faster at long range. Decrease:
        #   stronger long-range rejection, but less tolerance to depth error.

        # ---- Recovery supervisor: eligibility in every state ----------------
        DeclareLaunchArgument('cross_id_ambiguity_margin_m', default_value='0.10'),
        # ^ [m] evidence margin to quarantine a cross-ID observation. Increase:
        #   quarantine only decisive contradictions (fewer false quarantines).
        #   Decrease: quarantine more possible ID swaps (more valid IDs lost).
        DeclareLaunchArgument('auto_recovery_enabled', default_value='true'),
        # ^ true enables automatic state transitions; false keeps the initial
        #   mode. This is a boolean, so it has no increase/decrease direction.
        DeclareLaunchArgument(
            'recovery_camera_observation_ttl_sec', default_value='0.8'),
        # ^ [s] camera freshness in every state. Increase: tolerates lower camera
        #   rate but retains stale evidence longer. Decrease: reacts sooner to
        #   current data but can ignore valid low-rate camera observations.
        DeclareLaunchArgument(
            'recovery_lidar_success_ttl_sec', default_value='0.8'),
        # ^ [s] same-ID LiDAR success freshness. Increase: delays recovery after
        #   a cube disappears but tolerates lower LiDAR rate. Decrease: detects
        #   failure sooner but can trigger recovery between sparse batches.
        DeclareLaunchArgument(
            'recovery_pose_solution_ttl_sec', default_value='1.0'),
        # ^ [s] /aruco_rover_pos freshness for both recovery transitions.
        #   Increase: accepts an older solver result. Decrease: requires a more
        #   current solution, but can stall transitions at a low solver rate.
        DeclareLaunchArgument(
            'recovery_min_observable_range_m', default_value='0.5'),
        # ^ [m] nearest eligible camera range. Increase: ignores close detections
        #   (often noisy/occluded). Decrease: includes them, risking bad evidence.
        DeclareLaunchArgument(
            'recovery_max_observable_range_m', default_value='15.0'),
        # ^ [m] farthest eligible camera range. Increase: allows distant evidence
        #   (more range noise). Decrease: trusts only nearer, clearer detections.
        DeclareLaunchArgument(
            'recovery_forbidden_angle_min_deg', default_value='110.0'),
        # ^ [deg] lower edge of the excluded drill-obstructed sector. Increasing
        #   it narrows the sector; decreasing it widens the exclusion on this side.
        DeclareLaunchArgument(
            'recovery_forbidden_angle_max_deg', default_value='160.0'),
        # ^ [deg] upper edge of the excluded sector. Increasing it widens the
        #   exclusion; decreasing it narrows the sector on this side.

        # ---- MAP_GUIDED -> CAMERA_RECOVERY ---------------------------------
        DeclareLaunchArgument('recovery_min_trigger_tags', default_value='2'),
        # ^ independent failed IDs required to enter recovery. Increase: more
        #   robust to one bad ID but slower/harder to recover. Decrease: enters
        #   recovery sooner, but is more susceptible to a false trigger.
        DeclareLaunchArgument('recovery_enter_hold_sec', default_value='0.5'),
        # ^ [s] continuous failure evidence required before entering recovery.
        #   Increase: filters brief failures but delays recovery. Decrease:
        #   switches sooner but can react to short dropouts.

        # ---- CAMERA_RECOVERY -> REACQUIRE_MAP ------------------------------
        DeclareLaunchArgument('recovery_min_lidar_tags', default_value='2'),
        # ^ unique LiDAR IDs plus a fresh solver pose required before testing map
        #   guidance. Increase: stronger recovery proof but harder/slower exit.
        #   Decrease: exits recovery sooner with weaker evidence.

        # ---- REACQUIRE_MAP -> MAP_GUIDED -----------------------------------
        DeclareLaunchArgument('recovery_exit_disagreement_m', default_value='0.35'),
        # ^ [m] camera/map agreement diagnostic threshold during reacquisition.
        #   Increase: tolerates more disagreement; decrease: requires tighter
        #   agreement. Final return validation still uses tolerance_radius.
        DeclareLaunchArgument('recovery_exit_hold_sec', default_value='2.0'),
        # ^ [s] continuous LiDAR/solver/map-zone agreement before returning to
        #   MAP_GUIDED. Increase: safer, slower return. Decrease: faster return
        #   but more likely to bounce back into CAMERA_RECOVERY.

        # ---- Cube detector: applies in every recovery state -----------------
        DeclareLaunchArgument('distance_threshold_inliers', default_value='0.03'),
        # ^ [m] RANSAC plane inlier distance. Increase: rough/clutter points
        #   count as planes. Decrease: sharper planes only; sparse scans may fail.
        DeclareLaunchArgument('max_iterations', default_value='1000'),
        # ^ RANSAC iterations per plane. High: more consistent best-plane fit, more CPU per tag. Low: faster but can settle on a suboptimal plane
        DeclareLaunchArgument('ransac_bounded_plane_enable', default_value='true'),
        # ^ score RANSAC hypotheses by cube-face fit (trimmed footprint must fit the extent bounds) instead of raw inlier count. true: big ground/wall planes can never outbid the face. false: stock pcl::SACSegmentation, for A/B on the same scene
        DeclareLaunchArgument('ransac_max_plane_short_extent_m', default_value='0.0'),
        # ^ [m] max short side of an admissible hypothesis; 0 = derive as cube_width_m + face_dimension_tolerance_m. High: sprawling planes still win scoring. Low: partially-occluded faces still pass (this is an upper bound only), but noise-inflated ones get dropped
        DeclareLaunchArgument('ransac_max_plane_long_extent_m', default_value='0.0'),
        # ^ [m] max long side of an admissible hypothesis; 0 = derive as max(cube_width_m, cube_height_m) + face_dimension_tolerance_m. Same trade-off as above
        DeclareLaunchArgument('ransac_core_radius_margin', default_value='1.0'),
        # ^ x the face circumscribed radius: inliers further than this from the footprint median are treated as coplanar clutter (mainly the strip where the face plane, extended, cuts the ground) and excluded before measuring. High: that strip stretches the measured extents and the true face gets size-rejected. Low: legitimate face points get cut, extents under-report
        DeclareLaunchArgument('ransac_max_outside_fraction', default_value='0.35'),
        # ^ max share of a hypothesis's inliers allowed outside the core before it is rejected as not-a-face. High: ground/wall planes with a face-sized core still pass. Low: a face seen next to a lot of coplanar clutter gets rejected
        DeclareLaunchArgument('ransac_bbox_angle_step_deg', default_value='5.0'),
        # ^ [deg] orientation sweep step of the in-loop rectangle fit. High: cheaper, extents slightly over-reported. Low: more exact, more CPU per hypothesis
        DeclareLaunchArgument('ransac_embedded_veto_enable', default_value='true'),
        # ^ reject a face-sized candidate that is really a window onto a larger coplanar surface (a patch of wall/ground), by re-checking the plane against the ungated landmark neighbourhood. true: wall patches stop being accepted as faces. false: only the extent bounds apply, which the upstream crop can defeat
        DeclareLaunchArgument('ransac_embedded_min_bin_points', default_value='3'),
        # ^ coplanar points needed in a 45 deg bearing sector around the candidate before that sector counts as occupied. High: sparse surroundings ignored, embedded patches slip through. Low: LiDAR noise alone can occupy a sector and veto a real face
        DeclareLaunchArgument('ransac_context_max_points', default_value='700'),
        # ^ the veto's context cloud is strided down to this many points (0 = no cap). It is scanned inside the RANSAC loop, so this directly bounds CPU. High: more CPU per plane fit, can exceed the process_rate_hz budget. Low: sparse coverage, an embedded patch can look unsurrounded and slip through
        DeclareLaunchArgument('ransac_embedded_min_occupied_bins', default_value='5'),
        # ^ occupied sectors (of 8) at which the candidate is called embedded. A cube face standing on the ground has its plane clip the ground in a line, occupying ~2 opposite sectors, so this must stay above that. High: only fully surrounded patches vetoed. Low: the ground-intersection strip alone can veto a real face
        DeclareLaunchArgument('ransac_require_vertical_long_axis', default_value='true'),
        # ^ require a side-face candidate's LONG edge to run with gravity, as a cube standing on the ground does. Kills tilted slivers that happen to measure face-sized. Near-horizontal planes (top-face candidates) are exempt, since a top face's long axis is horizontal. NOTE: a side face seen below ~78% of its height measures wider than tall, so its long axis is horizontal and it gets rejected -- turn this off if you rely on heavily-cropped partial faces
        DeclareLaunchArgument('ransac_vertical_long_axis_max_deg', default_value='20.0'),
        # ^ [deg] half-angle of the cone about +/-z that the long edge must fall inside; for a side face this is the rectangle's roll within its own plane, and it does not constrain the face's heading. High: tilted clutter passes again. Low: strict, but a tilted cube or a LiDAR mounting tilt (this uses the cloud z, like top_vertical_dot_min) will reject real faces
        DeclareLaunchArgument('t', default_value='0.25'),
        # ^ UNUSED: legacy face width [m] of the old 2D-line detector; the plane path uses cube_width_m instead
        DeclareLaunchArgument('min_inliers', default_value='10'),
        # ^ min points per plane (a range-based dynamic floor also applies). High: only dense/close detections, far cube missed. Low: accepts sparse planes -> more false positives
        DeclareLaunchArgument('max_lines', default_value='3'),
        # ^ UNUSED: legacy limit of the old 2D-line detector
        DeclareLaunchArgument('max_planes', default_value='5'),
        # ^ RANSAC plane extractions per tag. High: digs past ground/walls to find the cube faces (more CPU). Low: only dominant planes, cube missed if clutter is fit first
        DeclareLaunchArgument('plane_group_centroid_max_m', default_value='0.80'),
        # ^ [m] max centroid distance to group a 2nd side face / top with the 1st. High: planes from different objects merged into one cube. Low: valid 2nd face rejected -> center estimated from a single face (less accurate)
        DeclareLaunchArgument('face_dimension_tolerance_m', default_value='0.1'),
        # ^ [m] allowed excess over nominal face dims (also scales the accept score). High: oversized/odd planes accepted as faces. Low: strict size match, partial or noisy faces rejected
        DeclareLaunchArgument('side_perpendicular_dot_max', default_value='0.75'),
        # ^ max |dot| between two side-face normals to pair them (0 = strictly perpendicular). High: near-parallel planes can pair as "two faces". Low: true face pairs dropped when normals are noisy
        DeclareLaunchArgument('top_vertical_dot_min', default_value='0.50'), #0.35
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
        DeclareLaunchArgument('accept_best_plane_fallback', default_value='true'),
        # ^ if no plausible face found, take the biggest extracted plane as a side. true: almost always outputs a detection (clutter can become the "cube"). false: silent unless a real face matched
        DeclareLaunchArgument('process_rate_hz', default_value='13.0'),
        # ^ processing throttle. High: fresher detections, more CPU. Low: saves CPU, laggier detections
        DeclareLaunchArgument('marker_lifetime_sec', default_value='1.5'), #1.0 second is too little to exit the recovery mode : REACQUIRE MAP
        # ^ [s] rviz marker persistence. High: markers linger after the cube moved (ghosting). Low: markers flicker between detections

        # ---- Recovery state machine -------------------------------------------
        # Eligible camera ID:
        #   fresh camera observation, range inside [min,max], outside forbidden
        #   sector, and not quarantined by the cross-ID classifier.
        # Failure:
        #   eligible camera ID with no same-ID /cube_markers success inside
        #   recovery_lidar_success_ttl_sec.
        #
        # MAP_GUIDED -> CAMERA_RECOVERY:
        #   failures >= recovery_min_trigger_tags continuously for
        #   recovery_enter_hold_sec.
        #
        # CAMERA_RECOVERY -> REACQUIRE_MAP:
        #   a fresh post-entry /cube_markers batch has at least
        #   recovery_min_lidar_tags unique IDs, every center transformed to
        #   odom, AND a fresh post-entry /aruco_rover_pos exists. No pose means
        #   the solver has not validated a recovery, so this transition blocks.
        #
        # REACQUIRE_MAP -> MAP_GUIDED (camera cones remain active while testing):
        #   fresh post-entry LiDAR batch + fresh post-entry /aruco_rover_pos,
        #   at least recovery_min_lidar_tags, every LiDAR center inside
        #   tolerance_radius of the configured landmark with the same ID,
        #   continuously for recovery_exit_hold_sec.
        # REACQUIRE_MAP -> CAMERA_RECOVERY:
        #   failures >= recovery_min_trigger_tags again.
        Node(
            package='ros2_aruco_with_lidar',
            executable='aruco_lidar_recovery_manager',
            name='aruco_lidar_recovery_manager',
            output='screen',
            parameters=[config, {
                'auto_recovery_enabled': auto_recovery_enabled,
                'use_camera_aruco_position': use_camera_aruco_position,
                # Use the exact map-landmark crop radius for cross-ID zone
                # classification and REACQUIRE_MAP validation.
                'enter_disagreement_m': tolerance_radius,
                'reacquire_landmark_radius_m': tolerance_radius,
                'exit_disagreement_m': recovery_exit_disagreement_m,
                'enter_hold_sec': recovery_enter_hold_sec,
                'exit_hold_sec': recovery_exit_hold_sec,
                'min_trigger_tags': recovery_min_trigger_tags,
                'min_recovery_lidar_tags': recovery_min_lidar_tags,
                'cross_id_ambiguity_margin_m': cross_id_ambiguity_margin_m,
                'camera_observation_ttl_sec': recovery_camera_observation_ttl_sec,
                'lidar_success_ttl_sec': recovery_lidar_success_ttl_sec,
                'pose_solution_ttl_sec': recovery_pose_solution_ttl_sec,
                'min_observable_range_m': recovery_min_observable_range_m,
                'max_observable_range_m': recovery_max_observable_range_m,
                'forbidden_angle_min_deg': recovery_forbidden_angle_min_deg,
                'forbidden_angle_max_deg': recovery_forbidden_angle_max_deg,
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
                'ransac_bounded_plane_enable': ransac_bounded_plane_enable,
                'ransac_max_plane_short_extent_m': ransac_max_plane_short_extent_m,
                'ransac_max_plane_long_extent_m': ransac_max_plane_long_extent_m,
                'ransac_core_radius_margin': ransac_core_radius_margin,
                'ransac_max_outside_fraction': ransac_max_outside_fraction,
                'ransac_bbox_angle_step_deg': ransac_bbox_angle_step_deg,
                'ransac_embedded_veto_enable': ransac_embedded_veto_enable,
                'ransac_embedded_min_bin_points': ransac_embedded_min_bin_points,
                'ransac_embedded_min_occupied_bins': ransac_embedded_min_occupied_bins,
                'ransac_context_max_points': ransac_context_max_points,
                'ransac_require_vertical_long_axis': ransac_require_vertical_long_axis,
                'ransac_vertical_long_axis_max_deg': ransac_vertical_long_axis_max_deg,
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
