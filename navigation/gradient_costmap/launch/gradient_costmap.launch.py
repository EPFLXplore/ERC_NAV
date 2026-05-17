from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gradient_mode = LaunchConfiguration("gradient_mode")
    lidar_points_topic = LaunchConfiguration("lidar_points_topic")
    filtered_cloud_topic = LaunchConfiguration("filtered_cloud_topic")

    map_input_topic = PythonExpression(
        [
            '"/cloud_pcd" if "',
            gradient_mode,
            '" == "global" else "',
            filtered_cloud_topic,
            '"',
        ]
    )
    output_local_topic = PythonExpression(
        [
            '"/occupancy_map_global" if "',
            gradient_mode,
            '" == "global" else "/occupancy_map_local"',
        ]
    )
    output_local_inflated_topic = PythonExpression(
        [
            '"/occupancy_map_global_inflated" if "',
            gradient_mode,
            '" == "global" else "/occupancy_map_local_inflated"',
        ]
    )
    output_elevation_topic = LaunchConfiguration("output_elevation_topic")
    map_frame = PythonExpression(
        ['"map" if "', gradient_mode, '" == "global" else "base_link"']
    )
    base_frame = PythonExpression(['"base_link"'])
    source_frame = LaunchConfiguration("source_frame")
    use_robot_centered_origin = PythonExpression(
        ['"true" if "', gradient_mode, '" == "local" else "false"']
    )
    fixed_origin_x = LaunchConfiguration("fixed_origin_x")
    fixed_origin_y = LaunchConfiguration("fixed_origin_y")
    use_msg_frame_id = LaunchConfiguration("use_msg_frame_id")
    filter_point_stride = LaunchConfiguration("filter_point_stride")
    filter_sensor_voxel_leaf_m = LaunchConfiguration("filter_sensor_voxel_leaf_m")
    filter_max_lidar_z_m = LaunchConfiguration("filter_max_lidar_z_m")
    inflation_radius = PythonExpression(
        [
            '"',
            LaunchConfiguration("global_inflation_radius"),
            '" if "',
            gradient_mode,
            '" == "global" else "',
            LaunchConfiguration("local_inflation_radius"),
            '"',
        ]
    )
    inflation_factor = PythonExpression(
        [
            '"',
            LaunchConfiguration("global_inflation_factor"),
            '" if "',
            gradient_mode,
            '" == "global" else "',
            LaunchConfiguration("local_inflation_factor"),
            '"',
        ]
    )
    sigmoid_k = PythonExpression(
        [
            '"',
            LaunchConfiguration("global_sigmoid_k"),
            '" if "',
            gradient_mode,
            '" == "global" else "',
            LaunchConfiguration("local_sigmoid_k"),
            '"',
        ]
    )
    sigmoid_x0 = PythonExpression(
        [
            '"',
            LaunchConfiguration("global_sigmoid_x0"),
            '" if "',
            gradient_mode,
            '" == "global" else "',
            LaunchConfiguration("local_sigmoid_x0"),
            '"',
        ]
    )
    # Traversability PCA / slope (traversability_map) — same for local and global mode
    neighbor_search_radius_m = LaunchConfiguration("neighbor_search_radius_m")
    slope_angle_limit_deg = LaunchConfiguration("slope_angle_limit_deg")
    roughness_norm_m = LaunchConfiguration("roughness_norm_m")
    min_neighbor_points = LaunchConfiguration("min_neighbor_points")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_gradient_mode = DeclareLaunchArgument(
        "gradient_mode",
        ### CHANGE LOCAL TO GLOBAL HEEEEEEEEEERE GRADIENT MODE ###
        # Should be in local for autonomous navigation, the stack automatically takes the inflated_global_map you already computed.
        # if you want to compute the global inflated map, change this gradient_mode to global, put your global pointcloud in saved_map/2026 and publish the point cloud on the topic /cloud_pcd
        # then register the inflated map still in saved_map/2026 
        default_value="local",
        description="Gradient map mode: local (base_link local map) or global (map-frame fixed origin)",
    )
    declare_lidar_points_topic = DeclareLaunchArgument(
        "lidar_points_topic",
        default_value="/ouster/points",
        description="Input point cloud topic for traversability_filter",
    )
    declare_filtered_cloud_topic = DeclareLaunchArgument(
        "filtered_cloud_topic",
        default_value="/filtered_pointcloud",
        description="Output of traversability_filter and input of traversability_map",
    )
    declare_output_elevation_topic = DeclareLaunchArgument(
        "output_elevation_topic",
        default_value="/elevation_pointcloud",
        description="Output elevation pointcloud topic",
    )
    declare_source_frame = DeclareLaunchArgument(
        "source_frame",
        default_value="Lidar_v2_1",
        description="Fallback lidar frame if message frame_id is missing",
    )
    declare_fixed_origin_x = DeclareLaunchArgument(
        "fixed_origin_x",
        default_value="-40.0",
        description="Fixed origin x used in global mode",
    )
    declare_fixed_origin_y = DeclareLaunchArgument(
        "fixed_origin_y",
        default_value="-40.0",
        description="Fixed origin y used in global mode",
    )
    declare_use_msg_frame_id = DeclareLaunchArgument(
        "use_msg_frame_id",
        default_value="true",
        description="Use incoming pointcloud frame_id as source frame in filter transform",
    )
    # --- traversability_filter CPU / downsampling (see comment on Node below) ---
    declare_filter_point_stride = DeclareLaunchArgument(
        "filter_point_stride",
        default_value="3",
        description=(
            "Only used by traversability_filter. After optional voxel downsampling, process "
            "every k-th point in the cloud (k>=1). 1 = no stride. 2 or 3 cuts transform + height-bin "
            "work with almost no extra code cost; very large k can miss thin obstacles. Applied in "
            "sensor frame order (point index), not spatially uniform."
        ),
    )
    declare_filter_sensor_voxel_leaf_m = DeclareLaunchArgument(
        "filter_sensor_voxel_leaf_m",
        default_value="0.1",
        description=(
            "Only used by traversability_filter. PCL voxel grid leaf size in meters in the "
            "incoming lidar frame, before TF to map and before the 2D height map. 0.0 disables "
            "voxel downsampling. Non-zero merges nearby returns into one point per voxel (typ. "
            "0.15-0.2 m when map resolution is 0.1 m) to reduce CPU; smaller leaf preserves detail."
        ),
    )
    declare_filter_max_lidar_z_m = DeclareLaunchArgument(
        "filter_max_lidar_z_m",
        default_value="1.0",
        description=(
            "Only used by traversability_filter. Maximum accepted point z in LiDAR frame "
            "(meters). Points above this are discarded before voxelization and mapping."
        ),
    )

    declare_local_inflation_radius = DeclareLaunchArgument(
        "local_inflation_radius",
        default_value="2",
        description="Inflation radius (cells) used when gradient_mode=local",
    )
    declare_local_inflation_factor = DeclareLaunchArgument(
        "local_inflation_factor",
        default_value="6.0",
        description="Inflation decay factor used when gradient_mode=local",
    )
    declare_local_sigmoid_k = DeclareLaunchArgument(
        "local_sigmoid_k",
        default_value="4.0",
        description="Sigmoid slope parameter used when gradient_mode=local",
    )
    declare_local_sigmoid_x0 = DeclareLaunchArgument(
        "local_sigmoid_x0",
        default_value="0.95",
        description="Sigmoid midpoint parameter used when gradient_mode=local",
    )

    declare_global_inflation_radius = DeclareLaunchArgument(
        "global_inflation_radius",
        default_value="1",
        description="Inflation radius (cells) used when gradient_mode=global",
    )
    declare_global_inflation_factor = DeclareLaunchArgument(
        "global_inflation_factor",
        default_value="1.0",
        description="Inflation decay factor used when gradient_mode=global",
    )
    declare_global_sigmoid_k = DeclareLaunchArgument(
        "global_sigmoid_k",
        default_value="1.0",
        description="Sigmoid slope parameter used when gradient_mode=global",
    )
    declare_global_sigmoid_x0 = DeclareLaunchArgument(
        "global_sigmoid_x0",
        default_value="0.60",
        description="Sigmoid midpoint parameter used when gradient_mode=global",
    )
    # --- traversability_map: slope / roughness (feed Nav2 local costmap via /occupancy_map_local_inflated) ---
    declare_neighbor_search_radius_m = DeclareLaunchArgument(
        "neighbor_search_radius_m",
        default_value="1.2",
        description=(
            "Meters: neighborhood radius for plane fit per cell. Larger values smooth noise near lidar min-range "
            "(often reduces spurious high cost around the robot). Typical 0.5-1.0."
        ),
    )
    declare_slope_angle_limit_deg = DeclareLaunchArgument(
        "slope_angle_limit_deg",
        default_value="35.0",
        description="Degrees at which slope cost saturates (same role as utility.h filterAngleLimit).",
    )
    declare_roughness_norm_m = DeclareLaunchArgument(
        "roughness_norm_m",
        default_value="0.1",
        description="Meters: roughness normalization (same role as utility.h filterMaxRoughness).",
    )
    declare_min_neighbor_points = DeclareLaunchArgument(
        "min_neighbor_points",
        default_value="3",
        description="Minimum neighbor samples for traversability PCA (≥3).",
    )

    # traversability_filter: raw cloud -> (optional voxel) -> (optional stride) -> range clip ->
    # transform to map -> bin to local height grid -> publish /filtered_pointcloud.
    # filter_sensor_voxel_leaf_m: spatial merge in lidar frame (0 = off). filter_point_stride:
    # use every k-th point after that. Example: voxel 0.15 + stride 1, or voxel 0 + stride 2.
    traversability_filter = Node(
        package="gradient_costmap",
        executable="traversability_filter",
        name="traversability_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_cloud_topic": lidar_points_topic,
                "output_cloud_topic": filtered_cloud_topic,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "source_frame": source_frame,
                "use_msg_frame_id": use_msg_frame_id,
                "point_stride": ParameterValue(filter_point_stride, value_type=int),
                "sensor_voxel_leaf_m": ParameterValue(
                    filter_sensor_voxel_leaf_m, value_type=float
                ),
                "max_lidar_z_m": ParameterValue(filter_max_lidar_z_m, value_type=float),
            }
        ],
    )

    # TODO: remove this dangerous shit 
    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "map"],
    #     condition=IfCondition(PythonExpression(['"', gradient_mode, '" == "global"'])),
    # )

    traversability_map = Node(
        package="gradient_costmap",
        executable="traversability_map",
        name="traversability_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pointcloud_topic": map_input_topic,
                "output_local_topic": output_local_topic,
                "output_local_inflated_topic": output_local_inflated_topic,
                "output_elevation_topic": output_elevation_topic,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "use_robot_centered_origin": use_robot_centered_origin,
                "fixed_origin_x": fixed_origin_x,
                "fixed_origin_y": fixed_origin_y,
                "inflation_radius": ParameterValue(inflation_radius, value_type=int),
                "inflation_factor": ParameterValue(inflation_factor, value_type=float),
                "sigmoid_k": ParameterValue(sigmoid_k, value_type=float),
                "sigmoid_x0": ParameterValue(sigmoid_x0, value_type=float),
                "neighbor_search_radius_m": ParameterValue(neighbor_search_radius_m, value_type=float),
                "slope_angle_limit_deg": ParameterValue(slope_angle_limit_deg, value_type=float),
                "roughness_norm_m": ParameterValue(roughness_norm_m, value_type=float),
                "min_neighbor_points": ParameterValue(min_neighbor_points, value_type=int),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_gradient_mode,
            declare_lidar_points_topic,
            declare_filtered_cloud_topic,
            declare_output_elevation_topic,
            declare_source_frame,
            declare_fixed_origin_x,
            declare_fixed_origin_y,
            declare_use_msg_frame_id,
            declare_filter_point_stride,
            declare_filter_sensor_voxel_leaf_m,
            declare_filter_max_lidar_z_m,
            declare_local_inflation_radius,
            declare_global_inflation_radius,
            declare_local_inflation_factor,
            declare_global_inflation_factor,
            declare_local_sigmoid_k,
            declare_global_sigmoid_k,
            declare_local_sigmoid_x0,
            declare_global_sigmoid_x0,
            declare_neighbor_search_radius_m,
            declare_slope_angle_limit_deg,
            declare_roughness_norm_m,
            declare_min_neighbor_points,
            traversability_filter,
            # static_transform_publisher,
            traversability_map,
        ]
    )
