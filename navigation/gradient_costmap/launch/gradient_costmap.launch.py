from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def lc(name):
    return LaunchConfiguration(name)


def arg(name, default, description=""):
    return DeclareLaunchArgument(
        name,
        default_value=str(default),
        description=description,
    )


def mode_if(global_value, local_value):
    """
    Return global_value if gradient_mode == global, else local_value.

    global_value and local_value can be strings or LaunchConfiguration objects.
    """
    return PythonExpression([
        '"',
        global_value,
        '" if "',
        lc("gradient_mode"),
        '" == "global" else "',
        local_value,
        '"',
    ])


def typed(name, value_type):
    return ParameterValue(lc(name), value_type=value_type)


def mode_typed(global_name, local_name, value_type):
    return ParameterValue(
        mode_if(lc(global_name), lc(local_name)),
        value_type=value_type,
    )


def generate_launch_description():
    declarations = [
        arg("use_sim_time", "false"),

        arg(
            "gradient_mode",
            "local",
            "Gradient map mode: local uses base_link-centered local map; "
            "global uses map-frame fixed origin and /cloud_pcd.",
        ),

        arg(
            "lidar_points_topic",
            "/ouster/points",
            "Input point cloud topic for traversability_filter.",
        ),
        arg(
            "filtered_cloud_topic",
            "/filtered_pointcloud",
            "Output of traversability_filter and input of traversability_map.",
        ),
        arg(
            "output_elevation_topic",
            "/elevation_pointcloud",
            "Output elevation pointcloud topic.",
        ),

        arg(
            "source_frame",
            "Lidar_v2_1",
            "Fallback lidar frame if message frame_id is missing.",
        ),
        arg(
            "use_msg_frame_id",
            "true",
            "Use incoming pointcloud frame_id as source frame in filter transform.",
        ),

        arg(
            "fixed_origin_x",
            "-40.0",
            "Fixed origin x used in global mode.",
        ),
        arg(
            "fixed_origin_y",
            "-40.0",
            "Fixed origin y used in global mode.",
        ),

        arg(
            "filter_point_stride",
            "3",
            "traversability_filter: process every k-th point after optional voxel downsampling.",
        ),
        arg(
            "filter_sensor_voxel_leaf_m",
            "0.1",
            "traversability_filter: voxel grid leaf size in meters in the incoming lidar frame. "
            "0.0 disables voxel downsampling.",
        ),
        arg(
            "filter_max_lidar_z_m",
            "1.0",
            "traversability_filter: discard points above this z value in LiDAR frame.",
        ),

        arg(
            "local_inflation_radius",
            "2",
            "Inflation radius in cells used when gradient_mode=local.",
        ),
        arg(
            "local_inflation_factor",
            "6.0",
            "Inflation decay factor used when gradient_mode=local.",
        ),
        arg(
            "local_sigmoid_k",
            "4.0",
            "Sigmoid slope parameter used when gradient_mode=local.",
        ),
        arg(
            "local_sigmoid_x0",
            "0.95",
            "Sigmoid midpoint parameter used when gradient_mode=local.",
        ),

        arg(
            "global_inflation_radius",
            "1",
            "Inflation radius in cells used when gradient_mode=global.",
        ),
        arg(
            "global_inflation_factor",
            "1.0",
            "Inflation decay factor used when gradient_mode=global.",
        ),
        arg(
            "global_sigmoid_k",
            "1.0",
            "Sigmoid slope parameter used when gradient_mode=global.",
        ),
        arg(
            "global_sigmoid_x0",
            "0.60",
            "Sigmoid midpoint parameter used when gradient_mode=global.",
        ),

        arg(
            "neighbor_search_radius_m",
            "1.2",
            "Neighborhood radius in meters for plane fit per cell.",
        ),
        arg(
            "slope_angle_limit_deg",
            "35.0",
            "Slope angle in degrees at which slope cost saturates.",
        ),
        arg(
            "roughness_norm_m",
            "0.1",
            "Roughness normalization in meters.",
        ),
        arg(
            "min_neighbor_points",
            "3",
            "Minimum neighbor samples for traversability PCA.",
        ),
    ]

    map_frame = mode_if("map", "base_link")
    map_input_topic = mode_if("/cloud_pcd", lc("filtered_cloud_topic"))

    output_local_topic = mode_if(
        "/occupancy_map_global",
        "/occupancy_map_local",
    )
    output_local_inflated_topic = mode_if(
        "/occupancy_map_global_inflated",
        "/occupancy_map_local_inflated",
    )

    use_robot_centered_origin = mode_if("false", "true")

    filter_params = {
        "use_sim_time": lc("use_sim_time"),
        "input_cloud_topic": lc("lidar_points_topic"),
        "output_cloud_topic": lc("filtered_cloud_topic"),
        "map_frame": map_frame,
        "base_frame": "base_link",
        "source_frame": lc("source_frame"),
        "use_msg_frame_id": lc("use_msg_frame_id"),
        "point_stride": typed("filter_point_stride", int),
        "sensor_voxel_leaf_m": typed("filter_sensor_voxel_leaf_m", float),
        "max_lidar_z_m": typed("filter_max_lidar_z_m", float),
    }

    map_params = {
        "use_sim_time": lc("use_sim_time"),
        "pointcloud_topic": map_input_topic,
        "output_local_topic": output_local_topic,
        "output_local_inflated_topic": output_local_inflated_topic,
        "output_elevation_topic": lc("output_elevation_topic"),
        "map_frame": map_frame,
        "base_frame": "base_link",
        "use_robot_centered_origin": use_robot_centered_origin,
        "fixed_origin_x": lc("fixed_origin_x"),
        "fixed_origin_y": lc("fixed_origin_y"),

        "inflation_radius": mode_typed(
            "global_inflation_radius",
            "local_inflation_radius",
            int,
        ),
        "inflation_factor": mode_typed(
            "global_inflation_factor",
            "local_inflation_factor",
            float,
        ),
        "sigmoid_k": mode_typed(
            "global_sigmoid_k",
            "local_sigmoid_k",
            float,
        ),
        "sigmoid_x0": mode_typed(
            "global_sigmoid_x0",
            "local_sigmoid_x0",
            float,
        ),

        "neighbor_search_radius_m": typed("neighbor_search_radius_m", float),
        "slope_angle_limit_deg": typed("slope_angle_limit_deg", float),
        "roughness_norm_m": typed("roughness_norm_m", float),
        "min_neighbor_points": typed("min_neighbor_points", int),
    }

    traversability_filter = Node(
        package="gradient_costmap",
        executable="traversability_filter",
        name="traversability_filter",
        output="screen",
        parameters=[filter_params],
    )

    traversability_map = Node(
        package="gradient_costmap",
        executable="traversability_map",
        name="traversability_map",
        output="screen",
        parameters=[map_params],
    )

    return LaunchDescription([
        *declarations,
        traversability_filter,
        traversability_map,
    ])