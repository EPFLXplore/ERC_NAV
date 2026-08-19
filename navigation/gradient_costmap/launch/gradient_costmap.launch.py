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
            "global uses map-frame fixed origin and /cloud_pcd. Use local for live robot-relative "
            "mapping; use global for fixed/offline map generation.",
        ),

        arg(
            "lidar_points_topic",
            "/ouster/points",
            "Input point cloud topic for traversability_filter. Change this to select which raw LiDAR "
            "cloud is filtered; it does not change map aggressiveness.",
        ),
        arg(
            "filtered_cloud_topic",
            "/filtered_pointcloud",
            "Output of traversability_filter and input of traversability_map. Change this only when "
            "remapping the intermediate filtered cloud topic.",
        ),
        arg(
            "output_elevation_topic",
            "/elevation_pointcloud",
            "Output elevation pointcloud topic. Change this to rename the visualization/debug cloud; "
            "it does not change map computation.",
        ),

        arg(
            "source_frame",
            "Lidar_v2_1",
            "Fallback lidar frame if message frame_id is missing. Wrong values cause TF lookup "
            "failures or shifted maps; no higher/lower tuning effect.",
        ),
        arg(
            "use_msg_frame_id",
            "true",
            "Use incoming pointcloud frame_id as source frame in filter transform. true trusts the "
            "message header; false always uses source_frame.",
        ),

        arg(
            "fixed_origin_x",
            "-40.0",
            "Fixed origin x used in global mode. Higher shifts the published grid origin toward +x; "
            "lower shifts it toward -x.",
        ),
        arg(
            "fixed_origin_y",
            "-40.0",
            "Fixed origin y used in global mode. Higher shifts the published grid origin toward +y; "
            "lower shifts it toward -y.",
        ),

        arg(
            "filter_point_stride",
            "1",
            "traversability_filter: process every k-th point after optional voxel downsampling. "
            "Higher is faster but less detailed; lower keeps more points but costs more CPU.",
        ),
        arg(
            "filter_sensor_voxel_leaf_m",
            "0.05",
            "traversability_filter: voxel grid leaf size in meters in the incoming lidar frame. "
            "Higher downsamples more and smooths detail; lower preserves detail but costs more CPU. "
            "0.0 disables voxel downsampling.",
        ),
        arg(
            "filter_noise_radius_m", 
            "0.10", # must be bigger than the size of voxel
            "traversability_filter: PCL RadiusOutlierRemoval radius in meters AFTER optional voxel downsampling. "
            "0.0 disables radius noise filtering; higher removes more isolated sparse returns.",
        ),
        arg(
            "filter_noise_min_neighbors",
            "8",
            "traversability_filter: minimum neighbors inside filter_noise_radius_m required to keep a point. "
            "Higher removes more isolated points but can erase sparse valid terrain.",
        ),
        arg(
            "filter_max_lidar_z_m",
            "0.5",
            "traversability_filter: discard points above this z value in LiDAR frame. Higher keeps "
            "taller points/obstacles; lower removes more high returns.",
        ),

        arg(
            "local_inflation_radius",
            "2",
            "Inflation radius in cells used when gradient_mode=local. Higher expands obstacles/costs "
            "farther; lower keeps inflated costs tighter.",
        ),
        arg(
            "local_inflation_factor",
            "0.0001",
            "Inflation decay factor used when gradient_mode=local. Higher makes inflated cost decay "
            "faster with distance; lower spreads stronger costs farther.",
        ),
        arg(
            "local_sigmoid_k",
            "0.5", #4.0
            "Sigmoid slope parameter used when gradient_mode=local. Higher makes the cost transition "
            "sharper around x0; lower makes traversability costs more gradual.",
        ),
        arg(
            "local_sigmoid_x0",
            "0.3", #0.95
            "Sigmoid midpoint parameter used when gradient_mode=local. Higher delays high occupancy "
            "until worse terrain; lower marks terrain costly sooner.",
        ),
        arg(
            "output_lookup_radius_cells",
            "0",
            "traversability_map: observed-cell lookup radius in grid cells when publishing output maps. "
            "Higher fills small observation/grid gaps more aggressively; lower leaves more unknown cells.",
        ),
        arg(
            "grid_size_m",
            "10.0",
            "traversability_map: dense map grid side length in meters. Higher covers a larger stored area "
            "but uses more memory; lower is lighter but clips points outside the grid.",
        ),
        arg(
            "grid_resolution_m",
            "0.1",
            "traversability_map: dense map grid resolution in meters/cell. Lower is finer but more CPU/memory; "
            "higher is coarser and smoother.",
        ),
        arg(
            "lidar_dead_zone_enabled",
            "true",
            "traversability_map: mark the LiDAR blind/dead angular sector as max cost in output maps.",
        ),
        arg(
            "lidar_dead_zone_min_angle_deg",
            "-70.0",
            "traversability_map: minimum dead-zone bearing in degrees in the LiDAR/source-frame XY plane.",
        ),
        arg(
            "lidar_dead_zone_max_angle_deg",
            "-20.0",
            "traversability_map: maximum dead-zone bearing in degrees in the LiDAR/source-frame XY plane.",
        ),
        arg(
            "lidar_dead_zone_radius_m",
            "10.0",
            "traversability_map: radial extent in meters for the LiDAR dead-zone max-cost sector.",
        ),
        arg(
            "global_inflation_radius",
            "1",
            "Inflation radius in cells used when gradient_mode=global. Higher expands obstacles/costs "
            "farther; lower keeps inflated costs tighter.",
        ),
        arg(
            "global_inflation_factor",
            "1.0",
            "Inflation decay factor used when gradient_mode=global. Higher makes inflated cost decay "
            "faster with distance; lower spreads stronger costs farther.",
        ),
        arg(
            "global_sigmoid_k",
            "1.0",
            "Sigmoid slope parameter used when gradient_mode=global. Higher makes the cost transition "
            "sharper around x0; lower makes traversability costs more gradual.",
        ),
        arg(
            "global_sigmoid_x0",
            "0.60",
            "Sigmoid midpoint parameter used when gradient_mode=global. Higher delays high occupancy "
            "until worse terrain; lower marks terrain costly sooner.",
        ),

        arg(
            "neighbor_search_radius_m",
            "0.25",
            "Neighborhood radius in meters for plane fit per cell. Higher smooths over a larger area "
            "and is more stable but less local; lower reacts to small terrain changes but is noisier.",
        ),
        arg(
            "slope_angle_limit_deg",
            "60.0",
            "Slope angle in degrees at which slope cost saturates. Higher tolerates steeper slopes "
            "before max cost; lower penalizes slopes earlier.",
        ),
        arg(
            "roughness_norm_m",
            "1.0",
            "Roughness normalization in meters. Higher tolerates rougher terrain before max cost; "
            "lower penalizes small height variation sooner.",
        ),
        arg(
            "min_neighbor_points",
            "8",
            "Minimum neighbor samples for traversability PCA. Higher requires more support and rejects "
            "sparse/noisy cells; lower computes costs with fewer points but is less reliable.",
        ),
        arg(
            "min_neighbor_quadrants",
            "4",
            "Minimum number of XY quadrants around a cell that must contain observed neighbors before "
            "fitting traversability. Higher rejects one-sided edge fits; lower accepts sparser geometry.",
        ),
    ]

    map_frame = mode_if("map", "base_link")
    map_input_topic = mode_if("/cloud_pcd", lc("filtered_cloud_topic")) # filtered_cloud_topic

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
        "noise_radius_m": typed("filter_noise_radius_m", float),
        "noise_min_neighbors": typed("filter_noise_min_neighbors", int),
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
        "output_lookup_radius_cells": typed("output_lookup_radius_cells", int),
        "grid_size_m": typed("grid_size_m", float),
        "grid_resolution_m": typed("grid_resolution_m", float),

        "neighbor_search_radius_m": typed("neighbor_search_radius_m", float),
        "slope_angle_limit_deg": typed("slope_angle_limit_deg", float),
        "roughness_norm_m": typed("roughness_norm_m", float),
        "min_neighbor_points": typed("min_neighbor_points", int),
        "min_neighbor_quadrants": typed("min_neighbor_quadrants", int),
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
