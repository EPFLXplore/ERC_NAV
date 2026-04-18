from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_gradient_mode = DeclareLaunchArgument(
        "gradient_mode",
        ### CHANGE LOCAL TO GLOBAL HEEEEEEEEEERE GRADIENT MODE ###
        default_value="local",
        description="Gradient map mode: local (base_link local map) or global (map-frame fixed origin)",
    )
    declare_lidar_points_topic = DeclareLaunchArgument(
        "lidar_points_topic",
        default_value="/ouster_points",
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
        default_value="ST_Lidar_1",
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

    declare_local_inflation_radius = DeclareLaunchArgument(
        "local_inflation_radius",
        default_value="1",
        description="Inflation radius (cells) used when gradient_mode=local",
    )
    declare_local_inflation_factor = DeclareLaunchArgument(
        "local_inflation_factor",
        default_value="10.0",
        description="Inflation decay factor used when gradient_mode=local",
    )
    declare_local_sigmoid_k = DeclareLaunchArgument(
        "local_sigmoid_k",
        default_value="15.0",
        description="Sigmoid slope parameter used when gradient_mode=local",
    )
    declare_local_sigmoid_x0 = DeclareLaunchArgument(
        "local_sigmoid_x0",
        default_value="0.70",
        description="Sigmoid midpoint parameter used when gradient_mode=local",
    )

    declare_global_inflation_radius = DeclareLaunchArgument(
        "global_inflation_radius",
        default_value="3",
        description="Inflation radius (cells) used when gradient_mode=global",
    )
    declare_global_inflation_factor = DeclareLaunchArgument(
        "global_inflation_factor",
        default_value="5.0",
        description="Inflation decay factor used when gradient_mode=global",
    )
    declare_global_sigmoid_k = DeclareLaunchArgument(
        "global_sigmoid_k",
        default_value="15.0",
        description="Sigmoid slope parameter used when gradient_mode=global",
    )
    declare_global_sigmoid_x0 = DeclareLaunchArgument(
        "global_sigmoid_x0",
        default_value="0.70",
        description="Sigmoid midpoint parameter used when gradient_mode=global",
    )

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
            }
        ],
    )

    # TODO: remove this dangerous shit
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "map"],
        condition=IfCondition(PythonExpression(['"', gradient_mode, '" == "global"'])),
    )

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
                "inflation_radius": inflation_radius,
                "inflation_factor": inflation_factor,
                "sigmoid_k": sigmoid_k,
                "sigmoid_x0": sigmoid_x0,
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
            declare_local_inflation_radius,
            declare_global_inflation_radius,
            declare_local_inflation_factor,
            declare_global_inflation_factor,
            declare_local_sigmoid_k,
            declare_global_sigmoid_k,
            declare_local_sigmoid_x0,
            declare_global_sigmoid_x0,
            traversability_filter,
            static_transform_publisher,
            traversability_map,
        ]
    )
