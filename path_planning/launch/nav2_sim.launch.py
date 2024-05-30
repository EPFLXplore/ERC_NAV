import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Launch Arguments -------------
    default_use_fake_odom = "true"
    use_fake_odom_arg = DeclareLaunchArgument(
        "use_fake_odom",
        default_value=default_use_fake_odom,
        description="Use fake odometry",
    )

    use_fake_odom = LaunchConfiguration("use_fake_odom", default=default_use_fake_odom)

    # ------------- Setup Paths -------------
    pkg_name = "path_planning"

    pkg_share_dir = get_package_share_directory(pkg_name)
    wheels_control_share_dir = get_package_share_directory("wheels_control")
    nav2_ros_share_dir = get_package_share_directory("nav2_bringup")

    map_server_params_config_path = os.path.join(
        pkg_share_dir, "config", "map_server_params.yaml"
    )
    nav2_params_config_path = os.path.join(pkg_share_dir, "config", "nav2_params_sim.yaml")
    ekf_config_path = os.path.join(pkg_share_dir, "config", "ekf.yaml")

    # ------------- Launch Commands -------------
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_ros_share_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params_config_path,
            "map": map_server_params_config_path,
        }.items(),
    )

    start_wheels_control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheels_control_share_dir, "launch", "wheels_control.launch.py")
        ),
        launch_arguments={
            "motor_cmds": "false",
        }.items(),
    )

    # ------------- Launch Nodes -------------
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_path,
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(use_fake_odom),
    )

    fake_map_tf_publisher_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    fake_odom_tf_publisher_node = launch_ros.actions.Node(
        package="path_planning",
        executable="fake_odom_tf_publisher.py",
        name="fake_odom_tf_publisher",
        output="screen",
        condition=IfCondition(use_fake_odom),
    )

    return [
        # Arguments
        use_fake_odom_arg,
        # Commands
        start_nav2_cmd,
        start_wheels_control_cmd,
        # Nodes
        robot_localization_node,
        fake_map_tf_publisher_node,
        fake_odom_tf_publisher_node,
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
