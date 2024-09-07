import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Setup Paths -------------
    pkg_name = "path_planning"

    pkg_share_dir = get_package_share_directory(pkg_name)
    wheels_control_share_dir = get_package_share_directory("wheels_control")
    nav2_ros_share_dir = get_package_share_directory("nav2_bringup")

    map_server_params_config_path = os.path.join(
        pkg_share_dir, "config", "map_server_params.yaml"
    )
    nav2_params_config_path = os.path.join(
        pkg_share_dir, "config", "nav2_params_real.yaml"
    )
    local_ekf_config_path = os.path.join(pkg_share_dir, "config", "local_ekf_real.yaml")
    global_ekf_config_path = os.path.join(
        pkg_share_dir, "config", "global_ekf_real.yaml"
    )

    # ------------- Launch Commands -------------
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_ros_share_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
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
            "motor_cmds": "true",
        }.items(),
    )

    # ------------- Launch Nodes -------------
    local_robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf_filter_node",
        output="screen",
        parameters=[
            local_ekf_config_path,
            {"use_sim_time": False},
        ],
    )
    
    


    global_robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_filter_node",
        output="screen",
        parameters=[
            global_ekf_config_path,
            {"use_sim_time": False},
        ],
    )

    odom_offset_node = launch_ros.actions.Node(
        package="path_planning",
        executable="odom_offset_node.py",
        name="odom_offset_node",
        output="screen",
    )

    return [
        # Commands
        start_nav2_cmd,
        # start_wheels_control_cmd,
        # Nodes
        # local_robot_localization_node,
        # global_robot_localization_node,
        # odom_offset_node
        map,
        
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
