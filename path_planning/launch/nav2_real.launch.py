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
    nav2_params_config_path = os.path.join(pkg_share_dir, "config", "nav2_params_real.yaml")

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
    # TODO

    return [
        # Commands
        start_nav2_cmd,
        start_wheels_control_cmd,
        # Nodes
        # TODO
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
