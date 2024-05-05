import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Launch Arguments -------------
    default_use_sim_time = "true"
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=default_use_sim_time,
        description="Use sim time if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default=default_use_sim_time)

    # ------------- Setup Paths -------------
    pkg_name = "path_planning"

    pkg_share_dir = get_package_share_directory(pkg_name)
    nav2_ros_share_dir = get_package_share_directory("nav2_bringup")

    map_server_params_config_path = os.path.join(
        pkg_share_dir, "config", "map_server_params.yaml"
    )
    nav2_params_config_path = os.path.join(pkg_share_dir, "config", "nav2_params.yaml")

    # ------------- Launch Commands -------------
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_ros_share_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": nav2_params_config_path,
            "map": map_server_params_config_path,
        }.items(),
    )

    return [
        # Arguments
        use_sim_time_arg,
        # Commands
        start_nav2_cmd,
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
