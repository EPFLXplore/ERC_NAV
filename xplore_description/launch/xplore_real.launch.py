import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Launch Arguments -------------
    default_use_rviz = "true"
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value=default_use_rviz,
        description="Use rviz if true",
    )

    default_rviz_config = "display_config.rviz"
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Name of the rviz configuration file to be loaded",
    )

    use_rviz = LaunchConfiguration("use_rviz", default=default_use_rviz)
    rviz_config = LaunchConfiguration(
        "rviz_config", default=default_rviz_config
    ).perform(context)

    # ------------- Setup Paths -------------
    pkg_name = "xplore_description"

    pkg_share_dir = get_package_share_directory(pkg_name)

    rviz_config_path = os.path.join(pkg_share_dir, "rviz", rviz_config)

    # ------------- Launch Commands -------------
    robot_state_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, "launch", "robot_state.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    # ------------- Launch Nodes -------------
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return [
        # Arguments
        use_rviz_arg,
        rviz_config_arg,
        # Commands
        robot_state_launch_cmd,
        # Nodes
        rviz_node,
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
