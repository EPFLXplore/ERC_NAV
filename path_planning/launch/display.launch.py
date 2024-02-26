import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="path_planning").find(
        "path_planning"
    )
    default_model_path = os.path.join(pkg_share, "src/description/rover.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", default_model_path])}],
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[default_model_path],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
    )

    return launch.LaunchDescription(
        [rviz_node, robot_state_publisher_node, joint_state_publisher_node]
    )
