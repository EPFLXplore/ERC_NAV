import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Launch Arguments -------------
    default_use_sim_time = "true"
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=default_use_sim_time,
        description="Use sim time if true",
    )

    default_use_joint_gui = "false"
    use_joint_gui_arg = DeclareLaunchArgument(
        "use_joint_gui",
        default_value=default_use_joint_gui,
        description="Use joint state publisher gui",
    )

    default_rover_urdf_file = "rover/rover.urdf.xacro"
    rover_urdf_file_arg = DeclareLaunchArgument(
        "rover_urdf_file",
        default_value=default_rover_urdf_file,
        description="Relative path, inside the urdf directory, of the rover URDF file to be loaded",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default=default_use_sim_time)
    use_joint_gui = LaunchConfiguration("use_joint_gui", default=default_use_joint_gui)
    rover_urdf_file = LaunchConfiguration(
        "rover_urdf_file", default=default_rover_urdf_file
    ).perform(context)

    # ------------- Setup Paths -------------
    pkg_name = "xplore_description"

    pkg_share_dir = get_package_share_directory(pkg_name)

    rover_model_path = os.path.join(pkg_share_dir, "urdf", rover_urdf_file)

    # ------------- Launch Nodes -------------
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", rover_model_path]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[rover_model_path],
        condition=UnlessCondition(use_joint_gui),
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_joint_gui),
    )

    return [
        # Arguments
        use_sim_time_arg,
        use_joint_gui_arg,
        rover_urdf_file_arg,
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
