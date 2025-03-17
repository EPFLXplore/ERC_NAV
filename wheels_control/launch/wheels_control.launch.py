import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context: launch.LaunchContext, *args, **kwargs):

    #-------------- File Paths ------------------
    local_ekf_file_path = os.path.join(get_package_share_directory("path_planning"), "config", "local_ekf_liorf-wheel.yaml")

    # ------------- Launch Arguments -------------
    default_fake_cs_gamepad = "false"
    fake_cs_gamepad_arg = DeclareLaunchArgument(
        "fake_cs_gamepad",
        default_value=default_fake_cs_gamepad,
        description="Use fake CS gamepad",
    )

    default_motor_cmds = "true"
    motor_cmds_arg = DeclareLaunchArgument(
        "motor_cmds",
        default_value=default_motor_cmds,
        description="Use motor commands",
    )

    default_homing = "false"
    homing_arg = DeclareLaunchArgument(
        "homing",
        default_value=default_homing,
        description="Use homing",
    )

    fake_cs_gamepad = LaunchConfiguration(
        "fake_cs_gamepad", default=default_fake_cs_gamepad
    )
    motor_cmds = LaunchConfiguration("motor_cmds", default=default_motor_cmds)
    homing = LaunchConfiguration("homing", default=default_homing)

    # ------------- Launch Nodes -------------
    fake_cs_gamepad_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="fake_cs_gamepad.py",
        name="NAV_fake_cs_gamepad",
        condition=IfCondition(fake_cs_gamepad),
    )

    gamepad_interface_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_gamepad_interface",
        name="NAV_gamepad_interface",
    )

    cmd_vel_manager_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_cmd_vel_manager",
        name="NAV_cmd_vel_manager",
    )

    cs_interface = launch_ros.actions.Node(
        package="interfacing_nav_cs",
        executable="interface",
        name="NavCSInterfacing",
    )

    displacement_cmds_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_displacement_cmds",
        name="NAV_displacement_cmds",
        parameters=[{"motor_cmds": motor_cmds}],
    )

    motor_cmds_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_motor_cmds",
        name="NAV_motor_cmds",
        parameters=[{"homing": homing}],
        condition=IfCondition(motor_cmds),
    )

    front_camera = launch_ros.actions.Node(
        package='camera',
        executable='camera',
        name='camera_nav_front',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oakd_stereo"},
            {'topic_service': "/NAV/req_camera_nav_0"},
            {'topic_pub': "/NAV/feed_camera_nav_0"},
            {'bw_pub': "/NAV/bw_camera_nav_0"}, 
            #the devrule is already in the dockerfile
        ],
    )

    wheel_odom_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_odometry_node",
        name="NAV_odometry_node",
    )

    local_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name = 'local_ekf_filter_node',
        output = 'screen',
        parameters=[local_ekf_file_path]
    )



    return [
        # Arguments
        motor_cmds_arg,
        homing_arg,
        # Nodes
        cs_interface,
        gamepad_interface_node,
        cmd_vel_manager_node,
        displacement_cmds_node,
        motor_cmds_node,
        #front_camera,
       # wheel_odom_node,
       # local_ekf_node
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
