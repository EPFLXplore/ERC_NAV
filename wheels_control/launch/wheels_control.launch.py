import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    # ------------- Launch Arguments -------------
    default_fake_cs_gamepad = "false"
    fake_cs_gamepad_arg = DeclareLaunchArgument(
        "fake_cs_gamepad",
        default_value=default_fake_cs_gamepad,
        description="Use fake CS gamepad",
    )

    default_motor_cmds = "false"
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

    displacement_cmds_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_displacement_cmds",
        name="NAV_displacement_cmds",
    )

    motor_cmds_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_motor_cmds",
        name="NAV_motor_cmds",
        parameters=[{"homing": homing}],
        condition=IfCondition(motor_cmds),
    )

    return [
        # Arguments
        fake_cs_gamepad_arg,
        motor_cmds_arg,
        homing_arg,
        # Nodes
        fake_cs_gamepad_node,
        gamepad_interface_node,
        cmd_vel_manager_node,
        displacement_cmds_node,
        motor_cmds_node,
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
