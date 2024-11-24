
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_package",
            default_value="ouster_imu_tester",
            description="Package containing the IMU low-pass node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_executable",
            default_value="ouster_imu_tester",
            description="Executable for the IMU low-pass filter node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "liorf_package",
            default_value="liorf",
            description="Package containing the LIO-SAM launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "driver_package",
            default_value="ros2_ouster",
            description="Package containing the Ouster driver.",
        )
    )

    # Nodes and Launch Files
    driver_node = Node(
        package=LaunchConfiguration("driver_package"),
        executable="driver_launch.py",
        name="ouster_driver",
        output="screen",
    )

    imu_node = Node(
        package=LaunchConfiguration("imu_package"),
        executable=LaunchConfiguration("imu_executable"),
        name="ouster_imu_lowpass_filter",
        output="screen",
    )

    liorf_node = Node(
        package=LaunchConfiguration("liorf_package"),
        executable="run_lio_sam_ouster.launch.py",
        name="liorf",
        output="screen",
    )

    # Timed Actions
    driver_timer = TimerAction(period=30.0, actions=[driver_node])
    imu_timer = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=driver_node,
            on_exit=[
                TimerAction(period=2.0, actions=[imu_node]),
            ],
        )
    )
    liorf_timer = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_node,
            on_exit=[
                TimerAction(period=2.0, actions=[liorf_node]),
            ],
        )
    )

    # Launch Description
    return LaunchDescription(
        declared_arguments + [driver_timer, imu_timer, liorf_timer]
    )
