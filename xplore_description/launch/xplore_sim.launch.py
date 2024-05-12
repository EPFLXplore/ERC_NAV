import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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

    default_use_gazebo = "true"
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo",
        default_value=default_use_gazebo,
        description="Use gazebo if true",
    )

    default_rviz_config = "display_config.rviz"
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Name of the rviz configuration file to be loaded",
    )

    default_world_model = "marsyard2022.world.xacro"
    world_model_arg = DeclareLaunchArgument(
        "world_model",
        default_value=default_world_model,
        description="Name of the world model file to be loaded",
    )

    use_rviz = LaunchConfiguration("use_rviz", default=default_use_rviz)
    use_gazebo = LaunchConfiguration("use_gazebo", default=default_use_gazebo)
    rviz_config = LaunchConfiguration(
        "rviz_config", default=default_rviz_config
    ).perform(context)
    world_model = LaunchConfiguration(
        "world_model", default=default_world_model
    ).perform(context)

    # ------------- Setup Paths -------------
    pkg_name = "xplore_description"

    pkg_share_dir = get_package_share_directory(pkg_name)
    gazebo_ros_share_dir = get_package_share_directory("gazebo_ros")

    rviz_config_path = os.path.join(pkg_share_dir, "rviz", rviz_config)
    world_model_path = os.path.join(pkg_share_dir, "worlds", world_model)
    control_config_path = os.path.join(pkg_share_dir, "config", "control.yaml")
    ekf_config_path = os.path.join(pkg_share_dir, "config", "ekf.yaml")

    gazebo_model_path = os.path.join(pkg_share_dir, "models")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_model_path

    # ------------- Launch Commands -------------
    start_gazebo_server_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_model_path}.items(),
        condition=IfCondition(use_gazebo),
    )

    robot_state_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, "launch", "robot_state.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # ------------- Launch Nodes -------------
    spawn_entity_gazebo_node = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "4.0",
            "-timeout",
            "60",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[control_config_path],
        output="both",
    )

    joint_state_broadcaster_spawner_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
    )

    position_controller_spawner_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
        ],
    )

    velocity_controller_spawner_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
        ],
    )

    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_path,
            {"use_sim_time": True},
        ],
    )

    # ------------- Delayed Launch Nodes -------------
    # Delay rviz_node start after joint_state_broadcaster_spawner_node
    delay_rviz_after_joint_state_broadcaster_spawner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[rviz_node],
        )
    )

    # Delay robot_localization_node start after joint_state_broadcaster_spawner_node
    delay_robot_localization_after_joint_state_broadcaster_spawner_node = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner_node,
                on_exit=[robot_localization_node],
            )
        )
    )

    return [
        # Arguments
        use_rviz_arg,
        use_gazebo_arg,
        rviz_config_arg,
        world_model_arg,
        # Commands
        start_gazebo_server_client_cmd,
        robot_state_launch_cmd,
        # Nodes
        spawn_entity_gazebo_node,
        control_node,
        joint_state_broadcaster_spawner_node,
        position_controller_spawner_node,
        velocity_controller_spawner_node,
        delay_rviz_after_joint_state_broadcaster_spawner_node,
        delay_robot_localization_after_joint_state_broadcaster_spawner_node,
        launch_ros.actions.Node(
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
        ),
        launch_ros.actions.Node(
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
                "odom",
                "--child-frame-id",
                "base_link",
            ],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
