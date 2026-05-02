import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

# ANSI colors for launch-time terminal messages (LogInfo / print). Ineffective when stdout is not a TTY.
_C_RESET = "\033[0m"
_C_BOLD_GREEN = "\033[1;32m"
_C_BOLD_YELLOW = "\033[1;33m"
DEFAULT_ARUCO_STACK_DELAY = "18.0"


def launch_setup(context: launch.LaunchContext, *args, **kwargs):

    #-------------- File Paths ------------------
    local_ekf_file_path = os.path.join(get_package_share_directory("path_planning"), "config", "minimal_local_ekf.yaml")
    global_ekf_file_path = os.path.join(get_package_share_directory("path_planning"), "config", "global_ekf_real.yaml") 
    
    # ------------- Launch Arguments -------------
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

    default_pub_urdf = "true"
    pub_urdf_arg = DeclareLaunchArgument(
        "pub_urdf",
        default_value=default_pub_urdf,
        description="Publish the URDF via the robot state publisher"
    )

    default_launch_lidar = "true"
    launch_lidar_arg = DeclareLaunchArgument(
        "launch_lidar",
        default_value=default_launch_lidar,
        description="Launch Ouster os_driver (driver.launch.py + driver_params.yaml)",
    )

    # After camera_node_nav: last SetBool at 9s; give JPEG + camera_info a short buffer
    # before multiview_aruco blocks on intrinsics service calls.
    motor_cmds = LaunchConfiguration("motor_cmds", default=default_motor_cmds)
    homing = LaunchConfiguration("homing", default=default_homing)
    publish_urdf = LaunchConfiguration("pub_urdf", default=default_pub_urdf)
    launch_lidar = LaunchConfiguration("launch_lidar", default=default_launch_lidar)

    delay_str = LaunchConfiguration(
        "aruco_stack_delay_sec",
        default=DEFAULT_ARUCO_STACK_DELAY,
    ).perform(context)
    try:
        aruco_delay_s = float(delay_str)
    except ValueError:
        aruco_delay_s = float(DEFAULT_ARUCO_STACK_DELAY)
    if aruco_delay_s < 0.0:
        aruco_delay_s = 0.0

    # ------------- Launch Nodes -------------
    cs_interface = launch_ros.actions.Node(
        package="interfacing_nav_cs",
        executable="interface",
        name="NavCSInterfacing",
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
        parameters=[{"motor_cmds": motor_cmds}],
    )

    motor_cmds_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_motor_cmds",
        name="NAV_motor_cmds",
        parameters=[{"homing": homing}],
        condition=IfCondition(motor_cmds),
    )

    wheel_odom_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_odometry_node",
        name="NAV_odometry_node",
    )


    # ------------- Ouster (ouster_ros os_driver) -------------
    # Use driver.launch.py (not driver_launch.py): driver_launch.py defaults to
    # community_driver_config.yaml and hardcodes namespace/name that do not match driver_params.yaml.
    ouster_share = get_package_share_directory("ouster_ros")
    ouster_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ouster_share, "launch", "driver.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(ouster_share, "config", "driver_params.yaml"),
            "ouster_ns": "ouster",
            "os_driver_name": "os_driver",
            "viz": "False",
        }.items(),
        condition=IfCondition(launch_lidar),
    )


    # -------------- ERC_CAMERAS NAV Launch file --------
    nav_cameras_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("camera").find("camera"), "launch", "camera_node_nav.launch.py")
        ),
        launch_arguments={}.items(),
    )


    #----------- ArUco Launch Files ---------

    aruco_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("ros2_aruco_cpp").find("ros2_aruco_cpp"), "launch", "aruco.launch.py")
        )
    )
    aruco_lidar_detection_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("ros2_aruco_with_lidar").find("ros2_aruco_with_lidar"), "launch", "aruco_lidar.launch.py")
        )
    )
    # -........... description (URDF) Launch File ---------------
    description_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("xplore_description").find("xplore_description"), "launch", "xplore_real.launch.py")
        ),
        condition=IfCondition(publish_urdf)
    )


    custom_local_ekf_node = launch_ros.actions.Node(
        package='local_nav_ekf',
        executable='nav_ekf_3d_node',
        name='nav_custom_ekf_3d',
        output='screen',
        parameters=[{'include_lidar': False, 'include_aruco': False, 'include_vio': False}]
    )

    olive_imu_restamp_node = launch_ros.actions.Node(
        package='olive_imu_restamper',
        executable='olive_imu_restamper_node',
        name='olive_imu_restamper_node',
        output='screen',
    )


    stack_boot_hint = TimerAction(
        period=1.0,
        actions=[
            LogInfo(
                msg=(
                    f"{_C_BOLD_YELLOW}manual_stack: ArUco+lidar_aruco start in "
                    f"{delay_str}s (arg aruco_stack_delay_sec). "
                    f"Silence until then is normal; cameras SetBool at ~9s.{_C_RESET}"
                )
            ),
        ],
    )

    delayed_aruco_launch = TimerAction(
        period=aruco_delay_s,
        actions=[
            LogInfo(
                msg=f"{_C_BOLD_GREEN}Launching ArUco + lidar_aruco stack...{_C_RESET}"
            ),
            aruco_launch,
            LogInfo(
                msg=f"{_C_BOLD_YELLOW}Aruco C++ launched; starting lidar_aruco...{_C_RESET}"
            ),
            aruco_lidar_detection_launch,
        ],
    )
    
    jetson_stats = launch_ros.actions.Node(
        package="jetson_stats",
        executable="launch_stats"
    )


    # Static transform from erc_map → map
    erc_to_map_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="erc_map_to_map_tf",
        arguments=[
            "0.0", "0.0", "0.0",          # translation
            "0.0", "0.0", "0.0",  # rotation in ZYX ?
            "erc_map", "map"              # parent → child
        ],
        output="screen"
    )

    slip_control_node = launch_ros.actions.Node(
        package="wheels_control",
        executable="NAV_steer_control",
        name="motor_steering_servoing",
        output="screen"
    )


    return [
        motor_cmds_arg,
        homing_arg,
        pub_urdf_arg,
        launch_lidar_arg,
        stack_boot_hint,
        cs_interface,
        gamepad_interface_node,
        cmd_vel_manager_node,
        displacement_cmds_node,
        motor_cmds_node,
        description_launch,
        olive_imu_restamp_node,
        wheel_odom_node,
        custom_local_ekf_node,
        ouster_launch,
        nav_cameras_launch,
        delayed_aruco_launch,
        jetson_stats,
        slip_control_node,
    ]

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "aruco_stack_delay_sec",
            default_value=DEFAULT_ARUCO_STACK_DELAY,
            description="Seconds after stack start before ArUco + lidar_aruco launches (cameras need stream first).",
        ),
        OpaqueFunction(function=launch_setup),
    ])