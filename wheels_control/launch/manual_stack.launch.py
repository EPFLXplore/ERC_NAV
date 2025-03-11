import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context: launch.LaunchContext, *args, **kwargs):

    #-------------- File Paths ------------------
    local_ekf_file_path = os.path.join(get_package_share_directory("path_planning"), "config", "minimal_local_ekf.yaml")
    config_dir_madgwick = os.path.join(get_package_share_directory('imu_madgwick'), 'config')
    
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

    motor_cmds = LaunchConfiguration("motor_cmds", default=default_motor_cmds)
    homing = LaunchConfiguration("homing", default=default_homing)

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

    # ------------- Ouster Launch File -------------
    ouster_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("ros2_ouster").find("ros2_ouster"), "launch", "driver_launch.py")
        ),
        launch_arguments={}.items(),
    )

    # -------------- ERC_CAMERAS NAV Launch file --------
    nav_cameras_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("camera").find("camera"), "launch", "camera_node_nav.launch.py")
        ),
        launch_arguments={}.items(),
    )

    # -........... liorf Launch File ---------------
    liorf_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("liorf").find("liorf"), "launch", "run_lio_sam_ouster.launch.py")
        ),
        launch_arguments={"params_file": os.path.join(
            get_package_share_directory("liorf"), "config", "lio_sam_ouster.yaml"
        )}.items(),
    )

    #----------- ArUco Launch File ---------

    aruco_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("ros2_aruco").find("ros2_aruco"), "launch", "aruco.launch.py")
        )
    )
    # -........... description (URDF) Launch File ---------------
    description_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("xplore_description").find("xplore_description"), "launch", "xplore_real.launch.py")
        )
    )

    # ------------- Nodes to Start AFTER Ouster is Ready -------------
    local_ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_filter_node',
        output='screen',
        parameters=[local_ekf_file_path]
    )


    imu_filter_node = launch_ros.actions.Node(
        package='ouster_imu_tester',
        executable='ouster_imu_tester',
        name='imu_filter',
        output='screen'
    )
    
    imu_madgwick_filter = launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='ouster_madgwick_filter',
                output='screen',
                parameters=[os.path.join(config_dir_madgwick, 'imu_madgwick_filter.yaml')],
    )

    delayed_launch = TimerAction(
        period=24.0,  # Wait 24 sec. before launching other nodes because the ouster driver is slow
        actions=[
            LogInfo(msg="Ouster started! Launching dependent nodes..."),
            liorf_launch,
            imu_madgwick_filter,
            local_ekf_node,
        ]
    )

    delayed_aruco_launch = TimerAction(
        period=7.0,  # Wait 7 sec. before launching other nodes
        actions=[
            LogInfo(msg="Cameras started! Launching dependent nodes..."),
            aruco_launch
        ]
    )


    return [
        #motor_cmds_arg,
        #homing_arg,
        #cs_interface,
        #gamepad_interface_node,
        #cmd_vel_manager_node,
        #displacement_cmds_node,
        #motor_cmds_node,
        #description_launch,
        #wheel_odom_node,
        #imu_filter_node,
        ouster_launch,
        delayed_launch,
        #nav_cameras_launch,
        #delayed_aruco_launch
    ]

def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
