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

    motor_cmds = LaunchConfiguration("motor_cmds", default=default_motor_cmds)
    homing = LaunchConfiguration("homing", default=default_homing)
    publish_urdf = LaunchConfiguration("pub_urdf", default=default_pub_urdf)
    

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
        condition=IfCondition(publish_urdf)
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
        executable='nav_ekf_node',
        name='nav_custom_ekf',
        output='screen',
        parameters=[{'include_lidar': True}]
    )

    olive_imu_restamp_node = launch_ros.actions.Node(
        package='olive_imu_restamper',
        executable='olive_imu_restamper_node',
        name='olive_imu_restamper_node',
        output='screen',
    )


    delayed_aruco_launch = TimerAction(
        period=7.0,  # Wait 7 sec. before launching other nodes
        actions=[
            LogInfo(msg="Cameras started! Launching aruco nodes..."),
            aruco_launch,
            aruco_lidar_detection_launch
        ]
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
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])