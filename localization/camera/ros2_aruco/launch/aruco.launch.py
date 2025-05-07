import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
# import xacro


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_aruco')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'dual_cam_setup.rviz')
    # xacro_file_path = os.path.join(package_dir, 'urdf', 'simple_dual_camera.urdf.xacro')

    sim = LaunchConfiguration('sim', default='false')
    multiview = LaunchConfiguration('multiview', default='true')
    initial_pose = LaunchConfiguration('initial_pose', default='start')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    rviz = LaunchConfiguration('rviz', default='false')
    # description = LaunchConfiguration('description', default='true')

    # Process Xacro file
    # robot_description = xacro.process_file(xacro_file_path).toxml()

    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='multiview_aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{'sim': sim}],
            condition=IfCondition(multiview)
        ),

        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{'sim': sim}],
            condition=UnlessCondition(multiview)
        ),

        Node(
            package='ros2_aruco',
            executable='pose_estimation_node',
            name='pose_estimation_node',
            output='screen',
            parameters=[{'sim': sim, 'initial_pose': initial_pose, 'x': x, 'y': y}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(rviz)
        ),

        # Static transform publisher for left realsense camera
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_left_realsense',
        #     arguments=['-0.3', '0.05', '0.645', '0', '0.3491', '1.9199', 'base_link', 'left_realsense_camera_link'],
        #     output='screen'
        # ),

        # # Static transform publisher for right realsense camera
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_right_realsense',
        #     arguments=['-0.3', '-0.05', '0.645', '0', '0.3491', '-1.9199', 'base_link', 'right_realsense_camera_link'],
        #     output='screen'
        # ),


        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description}],
        #     condition=IfCondition(description)
        # ),
    ])
