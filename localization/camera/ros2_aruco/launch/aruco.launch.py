import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_aruco')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'dual_cam_setup.rviz')

    multiview = LaunchConfiguration('multiview', default='true')
    rviz = LaunchConfiguration('rviz', default='false')


    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='multiview_aruco_node',
            name='aruco_node',
            output='screen',
            condition=IfCondition(multiview)
        ),

        # Node(
        #     package='ros2_aruco',
        #     executable='aruco_node',
        #     name='aruco_node',
        #     output='screen',
        #     condition=UnlessCondition(multiview)
        # ),

        # Node(
        #     package='ros2_aruco',
        #     executable='pose_estimation_node',
        #     name='pose_estimation_node',
        #     output='screen',
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        #     condition=IfCondition(rviz)
        # ),

        # Node(
        #     package='ros2_aruco',
        #     executable='plot_arucos',
        #     name='plot_arucos',
        #     output='screen',
        # ),

    ])
