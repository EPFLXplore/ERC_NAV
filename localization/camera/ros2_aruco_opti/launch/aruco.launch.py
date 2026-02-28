import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_aruco_opti')
    default_rviz_config = os.path.join(package_dir, 'rviz', 'dual_cam_setup.rviz')

    rviz = LaunchConfiguration('rviz', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 if true',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz config file',
        ),

        Node(
            package='ros2_aruco_opti',
            executable='multiview_aruco_node',
            name='aruco_node',
            output='screen',
        ),

        Node(
            package='ros2_aruco_opti',
            executable='pose_estimation_node',
            name='pose_estimation_node',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])
