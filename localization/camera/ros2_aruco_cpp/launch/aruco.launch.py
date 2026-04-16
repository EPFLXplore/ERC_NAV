import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([

        LogInfo(msg="Before aruco cpp in launch file..."),

        Node(
            package='ros2_aruco_cpp',
            executable='multiview_aruco_node',
            name='aruco_node',
            output='screen',
        ),

        LogInfo(msg="After aruco cpp before lidar launch file in launch file"),
        Node(
            package='ros2_aruco_cpp',
            executable='pose_estimator_lidar_node',
            name='pose_estimation_node_with_lidar',
            output='screen',
        ),
        LogInfo(msg="After lidar launch file in launch file"),

    ])
