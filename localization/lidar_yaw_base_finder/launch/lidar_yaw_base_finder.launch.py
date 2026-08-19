from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters_file = PathJoinSubstitution([
        FindPackageShare('lidar_yaw_base_finder'),
        'config',
        'dead_zone_finder.yaml',
    ])

    return LaunchDescription([
        Node(
            package='lidar_yaw_base_finder',
            executable='lidar_yaw_base_finder',
            name='lidar_yaw_base_finder',
            output='screen',
            parameters=[parameters_file],
        ),
    ])
