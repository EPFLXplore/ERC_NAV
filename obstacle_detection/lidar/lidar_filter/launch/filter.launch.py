from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value = 'false'),
        DeclareLaunchArgument('slope_threshold', default_value = '0.4', description = '1 => 45 °'),
        DeclareLaunchArgument('min_distance', default_value = '0.8'),
        DeclareLaunchArgument('n_clusters', default_value = '50'),


   


        Node(
            package='lidar_filter',
            executable='NAV_lidar_filter_node',
            name='NAV_lidar_filter_node',
            parameters=[{'sim': LaunchConfiguration('sim'),
                         'slope_threshold': LaunchConfiguration('slope_threshold'), 
                         'min_distance': LaunchConfiguration('min_distance'),
                         'n_clusters': LaunchConfiguration('n_clusters')
                         }]
        ),
    ])
