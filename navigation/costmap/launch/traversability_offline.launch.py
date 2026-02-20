from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    traversability_pkg = get_package_share_directory('traversability_mapping')
    
    # Static transforms for sensor setup
    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_base_link',
            arguments=['-0.18209', '0.1575', '0.65', '0', '0', '0', 'base_link', 'velodyne']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='odom_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),
    ]
    
    # Traversability mapping nodes
    traversability_nodes = [
        Node(
            package='traversability_mapping',
            executable='traversability_filter',
            name='traversability_filter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='traversability_mapping',
            executable='traversability_map',
            name='traversability_map',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Node(
        #     package='traversability_mapping',
        #     executable='traversability_prm',
        #     name='traversability_prm',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Node(
        #     package='traversability_mapping',
        #     executable='traversability_path',
        #     name='traversability_path',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ]
    
    # RViz node (optional - you may need to adjust the config file path)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Declare use_sim_time parameter
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Add all nodes
        *static_transforms,
        *traversability_nodes,
        # rviz_node
    ])
