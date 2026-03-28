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
    traversability_pkg = get_package_share_directory('gradient_costmap')
    
    
    # Traversability mapping nodes
    traversability_nodes = [
        Node(
            package='gradient_costmap',
            executable='traversability_filter',
            name='traversability_filter',
            output='screen',
            parameters=[{'use_sim_time': True}],
            emulate_tty=True
        ),
        Node(
            package='gradient_costmap',
            executable='traversability_map',
            name='traversability_map',
            output='screen',
            parameters=[{'use_sim_time': True, 
                         'inflation_radius': 2, #int representing the radius of the kernel in cells, make the gradient wider with bigger radius 
                         'inflation_factor': 0.01,
                         'sigmoid_k': 13.0, # between 8.0 and 20.0 for sharper gradient, between 0.5 and 1.0 for softer gradient
                         'sigmoid_x0': 0.7}], # between 0.45 to 0.60 shift the sigmoid curve to the left or right, can be used to make the gradient sharper or softer around the threshold (0.5 in this case)
        # inflation factor makes the gradient sharpes (get blacker faster with high factor)
        # inflation radius makes the gradient wider (get blacker in a wider area with bigger kernel)
            emulate_tty=True
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
        # *static_transforms,
        *traversability_nodes,
        # rviz_node
    ])
