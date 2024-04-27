from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_filter',
            executable='slope_filter_node',
            name='slope_filter',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'gridshape_y': 4,
                'gridshape_x': 8,
                'max_height': 2000,
                'max_dist': 6000,
                'num_iter': 50,
                'thresh_dist': 200,
                'num_inliers': 200,
                'obstacle_angle': 45,
                'how_to_replace': 'random',
                'filter': 'ransac',
                'device': 'cpu'
                }
            ]
        )
    ])
    
    

