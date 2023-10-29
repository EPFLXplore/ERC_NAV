from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_filter',
            executable='obstacle_filter_node',
            name='NAV_obstacle_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'gridshape_y': 8, # total 144
                'gridshape_x': 8, # total 256
                'max_depth': 3000,
                'min_depth': 800,
                'num_iter': 50,
                'thresh_dist': 20,
                'num_inliers': 360,
                'camera_angle': 17,
                'obstacle_angle': 45,
                'how_to_replace': 'random',
                'filter': 'ransac',
                'device': 'cpu'
                }
            ]
        )
    ])
    
    

