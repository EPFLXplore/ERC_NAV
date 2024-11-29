from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('wheels_control'), 'params', 'backup_gamepad_params.yaml')

#sudo apt-get install ros-humble-joy
#ros2 run joy joy_node # <-- Run in first terminal
#ros2 topic echo /joy # <-- Run in second terminal

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
        remappings=[('/joy', '/backup_gamepad_joy')]
    )


    return LaunchDescription([
        joy_node,
    ])
