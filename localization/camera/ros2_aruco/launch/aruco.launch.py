from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    rviz_config_file = "src/ros2_aruco/ros2_aruco/rviz/aruco.rviz" #os.path.join(share_dir, 'config', 'rviz2.rviz')

    urdf_file_path = "src/ros2_aruco/ros2_aruco/urdf/simple_camera.urdf.xacro"  
    # urdf_file_path = os.path.join(
    #     os.getenv('AMENT_PREFIX_PATH').split(':')[0],
    #     'share', 'ros2_aruco', 'urdf', urdf_file_name
    # )

    sim = LaunchConfiguration('sim', default=False)
    multiview = LaunchConfiguration('multiview', default=False)
    initial_pose = LaunchConfiguration('initial_pose', default='start')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    rviz = LaunchConfiguration('rviz', default=False)
    description = LaunchConfiguration('description', default=False) # publish simple urdf




    return LaunchDescription([
        

        Node(
            package='ros2_aruco',
            executable='multiview_aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{'sim': sim}] ,
            condition=IfCondition(multiview)
        ),

        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[{'sim': sim}] ,
            condition=UnlessCondition(multiview)    
        ),

        Node(
            package='ros2_aruco',
            executable='pose_estimation_node',
            name='pose_estimation_node',
            output='screen',
            parameters=[{'sim': sim, 'initial_pose': initial_pose, 'x': x, 'y':y}] 
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(rviz)
        ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': open(urdf_file_path).read()}],
        #     condition=IfCondition(description)
        # ),

    ])

if __name__ == '__main__':
    generate_launch_description()
