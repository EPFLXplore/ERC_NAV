from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch GLIM ROS node
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            output='screen',
            parameters=[{'config_path': '/home/xplore/dev_ws/src/localization/lidar/glim_starter/glim_config/config'}]
        ),

        # Static TF: imu_link_glim <- base_link_glim
        # syntax : xyz yaw pitch roll

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu_to_base',
            arguments=['0.235', '-0.15', '0.0', '0', '0', '0', 'base_link_glim', 'imu_link_glim'],
        ),

        # Static TF: lidar_link_glim <- base_link_glim
        # syntax : xyz yaw pitch roll
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar_to_base',
            arguments=['0.0', '0.0','0.6', '-1.5708', '0.0', '0.0', 'base_link_glim', 'lidar_link_glim'],
        ),

        # Launch the GLIM odometry republisher that chagnes the covariance to avoids tfs conflicts
        Node(
            package='glim_starter',
            executable='glim_odom_publisher',
            name='glim_odom_publisher_node',
            output='screen',
            parameters=[
                {'pose_topic': '/glim_rosnode/pose_corrected'},
                {'odom_topic': '/odom_glim_repub'},
                {'odom_frame_id': 'odom'},
                {'child_frame_id': 'base_link'},

                {'position_covariance': [0.05, 0.0, 0.0,
                                         0.0, 0.05, 0.0,
                                         0.0, 0.0, 0.10]},
                {'orientation_covariance': [0.02, 0.0, 0.0,
                                            0.0, 0.02, 0.0,
                                            0.0, 0.0, 0.04]},

                {'reflect_x_after': True},     # set True only if a downstream consumer expects the old sign
                {'rotation_sign': -1},          # try +1 only if motion still looks “old-frame”
                {'debug_logs': False},
            ]
        )
    ])
