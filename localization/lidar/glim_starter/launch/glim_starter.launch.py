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
            arguments=['-0.16', '-0.18', '0.6', '-1.5708', '0.0', '0.0', 'base_link_glim', 'lidar_link_glim'],
        ),

        # Launch the GLIM odometry republisher that changes the covariance to avoid TF conflicts.
        # Use the live odometry pose for navigation; pose_corrected comes from mapping/correction
        # and can lag behind real motion under load.
        Node(
            package='glim_starter',
            executable='glim_odom_publisher',
            name='glim_odom_publisher_node',
            output='screen',
            parameters=[
                {'pose_topic': '/glim_rosnode/pose'},
                {'odom_topic': '/odom_glim_repub'},
                {'odom_frame_id': 'odom'},
                {'child_frame_id': 'base_link'},

                {'position_covariance': [0.05, 0.0, 0.0,
                                         0.0, 0.05, 0.0,
                                         0.0, 0.0, 0.10]},
                {'orientation_covariance': [0.02, 0.0, 0.0,
                                            0.0, 0.02, 0.0,
                                            0.0, 0.0, 0.04]},

                # Keep init strategy (capture after 5s).
                {'initial_align_yaw_offset_rad': 1.5707963267948966},
                {'output_plane_rotation_rad': -1.5707963267948966},
                # Use captured yaw for stable odom alignment.
                {'use_initial_yaw_for_alignment': True},
                # Your case: base_link +X appeared as odom -Y -> apply +pi/2 remap on XY.
                {'remap_body_x_from_odom_neg_y': True},
                {'remap_neg_y_to_plus_x_include_yaw': False},
                {'zero_z': True},
                {'reflect_x_after': False},
                {'rotation_sign': False},
                {'invert_output_x': False},
                {'debug_logs': False},
            ]
        )
    ])
