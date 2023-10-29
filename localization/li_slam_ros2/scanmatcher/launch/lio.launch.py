import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('scanmatcher'),
            'param',
            'lio.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        remappings=[('/input_cloud','ouster/points')],
        output='screen'
        )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[mapping_param_dir],
        output='screen'
        )

    # tf = launch_ros.actions.Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','0','0','0','0','1','base_link','ouster']
    #     )

    imu_pre = launch_ros.actions.Node(
        package='scanmatcher',
        executable='imu_preintegration',
        remappings=[('/odometry','/lio_sam/odom')],
        parameters=[mapping_param_dir],
        output='screen'
        )

    img_pro = launch_ros.actions.Node(
        package='scanmatcher',
        executable='image_projection',
        parameters=[mapping_param_dir],
        output='screen'
        )
    

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        mapping,
        # tf,
        imu_pre,
        img_pro,
        graphbasedslam,
            ])