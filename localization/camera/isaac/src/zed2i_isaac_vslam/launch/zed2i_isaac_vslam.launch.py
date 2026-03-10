import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial_number = LaunchConfiguration('serial_number').perform(context)
    zed_node_name = LaunchConfiguration('zed_node_name').perform(context)
    zed_wrapper_config = LaunchConfiguration('zed_wrapper_config').perform(context)
    visual_slam_config = LaunchConfiguration('visual_slam_config').perform(context)
    container_name = LaunchConfiguration('container_name').perform(context)
    publish_urdf = LaunchConfiguration('publish_urdf').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or f'{camera_name}_camera_center'
    imu_frame = LaunchConfiguration('imu_frame').perform(context) or f'{camera_name}_imu_link'

    left_optical_frame = f'{camera_name}_left_camera_optical_frame'
    right_optical_frame = f'{camera_name}_right_camera_optical_frame'
    zed_topic_root = f'/{camera_name}/{zed_node_name}'

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': 'zed2i',
            'container_name': container_name,
            'serial_number': serial_number,
            'ros_params_override_path': zed_wrapper_config,
            'publish_urdf': publish_urdf,
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'true',
        }.items(),
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        namespace='',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[
            visual_slam_config,
            {
                'base_frame': base_frame,
                'imu_frame': imu_frame,
                'camera_optical_frames': [left_optical_frame, right_optical_frame],
            },
        ],
        remappings=[
            ('visual_slam/image_0', f'{zed_topic_root}/left_gray/image_rect_gray'),
            ('visual_slam/camera_info_0', f'{zed_topic_root}/left_gray/camera_info'),
            ('visual_slam/image_1', f'{zed_topic_root}/right_gray/image_rect_gray'),
            ('visual_slam/camera_info_1', f'{zed_topic_root}/right_gray/camera_info'),
            ('visual_slam/imu', f'{zed_topic_root}/imu/data'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name=container_name,
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return [zed_wrapper_launch, visual_slam_container, rviz_node]


def generate_launch_description():
    package_share = get_package_share_directory('zed2i_isaac_vslam')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='32835549'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node'),
        DeclareLaunchArgument('container_name', default_value='zed2i_isaac_vslam_container'),
        DeclareLaunchArgument(
            'zed_wrapper_config',
            default_value=os.path.join(package_share, 'config', 'zed2i_camera.yaml'),
        ),
        DeclareLaunchArgument(
            'visual_slam_config',
            default_value=os.path.join(package_share, 'config', 'visual_slam.yaml'),
        ),
        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'zed2i_isaac_vslam.rviz'),
        ),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('imu_frame', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
