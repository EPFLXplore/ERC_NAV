import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def get_local_share_dir():
    try:
        return get_package_share_directory('zed2i_isaac_vslam')
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial_number = LaunchConfiguration('serial_number').perform(context)
    zed_node_name = LaunchConfiguration('zed_node_name').perform(context)
    zed_wrapper_config = LaunchConfiguration('zed_wrapper_config').perform(context)
    visual_slam_config = LaunchConfiguration('visual_slam_config').perform(context)
    container_name = LaunchConfiguration('container_name').perform(context)
    publish_urdf = LaunchConfiguration('publish_urdf').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    launch_zed_wrapper = LaunchConfiguration('launch_zed_wrapper').perform(context).lower() in ('1', 'true', 'yes')
    use_vio_watcher = LaunchConfiguration('use_vio_watcher').perform(context).lower() in ('1', 'true', 'yes')
    vslam_vo_pose_topic = LaunchConfiguration('vslam_vo_pose_topic').perform(context)
    offset_vo_pose_topic = LaunchConfiguration('offset_vo_pose_topic').perform(context)
    vio_watcher_reference_odom_topic = LaunchConfiguration('vio_watcher_reference_odom_topic').perform(context)
    vslam_reset_service = LaunchConfiguration('vslam_reset_service').perform(context)
    vslam_set_slam_pose_service = LaunchConfiguration('vslam_set_slam_pose_service').perform(context)
    vio_watcher_jump_threshold_m = LaunchConfiguration('vio_watcher_jump_threshold_m').perform(context)
    vio_watcher_jump_frequency_window_sec = LaunchConfiguration(
        'vio_watcher_jump_frequency_window_sec'
    ).perform(context)
    vio_watcher_jump_frequency_reset_count = LaunchConfiguration(
        'vio_watcher_jump_frequency_reset_count'
    ).perform(context)
    vio_watcher_reset_cooldown_sec = LaunchConfiguration('vio_watcher_reset_cooldown_sec').perform(context)
    vio_watcher_set_slam_pose_retry_count = LaunchConfiguration(
        'vio_watcher_set_slam_pose_retry_count'
    ).perform(context)
    vio_watcher_set_slam_pose_retry_delay_sec = LaunchConfiguration(
        'vio_watcher_set_slam_pose_retry_delay_sec'
    ).perform(context)
    vio_watcher_post_reset_check_delay_sec = LaunchConfiguration(
        'vio_watcher_post_reset_check_delay_sec'
    ).perform(context)
    vio_watcher_post_reset_max_distance_m = LaunchConfiguration(
        'vio_watcher_post_reset_max_distance_m'
    ).perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or f'{camera_name}_camera_center'
    imu_frame = LaunchConfiguration('imu_frame').perform(context) or f'{camera_name}_imu_link'

    left_optical_frame = f'{camera_name}_left_camera_optical_frame'
    right_optical_frame = f'{camera_name}_right_camera_optical_frame'
    zed_topic_root = f'/{camera_name}/{zed_node_name}'
    local_package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    local_watcher_script = os.path.join(local_package_dir, 'scripts', 'vio_watcher')
    installed_watcher_available = False

    try:
        watcher_prefix = get_package_prefix('zed2i_isaac_vslam')
        installed_watcher_path = os.path.join(watcher_prefix, 'lib', 'zed2i_isaac_vslam', 'vio_watcher')
        installed_watcher_available = os.path.isfile(installed_watcher_path) and os.access(
            installed_watcher_path, os.X_OK
        )
    except PackageNotFoundError:
        installed_watcher_available = False

    zed_wrapper_launch = None
    if launch_zed_wrapper:
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

    launch_actions = [visual_slam_container, rviz_node]
    if zed_wrapper_launch is not None:
        launch_actions.insert(0, zed_wrapper_launch)

    if use_vio_watcher:
        if installed_watcher_available:
            launch_actions.append(
                Node(
                    package='zed2i_isaac_vslam',
                    executable='vio_watcher',
                    name='vio_watcher',
                    output='screen',
                    emulate_tty=True,
                    additional_env={'RCUTILS_COLORIZED_OUTPUT': '1'},
                    parameters=[{
                        'vslam_vo_pose_topic': LaunchConfiguration('vslam_vo_pose_topic'),
                        'offset_vo_pose_topic': LaunchConfiguration('offset_vo_pose_topic'),
                        'reference_odom_topic': LaunchConfiguration('vio_watcher_reference_odom_topic'),
                        'reset_service': LaunchConfiguration('vslam_reset_service'),
                        'set_slam_pose_service': LaunchConfiguration('vslam_set_slam_pose_service'),
                        'jump_threshold_m': ParameterValue(
                            LaunchConfiguration('vio_watcher_jump_threshold_m'), value_type=float
                        ),
                        'jump_frequency_window_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_jump_frequency_window_sec'), value_type=float
                        ),
                        'jump_frequency_reset_count': ParameterValue(
                            LaunchConfiguration('vio_watcher_jump_frequency_reset_count'), value_type=int
                        ),
                        'reset_cooldown_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_reset_cooldown_sec'), value_type=float
                        ),
                        'set_slam_pose_retry_count': ParameterValue(
                            LaunchConfiguration('vio_watcher_set_slam_pose_retry_count'), value_type=int
                        ),
                        'set_slam_pose_retry_delay_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_set_slam_pose_retry_delay_sec'), value_type=float
                        ),
                        'post_reset_check_delay_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_post_reset_check_delay_sec'), value_type=float
                        ),
                        'post_reset_max_distance_m': ParameterValue(
                            LaunchConfiguration('vio_watcher_post_reset_max_distance_m'), value_type=float
                        ),
                    }],
                )
            )
        elif os.path.isfile(local_watcher_script):
            launch_actions.extend([
                LogInfo(msg='Using source `vio_watcher` script because the installed executable is missing.'),
                ExecuteProcess(
                    cmd=[
                        'python3',
                        local_watcher_script,
                        '--ros-args',
                        '-r',
                        '__node:=vio_watcher',
                        '-p',
                        f'vslam_vo_pose_topic:={vslam_vo_pose_topic}',
                        '-p',
                        f'offset_vo_pose_topic:={offset_vo_pose_topic}',
                        '-p',
                        f'reference_odom_topic:={vio_watcher_reference_odom_topic}',
                        '-p',
                        f'reset_service:={vslam_reset_service}',
                        '-p',
                        f'set_slam_pose_service:={vslam_set_slam_pose_service}',
                        '-p',
                        f'jump_threshold_m:={vio_watcher_jump_threshold_m}',
                        '-p',
                        f'jump_frequency_window_sec:={vio_watcher_jump_frequency_window_sec}',
                        '-p',
                        f'jump_frequency_reset_count:={vio_watcher_jump_frequency_reset_count}',
                        '-p',
                        f'reset_cooldown_sec:={vio_watcher_reset_cooldown_sec}',
                        '-p',
                        f'set_slam_pose_retry_count:={vio_watcher_set_slam_pose_retry_count}',
                        '-p',
                        f'set_slam_pose_retry_delay_sec:={vio_watcher_set_slam_pose_retry_delay_sec}',
                        '-p',
                        f'post_reset_check_delay_sec:={vio_watcher_post_reset_check_delay_sec}',
                        '-p',
                        f'post_reset_max_distance_m:={vio_watcher_post_reset_max_distance_m}',
                    ],
                    output='screen',
                    emulate_tty=True,
                    additional_env={'RCUTILS_COLORIZED_OUTPUT': '1'},
                ),
            ])
        else:
            raise RuntimeError(
                'use_vio_watcher:=true was requested, but no installed executable or source script was found.'
            )

    return launch_actions


def generate_launch_description():
    package_share = get_local_share_dir()

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
        DeclareLaunchArgument('launch_zed_wrapper', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_vio_watcher', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'zed2i_isaac_vslam.rviz'),
        ),
        DeclareLaunchArgument('vslam_vo_pose_topic', default_value='/visual_slam/tracking/vo_pose'),
        DeclareLaunchArgument('offset_vo_pose_topic', default_value='/visual_slam/tracking/offset_vo_pose'),
        DeclareLaunchArgument('vio_watcher_reference_odom_topic', default_value='/fused_nav_ekf_odom'),
        DeclareLaunchArgument('vslam_reset_service', default_value='/visual_slam/reset'),
        DeclareLaunchArgument('vslam_set_slam_pose_service', default_value='/visual_slam/set_slam_pose'),
        DeclareLaunchArgument('vio_watcher_jump_threshold_m', default_value='0.2'),
        DeclareLaunchArgument('vio_watcher_jump_frequency_window_sec', default_value='5.0'),
        DeclareLaunchArgument('vio_watcher_jump_frequency_reset_count', default_value='1'),
        DeclareLaunchArgument('vio_watcher_reset_cooldown_sec', default_value='0.0'),
        DeclareLaunchArgument('vio_watcher_set_slam_pose_retry_count', default_value='10'),
        DeclareLaunchArgument('vio_watcher_set_slam_pose_retry_delay_sec', default_value='0.1'),
        DeclareLaunchArgument('vio_watcher_post_reset_check_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('vio_watcher_post_reset_max_distance_m', default_value='1.0'),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('imu_frame', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
