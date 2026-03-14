import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_local_share_dir():
    try:
        return get_package_share_directory('zed2i_native_vio')
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def launch_setup(context, *args, **kwargs):
    package_share = get_local_share_dir()
    camera_name = LaunchConfiguration('camera_name').perform(context)
    zed_node_name = LaunchConfiguration('zed_node_name').perform(context)
    use_vio_watcher = LaunchConfiguration('use_vio_watcher').perform(context).lower() in ('1', 'true', 'yes')
    pose_topic = LaunchConfiguration('native_pose_topic').perform(context) or f'/{camera_name}/{zed_node_name}/pose'
    offset_pose_topic = LaunchConfiguration('offset_pose_topic').perform(context) or f'/{camera_name}/{zed_node_name}/offset_pose'
    vio_watcher_reference_odom_topic = LaunchConfiguration('vio_watcher_reference_odom_topic').perform(context)
    set_pose_service = LaunchConfiguration('set_pose_service').perform(context) or f'/{camera_name}/{zed_node_name}/set_pose'

    local_package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    local_watcher_script = os.path.join(local_package_dir, 'scripts', 'native_vio_watcher')
    installed_watcher_available = False

    try:
        watcher_prefix = get_package_prefix('zed2i_native_vio')
        installed_watcher_path = os.path.join(watcher_prefix, 'lib', 'zed2i_native_vio', 'native_vio_watcher')
        installed_watcher_available = os.path.isfile(installed_watcher_path) and os.access(
            installed_watcher_path, os.X_OK
        )
    except PackageNotFoundError:
        installed_watcher_available = False

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_model': 'zed2i',
            'container_name': LaunchConfiguration('container_name'),
            'node_name': LaunchConfiguration('zed_node_name'),
            'serial_number': LaunchConfiguration('serial_number'),
            'ros_params_override_path': LaunchConfiguration('zed_wrapper_config'),
            'publish_urdf': LaunchConfiguration('publish_urdf'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_map_tf': LaunchConfiguration('publish_map_tf'),
            'publish_imu_tf': LaunchConfiguration('publish_imu_tf'),
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    launch_actions = [zed_wrapper_launch, rviz_node]

    if use_vio_watcher:
        if installed_watcher_available:
            launch_actions.append(
                Node(
                    package='zed2i_native_vio',
                    executable='native_vio_watcher',
                    name='native_vio_watcher',
                    output='screen',
                    emulate_tty=True,
                    additional_env={'RCUTILS_COLORIZED_OUTPUT': '1'},
                    parameters=[{
                        'pose_topic': pose_topic,
                        'offset_pose_topic': offset_pose_topic,
                        'reference_odom_topic': LaunchConfiguration('vio_watcher_reference_odom_topic'),
                        'set_pose_service': set_pose_service,
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
                        'max_pose_frame_gap_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_max_pose_frame_gap_sec'), value_type=float
                        ),
                        'set_pose_retry_count': ParameterValue(
                            LaunchConfiguration('vio_watcher_set_pose_retry_count'), value_type=int
                        ),
                        'set_pose_retry_delay_sec': ParameterValue(
                            LaunchConfiguration('vio_watcher_set_pose_retry_delay_sec'), value_type=float
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
                LogInfo(msg='Using source `native_vio_watcher` script because the installed executable is missing.'),
                ExecuteProcess(
                    cmd=[
                        'python3',
                        local_watcher_script,
                        '--ros-args',
                        '-r',
                        '__node:=native_vio_watcher',
                        '-p',
                        f'pose_topic:={pose_topic}',
                        '-p',
                        f'offset_pose_topic:={offset_pose_topic}',
                        '-p',
                        f'reference_odom_topic:={vio_watcher_reference_odom_topic}',
                        '-p',
                        f'set_pose_service:={set_pose_service}',
                        '-p',
                        f'jump_threshold_m:={LaunchConfiguration("vio_watcher_jump_threshold_m").perform(context)}',
                        '-p',
                        f'jump_frequency_window_sec:={LaunchConfiguration("vio_watcher_jump_frequency_window_sec").perform(context)}',
                        '-p',
                        f'jump_frequency_reset_count:={LaunchConfiguration("vio_watcher_jump_frequency_reset_count").perform(context)}',
                        '-p',
                        f'reset_cooldown_sec:={LaunchConfiguration("vio_watcher_reset_cooldown_sec").perform(context)}',
                        '-p',
                        f'max_pose_frame_gap_sec:={LaunchConfiguration("vio_watcher_max_pose_frame_gap_sec").perform(context)}',
                        '-p',
                        f'set_pose_retry_count:={LaunchConfiguration("vio_watcher_set_pose_retry_count").perform(context)}',
                        '-p',
                        f'set_pose_retry_delay_sec:={LaunchConfiguration("vio_watcher_set_pose_retry_delay_sec").perform(context)}',
                        '-p',
                        f'post_reset_check_delay_sec:={LaunchConfiguration("vio_watcher_post_reset_check_delay_sec").perform(context)}',
                        '-p',
                        f'post_reset_max_distance_m:={LaunchConfiguration("vio_watcher_post_reset_max_distance_m").perform(context)}',
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
        DeclareLaunchArgument('container_name', default_value=''),
        DeclareLaunchArgument(
            'zed_wrapper_config',
            default_value=os.path.join(package_share, 'config', 'zed2i_native_vio.yaml'),
        ),
        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='false'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_vio_watcher', default_value='false'),
        DeclareLaunchArgument('native_pose_topic', default_value=''),
        DeclareLaunchArgument('offset_pose_topic', default_value=''),
        DeclareLaunchArgument('vio_watcher_reference_odom_topic', default_value='/fused_nav_ekf_odom'),
        DeclareLaunchArgument('set_pose_service', default_value=''),
        DeclareLaunchArgument('vio_watcher_jump_threshold_m', default_value='0.1'),
        DeclareLaunchArgument('vio_watcher_jump_frequency_window_sec', default_value='5.0'),
        DeclareLaunchArgument('vio_watcher_jump_frequency_reset_count', default_value='1'),
        DeclareLaunchArgument('vio_watcher_reset_cooldown_sec', default_value='0.0'),
        DeclareLaunchArgument('vio_watcher_max_pose_frame_gap_sec', default_value='0.5'),
        DeclareLaunchArgument('vio_watcher_set_pose_retry_count', default_value='10'),
        DeclareLaunchArgument('vio_watcher_set_pose_retry_delay_sec', default_value='0.1'),
        DeclareLaunchArgument('vio_watcher_post_reset_check_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('vio_watcher_post_reset_max_distance_m', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
