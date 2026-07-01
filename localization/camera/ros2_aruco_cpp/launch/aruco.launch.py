import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _aruco_params_yaml_path():
    """Resolve aruco_params.yaml: install share, or repo config/ (symlink-install)."""
    share = get_package_share_directory('ros2_aruco_cpp')
    in_install = os.path.join(share, 'config', 'aruco_params.yaml')
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    next_to_launch = os.path.normpath(os.path.join(launch_dir, '..', 'config', 'aruco_params.yaml'))
    for p in (in_install, next_to_launch):
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        'aruco_params.yaml not found. Tried:\n'
        f'  {in_install}\n'
        f'  {next_to_launch}\n'
        'Rebuild and source: colcon build --packages-select ros2_aruco_cpp && source install/setup.bash'
    )


def generate_launch_description():
    config = _aruco_params_yaml_path()
    share = get_package_share_directory('ros2_aruco_cpp')
    # Per-camera config JSON files live next to aruco_params.yaml (installed under share/.../config)
    camera_config_dir = os.path.join(share, 'config')

    return LaunchDescription([

        # LogInfo(msg="Before aruco cpp in launch file..."),

        Node(
            package='ros2_aruco_cpp',
            executable='multiview_aruco_node',
            name='aruco_node',
            parameters=[config, {
                'camera_config_dir': camera_config_dir,
                # camera_config_files (which JSONs to load) comes from aruco_params.yaml.
            }],
        ),

        Node(
            package='ros2_aruco_cpp',
            executable='pose_estimator_lidar_node',
            name='pose_estimation_node_with_lidar',
        ),

        # WARNING BAD FOR NAV TASK, NEED TO INITITALIZE WITH ARUCOS!!!! 
        # map -> odom: x y z yaw pitch roll = 0 0 0 0 0 0
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom_identity_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     output='screen',
        # ),

    ])
