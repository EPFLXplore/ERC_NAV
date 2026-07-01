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
    # oak_calibration_depthai_<MXID>.json live next to aruco_params.yaml (installed under share/.../config)
    calib_dir = os.path.join(share, 'config')

    return LaunchDescription([

        # LogInfo(msg="Before aruco cpp in launch file..."),

        Node(
            package='ros2_aruco_cpp',
            executable='multiview_aruco_node',
            name='aruco_node',
            parameters=[config, {
                'calib_mode': 'file',
                'calib_dir': calib_dir,
                'cam_ids': ['19443010714B177E00', '19443010A19E157E00', '19443010816C177E00'],
                # Keep in sync with sensors/camera/.../camera_node_nav.launch.py Oak1W x,y
                'image_stream_width': 1280,
                'image_stream_height': 720,
            }],
            output='screen',
        ),

        # Replace pose_estimator_lidar_node with a fixed identity map->odom TF.
        # map -> odom: x y z yaw pitch roll = 0 0 0 0 0 0
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom_identity_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #     output='screen',
        # ),

        Node(
            package='ros2_aruco_cpp',
            executable='pose_estimator_lidar_node',
            name='pose_estimation_node_with_lidar',
            parameters=[config],
            output='screen',
        ),

    ])
