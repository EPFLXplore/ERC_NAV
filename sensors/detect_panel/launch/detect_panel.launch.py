from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    ld = LaunchDescription()
    detect_node = Node(
        package="detect_panel",
        executable="NAV_detect_panel_node",
    )

    
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix  = LaunchConfiguration('tf_prefix',   default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '0.0')

    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True) # keep it at True
    extended       = LaunchConfiguration('extended', default = True) # keep it at False
    subpixel       = LaunchConfiguration('subpixel', default = False)
    confidence     = LaunchConfiguration('confidence', default = 130)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 20)
    monoResolution = LaunchConfiguration('monoResolution',  default = '720p')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')

    stereo_node = launch_ros.actions.Node(
            package='front_camera', executable='front_camera_publisher',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'mode': mode},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'monoResolution': monoResolution}])


    # metric_converter_node = launch_ros.actions.ComposableNodeContainer(
    #         name='container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             # Driver itself
    #             launch_ros.descriptions.ComposableNode(
    #                 package='depth_image_proc',
    #                 plugin='depth_image_proc::ConvertMetricNode',
    #                 name='convert_metric_node',
    #                 remappings=[('image_raw', '/stereo/depth'),
    #                             ('camera_info', '/stereo/camera_info'),
    #                             ('image', '/stereo/converted_depth')]
    #             ),
    #         ],
    #         output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container2',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',

                    remappings=[('image_rect', '/stereo/depth'), # used to be /stereo/converted_depth, replaced by /filtered_depth
                                ('camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)

    ld.add_action(stereo_node)
    # ld.add_action(urdf_launch)

    # ld.add_action(metric_converter_node)
    # ld.add_action(point_cloud_node_orig)
    ld.add_action(point_cloud_node)
    # ld.add_action(rviz_node)
    ld.add_action(detect_node)
    return ld