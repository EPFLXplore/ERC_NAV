import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('imu_pub_cpp'), 'config')

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
            )
        ]
    )

    # The imu_filter_madgwick package is used to filter and fuse raw data from IMU devices. 
    #It fuses angular velocities, accelerations, and (optionally) magnetic readings from 
    #a generic IMU device into an orientation quaternion, and publishes the fused data on the imu/data topic. 

#    ---- Subscribed Topics ----
#       imu/data_raw (sensor_msgs/Imu)

#     Message containing raw IMU data, including angular velocities and linear accelerations. 

# imu/mag (sensor_msgs/MagneticField)

#     [optional] Magnetic field vector; the type is either sensor_msgs/MagneticField or geometry_msgs/Vector3Stamped (deprecated), depending on the parameter use_magnetic_field_msg (see below) 


# ---- Published Topics -----
#   imu/data (sensor_msgs/Imu)

#     The fused Imu message, containing the orientation. 
