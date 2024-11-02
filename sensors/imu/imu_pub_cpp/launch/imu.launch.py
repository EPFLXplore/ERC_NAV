import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction



def generate_launch_description():

    imu_config_dir = os.path.join(get_package_share_directory('imu_pub_cpp'), 'config')

    return LaunchDescription([
        Node(   #custom imu raw data publisher
            package='imu_pub_cpp',
            executable='imu_pub_raw',
            name='imu_pub_raw',
            output='screen',
        ),

        TimerAction(
            period=4.0,  # starts later otherwise the filter can't get an initial reading
            actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter',
                    output='screen',
                    parameters=[os.path.join(imu_config_dir, 'imu_filter_params.yaml')],
                )
            ]
        ),

    # The imu_filter_madgwick package is used to filter and fuse raw data from IMU devices. 
    #It fuses angular velocities, accelerations, and (optionally) magnetic readings from 
    #a generic IMU device into an orientation quaternion, and publishes the fused data on the imu/data topic. 

#    ---- Subscribed Topics ----
#       imu/data_raw (sensor_msgs/Imu) : Message containing raw IMU data, including angular velocities and linear accelerations. 

#       imu/mag (sensor_msgs/MagneticField) : [optional] Magnetic field vector; the type is either sensor_msgs/MagneticField or geometry_msgs/Vector3Stamped (deprecated), depending on the parameter use_magnetic_field_msg (see below) 


# ---- Published Topics -----
#       imu/data (sensor_msgs/Imu)  : The fused Imu message, containing the orientation. 


    ])



