#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
class ImuCovarianceModifier(Node):
    def __init__(self):
        super().__init__('imu_covariance_modifier')
        qos_profile = qos_profile_sensor_data        
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(
            Imu,
            'imu/modified',
            qos_profile
        )
        self.get_logger().info("IMU Covariance Modifier Node has been started.")

    def imu_callback(self, msg: Imu):
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.orientation = msg.orientation
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.linear_acceleration = msg.linear_acceleration

        # Orientation covariance: roll (rotation around x), pitch(rot. around y), yaw(rot. around z)
        new_msg.orientation_covariance = [
            0.2, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        new_msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        new_msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceModifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
