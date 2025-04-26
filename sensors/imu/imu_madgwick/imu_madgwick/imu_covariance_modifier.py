#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import time

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
        
        self.publisher_ = self.create_publisher(
            Imu,
            'imu/modified',
            qos_profile
        )

        self.get_logger().info("IMU Covariance Modifier Node has been started.")

        # Variables to collect samples
        self.collecting_offsets = True
        self.start_time = self.get_clock().now()
        self.acc_samples = []
        self.gyro_samples = []
        self.acc_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds

        if self.collecting_offsets:
            self.acc_samples.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            self.gyro_samples.append([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            if elapsed >= 3.0:
                # Compute offsets
                acc_sum = [sum(x) for x in zip(*self.acc_samples)]
                gyro_sum = [sum(x) for x in zip(*self.gyro_samples)]
                n = len(self.acc_samples)
                self.acc_offset = [s / n for s in acc_sum]
                self.gyro_offset = [s / n for s in gyro_sum]
                self.collecting_offsets = False
                self.get_logger().info(f"Collected IMU offsets. Acc: {self.acc_offset}, Gyro: {self.gyro_offset}")
            return  # Don't publish during offset collection

        # Correct incoming IMU data
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.orientation = msg.orientation  # Keeping orientation uncorrected

        # Subtract the offset
        new_msg.angular_velocity.x = msg.angular_velocity.x - self.gyro_offset[0]
        new_msg.angular_velocity.y = msg.angular_velocity.y - self.gyro_offset[1]
        new_msg.angular_velocity.z = msg.angular_velocity.z - self.gyro_offset[2]

        new_msg.linear_acceleration.x = msg.linear_acceleration.x - self.acc_offset[0]
        new_msg.linear_acceleration.y = msg.linear_acceleration.y - self.acc_offset[1]
        new_msg.linear_acceleration.z = msg.linear_acceleration.z - self.acc_offset[2] + 9.77

        # Set covariances
        new_msg.orientation_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.041
        ]
        new_msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.025
        ]
        new_msg.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
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
