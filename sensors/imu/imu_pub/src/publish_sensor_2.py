import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import time


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.publisher_ = self.create_publisher(Imu, 'Imu', 10)
        self.timer = self.create_timer(0.004, self.timer_callback)
        
        self.buffer = b''
        self.data_start_char = b'\n'  

        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration_covariance = [0.0] * 9
        self.imu_msg.angular_velocity_covariance = [0.0] * 9
        self.imu_msg.orientation_covariance = [-1., 0., 0.,
                                               0., 0., 0.,
                                               0., 0., 0.]

    def parse_data(self, data):
        parts = data.strip().split(', ')
        if len(parts) != 9:  # 3 acc + 3 gyro + 3 euler
            return None

        acc = list(map(float, parts[0:3]))
        gyro = list(map(float, parts[3:6]))
        euler = list(map(float, parts[6:9]))

        return {'acc': acc, 'gyro': gyro, 'euler': euler}

    def timer_callback(self):

        while self.serial_port.in_waiting > 0:
            byte = self.serial_port.read(1)
            if byte == self.data_start_char:
                line = self.buffer.decode('utf-8').strip()
                self.buffer = b''  # Clear buffer after processing

                try:
                    data = self.parse_data(line)
                    if data is None:
                        continue

                    acc_x, acc_y, acc_z = data['acc']
                    gyro_x, gyro_y, gyro_z = data['gyro']
                    roll, pitch, yaw = data['euler']

                    quat = get_quaternion_from_euler(roll, pitch, yaw)

                    # Fill the IMU message
                    self.imu_msg.linear_acceleration = Vector3(x=acc_x, y=acc_y, z=acc_z)
                    self.imu_msg.angular_velocity = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)
                    self.imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

                    # Update header timestamp
                    self.imu_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="lidar_link")

                    # Publish the message
                    self.publisher_.publish(self.imu_msg)
                except (IndexError, ValueError):
                    self.get_logger().warn("Failed to parse IMU data.")
            else:
                self.buffer += byte

        # self.get_logger().info(f"Callback duration: {end_time - start_time} seconds")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
