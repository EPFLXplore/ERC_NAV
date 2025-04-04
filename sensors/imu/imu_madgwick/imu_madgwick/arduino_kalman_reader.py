import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import serial.tools.list_ports
import time
import numpy as np
import re
import math

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw (in degrees) to a quaternion."""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_imu_node')

        self.publisher_ = self.create_publisher(Imu, '/imu_nano', 10)
        self.timer_ = self.create_timer(0.025, self.read_serial_data)
        self.serial_conn = self.find_arduino_serial()
        self.offset_set = False
        self.init_samples = []
        self.offsets = np.zeros(3)

    def find_arduino_serial(self):
        self.get_logger().info('Scanning for Arduino Nano 33 BLE...')
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if (port.vid == 9025 and port.pid == 32858 and port.product and 'Nano 33 BLE' in port.product):
                try:
                    ser = serial.Serial(port.device, 115200, timeout=1)
                    time.sleep(2)  # Let it reset
                    ser.reset_input_buffer()
                    self.get_logger().info(f"Connected to {port.device}")
                    return ser
                except Exception as e:
                    self.get_logger().warn(f"Failed to connect to {port.device}: {e}")
                    continue

        self.get_logger().error('Could not find a valid Arduino Nano 33 BLE serial port.')
        exit(1)


    def read_serial_data(self):
        if not self.serial_conn.in_waiting:
            return

        try:
            line = self.serial_conn.readline().decode('utf-8').strip()
            parts = line.split()
            if len(parts) != 9:
                return

            ax = float(parts[0])
            ay = float(parts[1])
            az = float(parts[2])

            gx = float(parts[3])
            gy = float(parts[4])
            gz = float(parts[5])

            roll = float(parts[6])
            pitch = float(parts[7])
            yaw = float(parts[8])

            # self.get_logger().info(f'ax: {ax}, ay : {ay}, az : {az}')
            # self.get_logger().info(f'gx: {gx}, gy : {gy}, gz : {gz}')
            # self.get_logger().info(f'Roll: {roll}, Pitch : {pitch}, Yaw : {yaw}')

            #self.get_logger().info(f'Roll: {roll}, Pitch : {pitch}, Yaw : {yaw}')

            if not self.offset_set:
                self.init_samples.append([roll, pitch, yaw])
                if len(self.init_samples) == 10:
                    self.offsets = np.mean(self.init_samples, axis=0)
                    self.offset_set = True
                    self.get_logger().info(f'Offsets set to: {self.offsets}')
                return

            corrected = np.array([roll, pitch, yaw]) - self.offsets
            #self.get_logger().info(f'Roll: {corrected[0]}, Pitch : {corrected[1]}, Yaw : {corrected[2]}')

            quat = rpy_to_quaternion(*corrected)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation = quat

            msg.orientation_covariance = [
                0.03, 0.0, 0.0,
                0.0, 0.03, 0.0,
                0.0, 0.0, 0.03
            ]
            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.025
            ]
            msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04
            ]
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error parsing line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
