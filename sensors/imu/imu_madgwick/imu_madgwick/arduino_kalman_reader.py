import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import serial.tools.list_ports
import time
import numpy as np
import math

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll, pitch, yaw (in degrees) to a quaternion."""
    # Convert from degrees to radians.
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

        # Flag to determine when calibration offsets have been set.
        self.offset_set = False

        # Lists for initial samples to compute offsets.
        self.orientation_init_samples = []
        self.accel_init_samples = []
        self.gyro_init_samples = []

        # Placeholders for the computed offsets.
        self.orientation_offsets = np.zeros(3)
        self.accel_offsets = np.zeros(3)
        self.gyro_offsets = np.zeros(3)

        self.filtered_yaw = None

    def find_arduino_serial(self):
        self.get_logger().info('Scanning for Arduino Nano 33 BLE...')
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if ((port.vid == 9025 and port.pid == 32858) or 'Nano 33 BLE' in port.product):
                try:
                    ser = serial.Serial(port.device, 115200, timeout=1)
                    time.sleep(2)  # Let it reset.
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
            # We expect nine data fields:
            # ax ay az gx gy gz roll pitch yaw
            if len(parts) != 9:
                return

            deg_to_rad = math.pi / 180.0

            # Read and scale linear acceleration. (ax, ay, az) in m/s²
            ax = float(parts[0]) * 9.81
            ay = float(parts[1]) * 9.81
            az = float(parts[2]) * 9.81

            # Read and convert angular velocities from deg/s to rad/s.
            gx = float(parts[3]) * deg_to_rad
            gy = float(parts[4]) * deg_to_rad
            gz = float(parts[5]) * deg_to_rad

            # Read roll, pitch, yaw; these are provided in degrees and converted to radians.
            roll = float(parts[6]) * deg_to_rad
            pitch = float(parts[7]) * deg_to_rad
            yaw = float(parts[8]) * deg_to_rad

            # Collect initial samples for calibration.
            if not self.offset_set:
                self.orientation_init_samples.append([roll, pitch, yaw])
                self.accel_init_samples.append([ax, ay, az])
                self.gyro_init_samples.append([gx, gy, gz])
                if len(self.orientation_init_samples) == 10:
                    self.orientation_offsets = np.mean(self.orientation_init_samples, axis=0)
                    self.accel_offsets = np.mean(self.accel_init_samples, axis=0)
                    self.gyro_offsets = np.mean(self.gyro_init_samples, axis=0)
                    self.offset_set = True
                    self.get_logger().info(
                        f'Calibration offsets set:\n'
                        f'Orientation: {self.orientation_offsets}\n'
                        f'Acceleration: {self.accel_offsets}\n'
                        f'Gyro: {self.gyro_offsets}'
                    )
                return

            # Subtract the offsets to get corrected readings.
            corrected_orientation = np.array([roll, pitch, yaw]) - self.orientation_offsets
            corrected_accel = np.array([ax, ay, az]) - self.accel_offsets
            corrected_gyro = np.array([gx, gy, gz]) - self.gyro_offsets

            # Apply a lowpass filter on the yaw of the orientation.
            yaw_lowpass_cutoff = 10.0  # 2.5 Hz cutoff frequency.
            RC = 1.0 / (2 * math.pi * yaw_lowpass_cutoff)
            dt = 0.025  # Timer period in seconds.
            alpha = dt / (RC + dt)
            
            if self.filtered_yaw is None:
                self.filtered_yaw = corrected_orientation[2]
            else:
                self.filtered_yaw += alpha * (corrected_orientation[2] - self.filtered_yaw)
            corrected_orientation[2] = self.filtered_yaw

            # Generate quaternion from the corrected orientation.
            # Note: rpy_to_quaternion expects its inputs in degrees.
            # If desired, you might convert the radians back to degrees before passing.
            quat = rpy_to_quaternion(
                corrected_orientation[0] * (180.0 / math.pi),
                corrected_orientation[1] * (180.0 / math.pi),
                corrected_orientation[2] * (180.0 / math.pi)
            )

            # Create and populate the IMU message.
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Use the corrected linear acceleration values.
            msg.linear_acceleration.x = corrected_accel[0]
            msg.linear_acceleration.y = corrected_accel[1]
            msg.linear_acceleration.z = corrected_accel[2]

            # Use the corrected angular velocity values.
            msg.angular_velocity.x = corrected_gyro[0]
            msg.angular_velocity.y = corrected_gyro[1]
            msg.angular_velocity.z = corrected_gyro[2]

            # Assign the orientation quaternion.
            msg.orientation = quat

            # Set covariance matrices.
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
