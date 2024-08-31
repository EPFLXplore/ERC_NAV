import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from resource.madgwick_py.madgwickahrs import MadgwickAHRS
import time
import threading


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu__publisher')

        self.serial_port = serial.Serial('/dev/ttyUSB0', 921600, timeout=1)
        self.serial_port.flush()
        self.serial_port.write("monitor enable imu\n".encode())

        self.publisher_ = self.create_publisher(Imu, 'Imu', 10)
        timer_period = 0.005 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.imu_msg = Imu()
        
        self.imu_msg.linear_acceleration_covariance = [0.0] * 9
        self.imu_msg.angular_velocity_covariance = [0.0] * 9
        self.imu_msg.orientation_covariance = [-1., 0., 0.,
                                                0., 0. ,0.,
                                                    0., 0. ,0.]
        self.last_acc = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_gyro = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.lock = threading.Lock()
        self.read_thread = threading.Thread(target=self.serial_reading_task)
        self.read_thread.daemon = True  # Daemonize the thread so it exits when the main program exits
        self.read_thread.start()

    def serial_reading_task(self):
        """Thread function to continuously read data from the serial port."""
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip('[').strip(']').split(":")

                try:
                    acc_data = line[1].split('\t')[0].split()
                    gyro_data = line[2].split('\t')[0].split()
                    quat_data = line[3].split()

                    # self.get_logger().info(f"acc:{float(acc_data[0].strip('['))}")
                    # self.get_logger().info(f"gyro:{gyro_data}")
                    # self.get_logger().info(f"quat:{quat_data}")

                    with self.lock:
                        self.last_acc = Vector3(x=float(acc_data[0].strip('[')), y=float(acc_data[1]), z=float(acc_data[2].strip(']')))
                        self.last_gyro = Vector3(x=float(gyro_data[0].strip('[')), y=float(gyro_data[1]), z=float(gyro_data[2].strip(']')))
                        self.last_quat = Quaternion(x=float(quat_data[0].strip('[')), y=float(quat_data[1]), z=float(quat_data[2]), w=float(quat_data[3].strip(']')))
                    
                except (IndexError, ValueError):
                    self.get_logger().warn("Failed to parse IMU data.")
                    return 

    def timer_callback(self):
        # start_time = time.time()
        header  = Header(stamp = self.get_clock().now().to_msg(), frame_id = "lidar_link")
        self.imu_msg.header = header

        with self.lock:
            self.imu_msg.linear_acceleration = self.last_acc
            self.imu_msg.angular_velocity = self.last_gyro
            self.imu_msg.orientation = self.last_quat

        self.publisher_.publish(self.imu_msg)
        self.i += 1
        # end_time = time.time() 
        # self.get_logger().info(f"Callback duration: {end_time - start_time} seconds")

    def destroy_node(self):
        self.get_logger().info('Shutting down node, closing serial port...')
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  imu__publisher = ImuPublisher()
  rclpy.spin(imu__publisher)
  imu__publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()