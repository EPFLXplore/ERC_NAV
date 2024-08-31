import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import time
import threading


DEG_TO_RAD = np.pi/180

def get_quaternion_from_euler(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class MinimalPublisher(Node):


    def __init__(self):
        super().__init__('minimal_publisher')
       
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.serial_port.flush()

        self.publisher_ = self.create_publisher(Imu, 'Imu', 10)
        timer_period = 0.004  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        self.i = 0

        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration_covariance = [0.0] * 9
        self.imu_msg.angular_velocity_covariance = [0.0] * 9
        self.imu_msg.orientation_covariance = [0., 0., 0.,
                                                0., 0. ,0.,
                                                    0., 0. ,0.]

        self.last_acc = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_gyro = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.lock = threading.Lock()
        self.read_thread = threading.Thread(target=self.serial_reading_task)
        self.read_thread.daemon = True  # Daemonize the thread so it exits when the main program exits
        self.read_thread.start()

    def parse_data(self, data):
        parts = data.strip().split(', ')
        if len(parts) != 9:  # 3 acc + 3 gyro + Euler orientation
            return None

        acc = list(map(float, parts[0:3]))
        gyro = list(map(float, parts[3:6]))
        euler = list(map(float, parts[6:9]))

        return {'acc': acc, 'gyro': gyro, 'euler': euler}
        

    def serial_reading_task(self):
        """Thread function to continuously read data from the serial port."""
        while rclpy.ok():

            # Read the IMU data
            if  self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()

                try:
                    data = self.parse_data(line)
                    
                    # Accelerometer
                    acc_x = data['acc'][0]
                    acc_y = data['acc'][1]
                    acc_z = data['acc'][2]
                    # Gryoscope
                    gyro_x = data['gyro'][0]
                    gyro_y = data['gyro'][1]
                    gyro_z = data['gyro'][2]
                    # Orientation (euler)
                    roll = data['euler'][0] * DEG_TO_RAD
                    pitch = data['euler'][1] * DEG_TO_RAD
                    yaw = data['euler'][2] * DEG_TO_RAD

                    quat = get_quaternion_from_euler(roll, pitch, yaw)
    
                    with self.lock:
                        self.last_acc = Vector3(x=acc_x, y=acc_y, z=acc_z)
                        self.last_gyro = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)
                        self.last_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    
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



def main(args=None):
  rclpy.init(args=args)
  minimal_publisher = MinimalPublisher()
  rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()