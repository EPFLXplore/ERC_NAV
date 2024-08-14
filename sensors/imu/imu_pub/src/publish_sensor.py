import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from resource.madgwick_py.madgwickahrs import MadgwickAHRS





class MinimalPublisher(Node):


    def __init__(self):
        super().__init__('minimal_publisher')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.serial_port.flush()
        self.publisher_ = self.create_publisher(Imu, 'Imu', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.madgwick = MadgwickAHRS()
        self.i = 0
        self.marc = False

    def parse_data(self, data):
        parts = data.strip().split('\t')
        if len(parts) != 3:
            return None

        try:
            acc_data = parts[0].split(': ')[1].split(', ')
            gyro_data = parts[1].split(': ')[1].split(', ')
            mag_data = parts[2].split(': ')[1].split(', ')

            acc = list(map(float, acc_data))
            gyro = list(map(float, gyro_data))
            mag = list(map(float, mag_data))

            return {'acc': acc, 'gyro': gyro, 'mag': mag}
        except (IndexError, ValueError):
            return None

    def timer_callback(self):
        
        # Initialize message
        imu_msg = Imu()
        header  = Header(stamp = self.get_clock().now().to_msg(), frame_id = "av_imu")
        imu_msg.header = header

        # Read the IMU data
        if  self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            # self.get_logger().info(f"Raw data : {line}")

            data = self.parse_data(line)

            if data:
                # Accelerometer
                acc_x = data['acc'][0]
                acc_y = data['acc'][1]
                acc_z = data['acc'][2]
                # Gryoscope
                gyro_x = data['gyro'][0]
                gyro_y = data['gyro'][1]
                gyro_z = data['gyro'][2]
                # Magnetometer
                mag_x = data['mag'][0]
                mag_y = data['mag'][1]
                mag_z = data['mag'][2] 

                imu_msg.linear_acceleration = Vector3(x=acc_x, y=acc_y, z=acc_z)
                imu_msg.linear_acceleration_covariance = [0.0] * 9

                # Set angular velocity
                imu_msg.angular_velocity = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)
                imu_msg.angular_velocity_covariance = [0.0] * 9


                if self.marc:
                    qx = np.sin(gyro_x/2) * np.cos(gyro_y/2) * np.cos(gyro_z/2) - np.cos(gyro_x/2) * np.sin(gyro_y/2) * np.sin(gyro_z/2)
                    qy = np.cos(gyro_x/2) * np.sin(gyro_y/2) * np.cos(gyro_z/2) + np.sin(gyro_x/2) * np.cos(gyro_y/2) * np.sin(gyro_z/2)
                    qz = np.cos(gyro_x/2) * np.cos(gyro_y/2) * np.sin(gyro_z/2) - np.sin(gyro_x/2) * np.sin(gyro_y/2) * np.cos(gyro_z/2)
                    qw = np.cos(gyro_x/2) * np.cos(gyro_y/2) * np.cos(gyro_z/2) + np.sin(gyro_x/2) * np.sin(gyro_y/2) * np.sin(gyro_z/2)
            
                    imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
                
                else:
                    # Sensor fusion to get the orientation
                    self.madgwick.update(gyroscope=np.array([gyro_x, gyro_y, gyro_z]),
                                        accelerometer=np.array([acc_x, acc_y, acc_z]),
                                        magnetometer=np.array([mag_x, mag_y, mag_z]))
                    quat = self.madgwick.quaternion
                    # Set orientation
                    imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

                imu_msg.orientation_covariance = [0.0] * 9


                self.publisher_.publish(imu_msg)
                self.i += 1



def main(args=None):
  rclpy.init(args=args)
  minimal_publisher = MinimalPublisher()
  rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()