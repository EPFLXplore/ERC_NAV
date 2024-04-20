import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
ser = Serial.serial('dev/ttyACM0', 115200)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Imu, 'topic', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        
        msg_raw = ser.read().split(' ')
        imu_msg = Imu()
        header  = Header()
        orientation_quat = Quaternion()
        orientation_cov = [-1, 0, 0,
                            0, 0 ,0,
                            0, 0 ,0]
        angular_cov = [0, 0, 0,
                       0, 0 ,0,
                       0, 0 ,0]
        linear_cov = [0, 0, 0,
                      0, 0, 0, 
                      0, 0 ,0]

        # Supposed data "[verif_token], ax, ay, az, gx, ..., mx,..."
        if(msg_raw[0].lower() == 'imu'):
            acc_x = float(msg_raw[1])
            acc_y = float(msg_raw[2])
            acc_z = float(msg_raw[3])
            pitch = float(msg_raw[4])
            yaw = float(msg_raw[5])
            roll = float(msg_raw[6])
            mag_x = float(msg_raw[7])
            mag_y = float(msg_raw[8])
            mag_z = float(msg_raw[9]) 

        angular_acc = Vector3(pitch, yaw, roll)
        linear_acc = Vector3(acc_x, acc_y, acc_z)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
 
        imu_msg.header = header

        imu_msg.orientation = orientation_quat
        imu_msg.orientation_covariance = orientation_cov
        
        imu_msg.angular_velocity = angular_acc
        imu_msg.angular_velocity_covariance = angular_cov
        
        imu_msg.linear_acceleration = linear_acc
        imu_msg.linear_acceleration_covariance = linear_cov
        

        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
  rclpy.init(args=args)
  minimal_publisher = MinimalPublisher()
  rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()