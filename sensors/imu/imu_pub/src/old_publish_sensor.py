import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
ser = serial.Serial('/dev/ttyACM0', 115200)


class MinimalPublisher(Node):

    pitch = float(0.0)
    yaw = float(0.0)
    roll = float(0.0)

    acc_x = float(0.0)
    acc_y = float(0.0)
    acc_z = float(0.0)

    mag_x = float(0.0)
    mag_y = float(0.0)
    mag_z = float(0.0)

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Imu, 'Imu', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        
        msg_raw = ser.readline().split(b' ')
        imu_msg = Imu()
        header  = Header(stamp = self.get_clock().now().to_msg(), frame_id = "av_imu")
        orientation_quat = Quaternion()
        
        if(msg_raw[0].lower() == b'imu'):
            #accelerometer
            self.acc_x = float(msg_raw[1])
            self.acc_y = float(msg_raw[2])
            self.acc_z = float(msg_raw[3])
            #gyro
            self.pitch = float(msg_raw[4])
            self.yaw = float(msg_raw[5])
            self.roll = float(msg_raw[6])
            #magnetometer
            self.mag_x = float(msg_raw[7])
            self.mag_y = float(msg_raw[8])
            self.mag_z = float(msg_raw[9]) 

        pitch = self.pitch
        yaw = self.yaw
        roll = self.roll

        acc_x = self.acc_x
        acc_y = self.acc_y
        acc_z = self.acc_z

        mag_x = self.mag_x    
        mag_y = self.mag_y    
        mag_z = self.mag_z    

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        orientation_quat.x = qx
        orientation_quat.y = qy
        orientation_quat.z = qz
        orientation_quat.w = qw

        # orientation_cov = [-1, 0, 0,
        #                     0, 0 ,0,
        #                     0, 0 ,0]
        # angular_cov = [0, 0, 0,
        #                0, 0 ,0,
        #                0, 0 ,0]
        # linear_cov = [0, 0, 0,
        #               0, 0, 0, 
        #               0, 0 ,0]

        # Supposed data "[verif_token], ax, ay, az, gx, ..., mx,..."

        angular_acc = Vector3(x=pitch, y=yaw, z=roll)
        linear_acc = Vector3(x=acc_x, y=acc_y, z=acc_z)


        imu_msg.header = header

        imu_msg.orientation = orientation_quat
        #imu[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu_msg.orientation_covariance = np.zeros(9, dtype=np.float64)

        imu_msg.angular_velocity = angular_acc
        imu_msg.angular_velocity_covariance = np.zeros(9, dtype=np.float64)
        
        imu_msg.linear_acceleration = linear_acc
        imu_msg.linear_acceleration_covariance = np.zeros(9, dtype=np.float64)
        
        self.publisher_.publish(imu_msg)
        self.i += 1


def main(args=None):
  rclpy.init(args=args)
  minimal_publisher = MinimalPublisher()
  rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()