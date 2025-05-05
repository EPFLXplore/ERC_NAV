import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from custom_msg.msg import MotorStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import numpy as np
import math
from collections import deque

from message_filters import ApproximateTimeSynchronizer, Subscriber

WHEEL_RADIUS = 0.12  # meters
GEAR_RATIO = 1.0 / 53.0
RPM_TO_MS = (2 * math.pi * WHEEL_RADIUS) / 60.0
INCR_TO_RAD = 2 * math.pi / (2 ** 14)  # 14-bit encoder
ANGLE_THRESHOLD_RAD = 0.5 * math.pi / 180.0
ROTATION_ANGLE_THRESHOLD = 0.642  # 36.6 degrees in rad
SPEED_EPSILON = 0.02  # m/s
LENGTH = 0.68  # distance between front and back axles
WIDTH = 0.52   # distance between left and right wheels

def quaternion_to_yaw(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class OdomPreprocessor(Node):
    def __init__(self):
        super().__init__('odom_preprocessor')
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.declare_parameter('alpha_gyro', 0.5)
        self.declare_parameter('beta_yaw_fusion', 0.6)
        self.declare_parameter('yaw_avg_window', 5)
        self.declare_parameter('startup_delay_sec', 5.0)
        self.declare_parameter('publish_rate_hz', 80.0)

        self.alpha_gyro = self.get_parameter('alpha_gyro').value
        self.beta_yaw = self.get_parameter('beta_yaw_fusion').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.startup_delay = self.get_parameter('startup_delay_sec').value
        self.yaw_window = deque(maxlen=self.get_parameter('yaw_avg_window').value)

        self.yaw = 0.0
        self.last_time = None
        self.first_imu_time = None

        self.nano_yaw = None
        self.nano_gyro_z = None
        self.ouster_gyro_z = None

        self.wheel_speeds = [0.0] * 4
        self.wheel_angles = [0.0] * 4
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.latest_vx = 0.0
        self.latest_vy = 0.0

        self.imu_nano_sub = Subscriber(self, Imu, '/imu_nano', qos_profile=qos_sensor)
        self.imu_ouster_sub = Subscriber(self, Imu, '/imu/modified', qos_profile=qos_sensor)    
        self.ts = ApproximateTimeSynchronizer([self.imu_nano_sub, self.imu_ouster_sub], queue_size=10, slop=0.015)
        self.ts.registerCallback(self.synced_imu_callback)

        self.create_subscription(MotorStatus, '/NAV/motor_nav_status', self.motor_callback, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom_processed', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)

        self.get_logger().info('OdomPreprocessor started.')

    def synced_imu_callback(self, nano_msg, ouster_msg):
        nano_time = nano_msg.header.stamp.sec + nano_msg.header.stamp.nanosec * 1e-9
        ouster_time = ouster_msg.header.stamp.sec + ouster_msg.header.stamp.nanosec * 1e-9
        time_diff = abs(nano_time - ouster_time)
        #self.get_logger().info(f"IMU sync delay: {time_diff*1000:.2f} ms")

        self.nano_gyro_z = nano_msg.angular_velocity.z
        self.nano_yaw = quaternion_to_yaw(nano_msg.orientation.x, nano_msg.orientation.y, nano_msg.orientation.z, nano_msg.orientation.w)
        self.ouster_gyro_z = ouster_msg.angular_velocity.z

        if self.first_imu_time is None:
            self.first_imu_time = self.get_clock().now()

    def motor_callback(self, msg: MotorStatus):
        if len(msg.velocity) != 4 or len(msg.position) != 4:
            return

        for i in range(4):
            self.wheel_speeds[i] = msg.velocity[i] * GEAR_RATIO * RPM_TO_MS * (-1.0 if i in [1, 2] else 1.0)
            angle = msg.position[i] * INCR_TO_RAD
            self.wheel_angles[i] = (angle + math.pi) % (2 * math.pi) - math.pi

    def wheel_odom_callback(self, msg: Odometry):
        self.latest_vx = msg.twist.twist.linear.x
        self.latest_vy = msg.twist.twist.linear.y

    def update(self):
        if None in (self.nano_yaw, self.nano_gyro_z, self.ouster_gyro_z) or self.first_imu_time is None:
            return

        now = self.get_clock().now()
        if (now - self.first_imu_time).nanoseconds * 1e-9 < self.startup_delay:
            return

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        avg_angle = sum(abs(a) for a in self.wheel_angles) / 4.0
        alpha = max(0.0, min(1.0, self.alpha_gyro * (avg_angle / (math.pi / 4.0))))

        if avg_angle < ANGLE_THRESHOLD_RAD:
            fused_yaw = self.yaw
            #self.get_logger().info('Wheels straight ,yaw static')

        else:
            gyro_z = alpha * self.ouster_gyro_z + (1 - alpha) * self.nano_gyro_z
            predicted_yaw = self.yaw + gyro_z * dt
            predicted_yaw = self.wrap_to_pi(predicted_yaw)
            self.get_logger().info(f'gyro yaw: {predicted_yaw}')

            fused_yaw = self.beta_yaw * self.nano_yaw + (1 - self.beta_yaw) * predicted_yaw
            fused_yaw = self.wrap_to_pi(fused_yaw)

        self.yaw_window.append(fused_yaw)
        self.yaw = self.wrap_to_pi(sum(self.yaw_window) / len(self.yaw_window))
        #self.get_logger().info(f'fused yaw: {self.yaw}')

        dx = self.latest_vx * math.cos(self.yaw) * dt - self.latest_vy * math.sin(self.yaw) * dt
        dy = self.latest_vx * math.sin(self.yaw) * dt + self.latest_vy * math.cos(self.yaw) * dt

        self.pos_x += dx
        self.pos_y += dy

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.pos_x
        msg.pose.pose.position.y = self.pos_y
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = self.latest_vx
        msg.twist.twist.linear.y = self.latest_vy
        msg.twist.twist.angular.z = 0.0

        msg.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.2]

        msg.twist.covariance = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]


        self.odom_pub.publish(msg)

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = OdomPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
