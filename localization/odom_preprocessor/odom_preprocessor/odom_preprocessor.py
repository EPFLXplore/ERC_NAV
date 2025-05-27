#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from custom_msg.msg import MotorStatus

import math
from collections import deque

# Constants
WHEEL_RADIUS = 0.12  # meters
GEAR_RATIO = 1.0 / 53.0
RPM_TO_MS = (2 * math.pi * WHEEL_RADIUS) / 60.0
INCR_TO_RAD = 2 * math.pi / (2 ** 14)  # 14-bit encoder
ANGLE_THRESHOLD_RAD = 0.5 * math.pi / 180.0  # 0.5°
LENGTH = 0.835  # distance between front/back axles
WIDTH = 0.75   # distance between left/right wheels

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

        # parameters
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT


        self.declare_parameter('beta_yaw_fusion', 0.75)
        self.declare_parameter('yaw_avg_window', 5)
        self.declare_parameter('startup_delay_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 60.0)

        self.beta_yaw = self.get_parameter('beta_yaw_fusion').value
        self.yaw_window = deque(maxlen=self.get_parameter('yaw_avg_window').value)
        self.startup_delay = self.get_parameter('startup_delay_sec').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value

        # state
        self.yaw = 0.0
        self.first_imu_time = None
        self.last_time = None

        self.imu_yaw = None
        self.imu_gyro_z = None

        self.wheel_speeds = [0.0]*4
        self.wheel_angles = [0.0]*4

        self.latest_vx = 0.0
        self.latest_vy = 0.0
        self.wheel_yaw = 0.0

        # subscribers
        self.create_subscription(
            Imu,
            '/imu/waveshare',
            self.imu_callback,
            qos_sensor
        )
        self.create_subscription(
            MotorStatus,
            '/NAV/motor_nav_status',
            self.motor_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_odom_callback,
            10
        )

        # publisher & timer
        self.odom_pub = self.create_publisher(Odometry, '/odom_processed', 10)
        self.create_timer(1.0/self.publish_rate, self.update)

        self.get_logger().info('OdomPreprocessor (w/ wheel + waveshare IMU) started.')

    def imu_callback(self, msg: Imu):
        # extract yaw & gyro-z
        self.imu_yaw = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.imu_gyro_z = msg.angular_velocity.z

        if self.first_imu_time is None:
            self.first_imu_time = self.get_clock().now()

    def motor_callback(self, msg: MotorStatus):
        if len(msg.velocity) != 4 or len(msg.position) != 4:
            return
        for i in range(4):
            # convert RPM → m/s, flip sign if needed
            self.wheel_speeds[i] = msg.velocity[i] * GEAR_RATIO * RPM_TO_MS * (-1.0 if i in [1,2] else 1.0)
            angle = msg.position[i] * INCR_TO_RAD
            # keep in [-π, π]
            self.wheel_angles[i] = (angle + math.pi) % (2*math.pi) - math.pi

    def wheel_odom_callback(self, msg: Odometry):
        # wheel-odom twist in world frame → body frame
        world_vx = msg.twist.twist.linear.x
        world_vy = msg.twist.twist.linear.y

        yaw_w = quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        # invert rotation
        self.latest_vx =  math.cos(-yaw_w)*world_vx - math.sin(-yaw_w)*world_vy
        self.latest_vy =  math.sin(-yaw_w)*world_vx + math.cos(-yaw_w)*world_vy
        self.wheel_yaw = yaw_w

    def update(self):
        # wait for initial data & startup-delay
        if self.imu_yaw is None or self.imu_gyro_z is None or self.first_imu_time is None:
            return
        now = self.get_clock().now()
        if (now - self.first_imu_time).nanoseconds * 1e-9 < self.startup_delay:
            return

        # time delta
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # check steering angle magnitude
        avg_angle = sum(abs(a) for a in self.wheel_angles) / 4.0

        # fuse yaw: more trust in IMU when wheels straight, blend otherwise
        if avg_angle < ANGLE_THRESHOLD_RAD:
            fused = self.imu_yaw
        else:
            fused = self.beta_yaw * self.imu_yaw + (1.0 - self.beta_yaw) * self.wheel_yaw

        # running average for smoothness
        self.yaw_window.append(fused)
        self.yaw = (sum(self.yaw_window) / len(self.yaw_window) + math.pi) % (2*math.pi) - math.pi

        # integrate position in odom frame
        dx = self.latest_vx * math.cos(self.yaw) * dt - self.latest_vy * math.sin(self.yaw) * dt
        dy = self.latest_vx * math.sin(self.yaw) * dt + self.latest_vy * math.cos(self.yaw) * dt

        # accumulate
        if not hasattr(self, 'pos_x'):
            self.pos_x = 0.0
            self.pos_y = 0.0
        self.pos_x += dx
        self.pos_y += dy

        # publish
        out = Odometry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'

        out.pose.pose.position.x = self.pos_x
        out.pose.pose.position.y = self.pos_y
        out.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        out.twist.twist.linear.x = self.latest_vx
        out.twist.twist.linear.y = self.latest_vy
        out.twist.twist.angular.z = 0.0

        out.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

        out.twist.covariance = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

        self.odom_pub.publish(out)

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = OdomPreprocessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
