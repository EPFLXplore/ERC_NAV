import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import math
import copy
from collections import deque

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (rotation around Z axis in radians)."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class OdomPreprocessor(Node):
    def __init__(self):
        super().__init__('odom_preprocessor')
        qos_profile = qos_profile_sensor_data

        # Parameters
        self.declare_parameter('acc_threshold', 0.1)
        self.declare_parameter('alpha', 0.25)
        self.declare_parameter('yaw_avg_window', 5)
        self.declare_parameter('publish_rate_hz', 80.0)

        self.acc_threshold = self.get_parameter('acc_threshold').value
        self.alpha = self.get_parameter('alpha').value
        self.avg_window_size = self.get_parameter('yaw_avg_window').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value

        self.x = 0.0  # filtered yaw

        # Latest sensor readings
        self.latest_acc = None
        self.latest_imu_yaw = None
        self.latest_madgwick_yaw = None
        self.prev_madgwick_yaw = None

        self.last_time = None
        self.yaw_buffer = deque(maxlen=self.avg_window_size)

        # Subscribers
        # self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu_nano', self.imu_callback, qos_profile)
        self.create_subscription(Imu, '/imu/modified', self.imu_ouster_callback, qos_profile)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom_processed', 10)

        # Timer callback to run the filter and publish even without wheel odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.process_imu_yaw)

        self.get_logger().info('Odom Preprocessor Node Started (IMU-only mode)')

    def imu_callback(self, msg: Imu):
        self.latest_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.latest_imu_yaw = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

    def imu_ouster_callback(self, msg: Imu):
        self.latest_madgwick_yaw = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

    def process_imu_yaw(self):
        # Only proceed if we have all required IMU inputs
        if (
            self.latest_acc is None
            or self.latest_imu_yaw is None
            or self.latest_madgwick_yaw is None
        ):
            return

        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            self.prev_madgwick_yaw = self.latest_madgwick_yaw
            return
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # delta yaw from Madgwick IMU
        delta_yaw = self.wrap_to_pi(self.latest_madgwick_yaw - self.prev_madgwick_yaw)
        self.prev_madgwick_yaw = self.latest_madgwick_yaw

        # Predict yaw and apply complementary correction from imu_nano
        pred_yaw = self.wrap_to_pi(self.x + delta_yaw)
        fused_yaw = self.alpha * pred_yaw + (1 - self.alpha) * self.latest_imu_yaw
        fused_yaw = self.wrap_to_pi(fused_yaw)

        # Smooth final yaw using moving average
        self.yaw_buffer.append(fused_yaw)
        smoothed_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)
        self.x = self.wrap_to_pi(smoothed_yaw)

        self.get_logger().info(f'[FUSED] ΔYaw={delta_yaw * 180 / math.pi:.4f}°, '
                               f'Raw={fused_yaw * 180 / math.pi:.4f}°, '
                               f'Smoothed={self.x * 180 / math.pi:.4f}°')

        # Create odometry message
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation = self.yaw_to_quaternion(self.x)

        # Optional: zero velocity if acceleration is low
        acc_norm = np.linalg.norm(self.latest_acc[:2])
        if acc_norm < self.acc_threshold:
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0

        self.odom_pub.publish(msg)

    # def odom_callback(self, msg: Odometry):
    #     # Original wheel_odom-based callback (commented out for IMU-only mode)
    #     pass

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

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
