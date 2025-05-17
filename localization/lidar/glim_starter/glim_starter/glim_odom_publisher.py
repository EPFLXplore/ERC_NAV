import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

def yaw_from_quaternion(q):
    """
    Extract yaw (Z-axis rotation) from a quaternion.
    Input: geometry_msgs.msg.Quaternion
    Output: yaw (float, radians)
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw):
    """
    Create a quaternion from a yaw angle.
    Input: yaw (float, radians)
    Output: tuple (x, y, z, w)
    """
    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return (0.0, 0.0, qz, qw)

class GlimOdomRepublisher(Node):
    def __init__(self):
        super().__init__('glim_odom_publisher')

        # Parameters
        self.declare_parameter('pose_topic', '/glim_rosnode/pose_corrected')
        self.declare_parameter('odom_topic', '/odom_glim_repub')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('lowpass_cutoff', 20.0)  # Hz
        self.declare_parameter('position_covariance', [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.07])
        self.declare_parameter('orientation_covariance', [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04])

        # Load params
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.lowpass_cutoff = self.get_parameter('lowpass_cutoff').get_parameter_value().double_value
        self.position_covariance = self.get_parameter('position_covariance').get_parameter_value().double_array_value
        self.orientation_covariance = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Subscriber
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        # State
        self.last_pos = None
        self.last_yaw = None
        self.last_time = None  # For dynamic dt

        self.get_logger().info(f'Glim Odometry Republisher Started! Listening to: {self.pose_topic}')

    def compute_alpha(self, cutoff_hz, dt):
        rc = 1.0 / (2 * math.pi * cutoff_hz)
        return dt / (rc + dt)

    def pose_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.02  # Assume 50Hz initially
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 1.0:
                dt = 0.02
        self.last_time = now

        alpha = self.compute_alpha(self.lowpass_cutoff, dt)

        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        q = msg.pose.orientation
        yaw = yaw_from_quaternion(q)

        if self.last_pos is None:
            self.last_pos = pos
            self.last_yaw = yaw
        else:
            self.last_pos += alpha * (pos - self.last_pos)
            yaw_diff = yaw - self.last_yaw
            yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
            self.last_yaw += alpha * yaw_diff

        qx, qy, qz, qw = quaternion_from_yaw(self.last_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = self.last_pos[0]
        odom_msg.pose.pose.position.y = self.last_pos[1]
        odom_msg.pose.pose.position.z = self.last_pos[2]

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.pose.covariance = [
            self.position_covariance[0], self.position_covariance[1], self.position_covariance[2], 0.0, 0.0, 0.0,
            self.position_covariance[3], self.position_covariance[4], self.position_covariance[5], 0.0, 0.0, 0.0,
            self.position_covariance[6], self.position_covariance[7], self.position_covariance[8], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0], self.orientation_covariance[1], self.orientation_covariance[2],
            0.0, 0.0, 0.0, self.orientation_covariance[3], self.orientation_covariance[4], self.orientation_covariance[5],
            0.0, 0.0, 0.0, self.orientation_covariance[6], self.orientation_covariance[7], self.orientation_covariance[8]
        ]


        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlimOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
