import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

class GlimOdomRepublisher(Node):
    def __init__(self):
        super().__init__('glim_odom_publisher')

        # Parameters
        self.declare_parameter('pose_topic', '/glim_rosnode/pose_corrected')
        self.declare_parameter('odom_topic', '/odom_glim_repub')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('position_covariance', [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.07])
        self.declare_parameter('orientation_covariance', [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04])
        # Controls the rotation direction: use -1 (standard). If motion still follows old heading, try +1.
        self.declare_parameter('rotation_sign', -1)  # {-1, +1}
        # Debug logs of raw/rotated deltas
        self.declare_parameter('debug_logs', True)

        # Load params
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.position_covariance = self.get_parameter('position_covariance').get_parameter_value().double_array_value
        self.orientation_covariance = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value
        self.rotation_sign = self.get_parameter('rotation_sign').get_parameter_value().integer_value
        self.debug_logs = self.get_parameter('debug_logs').get_parameter_value().bool_value

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        # Capture state after 5s
        self._capture_next = False
        self._timer = self.create_timer(5.0, self._arm_capture_once)

        # Reference (origin + yaw) and rotation matrix R(sign*yaw0)
        self._x0 = 0.0
        self._y0 = 0.0
        self._yaw0 = 0.0
        self._Rc = 1.0
        self._Rs = 0.0
        self._zeroed = False

        # For Twist estimation (optional)
        self._prev_t = None
        self._prev_xr = None
        self._prev_yr = None
        self._prev_yaw = None

        self.get_logger().info(f"GLIM odom re-zero: waiting 5s, then reframe to yaw=0 and origin at capture. Sub: {self.pose_topic}")

    def _arm_capture_once(self):
        self._capture_next = True
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("Timer fired: next pose will set (x0,y0,yaw0)")

    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()

        x = msg.pose.position.x * (-1.0)
        y = msg.pose.position.y
        z = msg.pose.position.z
        yaw = yaw_from_quaternion(msg.pose.orientation)

        if self._capture_next:
            self._x0, self._y0, self._yaw0 = x, y, yaw
            phi = self.rotation_sign * self._yaw0  # rotate by sign * yaw0
            self._Rc = math.cos(phi)
            self._Rs = math.sin(phi)
            self._zeroed = True
            self._capture_next = False
            self._prev_t = None
            self._prev_xr = None
            self._prev_yr = None
            self._prev_yaw = None
            self.get_logger().info(f"Captured x0={self._x0:.3f}, y0={self._y0:.3f}, yaw0={self._yaw0:.3f} rad; rotation_sign={self.rotation_sign}")

        # Translate relative to captured origin
        dx = x - self._x0
        dy = y - self._y0

        # Rotate into new frame: p' = R(sign*yaw0) * (p - p0)
        # With sign = -1 (default), this is R(-yaw0)·(p - p0) — the standard “re-zero to robot’s heading” transform.
        xr = self._Rc * dx - self._Rs * dy
        yr = self._Rs * dx + self._Rc * dy

        # Re-zero heading: yaw' = wrap(yaw - yaw0)
        yaw_r = wrap_pi(yaw - self._yaw0) if self._zeroed else yaw

        if self.debug_logs and self._zeroed:
            self.get_logger().info(f"raw dx,dy=({dx:.3f},{dy:.3f}) -> rotated xr,yr=({xr:.3f},{yr:.3f}); yaw0={self._yaw0:.3f}, yaw={yaw:.3f}, yaw'={yaw_r:.3f}")

        # Orientation quaternion in the re-zeroed frame
        qx, qy, qz, qw = quaternion_from_yaw(yaw_r)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.child_frame_id

        self.get_logger().info(f"flipped x sign")
        odom.pose.pose.position.x = xr
        odom.pose.pose.position.y = yr
        odom.pose.pose.position.z = z

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = [
            self.position_covariance[0], self.position_covariance[1], self.position_covariance[2], 0.0, 0.0, 0.0,
            self.position_covariance[3], self.position_covariance[4], self.position_covariance[5], 0.0, 0.0, 0.0,
            self.position_covariance[6], self.position_covariance[7], self.position_covariance[8], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0], self.orientation_covariance[1], self.orientation_covariance[2],
            0.0, 0.0, 0.0, self.orientation_covariance[3], self.orientation_covariance[4], self.orientation_covariance[5],
            0.0, 0.0, 0.0, self.orientation_covariance[6], self.orientation_covariance[7], self.orientation_covariance[8]
        ]

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = GlimOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
