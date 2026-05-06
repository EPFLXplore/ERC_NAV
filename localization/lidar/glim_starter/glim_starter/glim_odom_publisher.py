import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate_xy(x, y, theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return c * x - s * y, s * x + c * y


class GlimOdomRepublisher(Node):
    """
    Re-publish GLIM pose as odometry with one-shot origin capture.

    Initialization strategy is kept:
      - wait 5s
      - arm capture
      - first pose after arm sets (x0, y0, z0, yaw0)
      - all subsequent poses are expressed relative to that captured origin
    """

    def __init__(self):
        super().__init__("glim_odom_publisher")

        # Core topics/frames
        self.declare_parameter("pose_topic", "/glim_rosnode/pose_corrected")
        self.declare_parameter("odom_topic", "/odom_glim_repub")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # Covariances (3x3 blocks flattened)
        self.declare_parameter(
            "position_covariance",
            [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.07],
        )
        self.declare_parameter(
            "orientation_covariance",
            [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04],
        )

        # Main rigid transform knobs
        self.declare_parameter("initial_align_yaw_offset_rad", math.pi / 2.0)
        self.declare_parameter("output_plane_rotation_rad", 0.0)
        # If False, keep capture for origin translation but use only fixed yaw offsets for axes alignment.
        self.declare_parameter("use_initial_yaw_for_alignment", True)
        self.declare_parameter("zero_z", True)
        self.declare_parameter("debug_logs", False)

        # Legacy / compatibility knobs
        self.declare_parameter("reflect_x_after", False)
        self.declare_parameter("rotation_sign", False)
        self.declare_parameter("invert_output_x", False)
        self.declare_parameter("remap_body_x_from_odom_neg_y", False)
        self.declare_parameter("remap_neg_y_to_plus_x_include_yaw", False)

        # Load params
        self.pose_topic = self.get_parameter("pose_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.position_covariance = list(self.get_parameter("position_covariance").value)
        self.orientation_covariance = list(self.get_parameter("orientation_covariance").value)

        self.initial_align_yaw_offset_rad = float(
            self.get_parameter("initial_align_yaw_offset_rad").value
        )
        self.output_plane_rotation_rad = float(
            self.get_parameter("output_plane_rotation_rad").value
        )
        self.use_initial_yaw_for_alignment = bool(
            self.get_parameter("use_initial_yaw_for_alignment").value
        )
        self.zero_z = bool(self.get_parameter("zero_z").value)
        self.debug_logs = bool(self.get_parameter("debug_logs").value)

        self.reflect_x_after = bool(self.get_parameter("reflect_x_after").value)
        self.rotation_sign = bool(self.get_parameter("rotation_sign").value)
        self.invert_output_x = bool(self.get_parameter("invert_output_x").value)
        self.remap_body_x_from_odom_neg_y = bool(
            self.get_parameter("remap_body_x_from_odom_neg_y").value
        )
        self.remap_neg_y_to_plus_x_include_yaw = bool(
            self.get_parameter("remap_neg_y_to_plus_x_include_yaw").value
        )

        if len(self.position_covariance) != 9:
            self.get_logger().warn("position_covariance must have 9 values, using defaults.")
            self.position_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.07]
        if len(self.orientation_covariance) != 9:
            self.get_logger().warn("orientation_covariance must have 9 values, using defaults.")
            self.orientation_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04]

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, qos
        )

        # Initialization strategy (kept)
        self._capture_next = False
        self._timer = self.create_timer(5.0, self._arm_capture_once)

        self._x0 = 0.0
        self._y0 = 0.0
        self._z0 = 0.0
        self._yaw0 = 0.0
        self._zeroed = False

        self.get_logger().info("Waiting 5s to capture (x0,y0,z0,yaw0) and re-zero.")
        self.get_logger().info(
            "Transform params: "
            f"init_offset={self.initial_align_yaw_offset_rad:.4f}, "
            f"output_rot={self.output_plane_rotation_rad:.4f}, "
            f"use_initial_yaw_for_alignment={self.use_initial_yaw_for_alignment}, "
            f"zero_z={self.zero_z}, "
            f"legacy(reflect_x_after={self.reflect_x_after}, rotation_sign={self.rotation_sign}, "
            f"invert_output_x={self.invert_output_x}, remap_neg_y={self.remap_body_x_from_odom_neg_y}, "
            f"remap_neg_y_with_yaw={self.remap_neg_y_to_plus_x_include_yaw})"
        )

    def _arm_capture_once(self):
        self._capture_next = True
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("Timer fired: next pose sets origin.")

    def _apply_main_rigid_transform(self, x, y, z, yaw):
        """Apply captured-origin re-zero + configured rigid plane rotations."""
        if not self._zeroed:
            # Before origin capture, pass-through to avoid surprising pre-zero rotation.
            return x, y, z, yaw

        dx = x - self._x0
        dy = y - self._y0
        dz = (z - self._z0) if self.zero_z else z

        if self.use_initial_yaw_for_alignment:
            align = -self._yaw0 + self.initial_align_yaw_offset_rad
            yaw_o = wrap_pi(yaw - self._yaw0 + self.initial_align_yaw_offset_rad)
        else:
            align = self.initial_align_yaw_offset_rad
            yaw_o = wrap_pi(yaw + self.initial_align_yaw_offset_rad)

        xo, yo = rotate_xy(dx, dy, align)

        return xo, yo, dz, yaw_o

    def _apply_legacy_compat(self, x, y, yaw):
        """
        Optional compatibility transforms.

        These are non-ideal and can break rigid consistency. Kept only to avoid
        breaking existing launch setups while migrating.
        """
        xo, yo, yaw_o = x, y, yaw

        if self.reflect_x_after:
            # Mirror X axis: (x, y) -> (-x, y), heading transforms as yaw' = pi - yaw.
            xo = -xo
            yaw_o = wrap_pi(math.pi - yaw_o)

        if self.rotation_sign:
            yaw_o = wrap_pi(-yaw_o)

        if self.remap_body_x_from_odom_neg_y:
            # Rz(+pi/2): x' = -y, y' = x
            # Use this when forward motion appears along odom -Y.
            xo, yo = -yo, xo
            if self.remap_neg_y_to_plus_x_include_yaw:
                yaw_o = wrap_pi(yaw_o + math.pi / 2.0)

        # Apply x inversion last so it affects the FINAL odom frame.
        if self.invert_output_x:
            xo = -xo

        return xo, yo, yaw_o

    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        yaw = yaw_from_quaternion(msg.pose.orientation)

        if self._capture_next:
            self._x0, self._y0, self._z0, self._yaw0 = x, y, z, yaw
            self._zeroed = True
            self._capture_next = False
            self.get_logger().info(
                f"Captured origin x0={self._x0:.3f}, y0={self._y0:.3f}, "
                f"z0={self._z0:.3f}, yaw0={self._yaw0:.3f} rad"
            )

        xo, yo, zo, yaw_o = self._apply_main_rigid_transform(x, y, z, yaw)
        xo, yo, yaw_o = self._apply_legacy_compat(xo, yo, yaw_o)

        # Final frame rotation (applied after all remaps/sign fixes).
        if abs(self.output_plane_rotation_rad) > 1e-9:
            xo, yo = rotate_xy(xo, yo, self.output_plane_rotation_rad)
            yaw_o = wrap_pi(yaw_o + self.output_plane_rotation_rad)

        if self.debug_logs and self._zeroed:
            self.get_logger().info(
                f"raw=({x:.3f},{y:.3f},{yaw:.3f}) -> out=({xo:.3f},{yo:.3f},{yaw_o:.3f})"
            )

        qx, qy, qz, qw = quaternion_from_yaw(yaw_o)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = xo
        odom.pose.pose.position.y = yo
        odom.pose.pose.position.z = zo
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = [
            self.position_covariance[0],
            self.position_covariance[1],
            self.position_covariance[2],
            0.0,
            0.0,
            0.0,
            self.position_covariance[3],
            self.position_covariance[4],
            self.position_covariance[5],
            0.0,
            0.0,
            0.0,
            self.position_covariance[6],
            self.position_covariance[7],
            self.position_covariance[8],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            self.orientation_covariance[0],
            self.orientation_covariance[1],
            self.orientation_covariance[2],
            0.0,
            0.0,
            0.0,
            self.orientation_covariance[3],
            self.orientation_covariance[4],
            self.orientation_covariance[5],
            0.0,
            0.0,
            0.0,
            self.orientation_covariance[6],
            self.orientation_covariance[7],
            self.orientation_covariance[8],
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
