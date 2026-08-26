#!/usr/bin/env python3

# Work around the removed `np.float` alias in NumPy >= 1.20
import numpy as _np
if not hasattr(_np, "float"):
    _np.float = float

import math
from collections import namedtuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
import tf_transformations


# =========================================================================
# Waypoint description
# =========================================================================
#
# Each waypoint carries its own pause duration, so there is no longer a
# single global "wait at every waypoint" time.
#
#   WP([x, y, yaw], ERC_WPT=True, wait_time=15.0)
#
#   position   [x, y] or [x, y, yaw]. The third value is the heading in
#              radians, NOT an altitude. Paste 0.0 when the ERC sheet gives
#              you a z coordinate you do not care about.
#   ERC_WPT    True  -> official ERC waypoint, counted and announced as such
#              False -> intermediate waypoint used only to shape the route
#   wait_time  Seconds the rover stands still once the waypoint is reached.
#              0.0 means drive straight through to the next one.
#
# The pause is executed by THIS node, not by Nav2's WaitAtWaypoint plugin:
# waypoints are sent one goal at a time so every one of them can have a
# different duration. See disable_nav2_waypoint_pause() below for how the
# Nav2-side pause is neutralised.
#
# RETRY POLICY
# ------------
# A waypoint that Nav2 reports as failed is NOT skipped. It is re-sent after
# `retry_delay` seconds, up to `max_attempts_per_waypoint` times
# (0 = retry forever). Between attempts the costmaps are optionally cleared,
# because a failure caused by a stale lethal cell is otherwise deterministic
# and every retry would fail identically.
#
# =========================================================================

WaypointSpec = namedtuple(
    "WaypointSpec",
    ["x", "y", "yaw", "erc_wpt", "wait_time"],
)


def WP(position, ERC_WPT=True, wait_time=15.0):
    """
    Build one waypoint entry for `waypoint_list`.

    `position` is [x, y] or [x, y, yaw], expressed in whichever convention
    `waypoint_input_coordinates` selects ("map" or "erc_map").
    """
    if len(position) < 2:
        raise ValueError(f"Waypoint position needs at least [x, y], got {position}")

    yaw = float(position[2]) if len(position) > 2 else 0.0

    return WaypointSpec(
        x=float(position[0]),
        y=float(position[1]),
        yaw=yaw,
        erc_wpt=bool(ERC_WPT),
        wait_time=float(wait_time),
    )


class WaypointFollower(Node):
    def __init__(self):
        # Keep this client distinct from Nav2's /waypoint_follower server.
        super().__init__("waypoint_sender")

        # =====================================================================
        # Coordinate convention
        # =====================================================================
        #
        # ROS / Nav2 / RViz use the real TF frame called:
        #
        #   map
        #
        # Your ERC map convention is only a coordinate convention, not necessarily
        # a real TF frame.
        #
        # Conversion used here:
        #
        #   x_map   =  x_erc
        #   y_map   = -y_erc
        #   yaw_map = -yaw_erc
        #
        # and equivalently:
        #
        #   x_erc   =  x_map
        #   y_erc   = -y_map
        #   yaw_erc = -yaw_map
        #
        # Set this to either:
        #
        #   "map"      -> waypoint_list is written directly in ROS/Nav2 map coords
        #   "erc_map"  -> waypoint_list is written in ERC coords and converted to map
        #
        # =====================================================================
        self.declare_parameter("waypoint_input_coordinates", "map") # WATCH OUT FOR MEEE !!!!
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        # Output of the fused navigation EKF (local_nav_ekf/src/NavEKF3D.cpp),
        # not raw wheel odometry: pose, body-frame velocities, attitude and
        # covariances of the filter that fuses wheel odom, IMU, LiDAR, ArUco
        # and VIO. Wheel odometry alone is only one of its inputs (/wheel_odom).
        self.declare_parameter("ekf_odom_topic", "/fused_nav_ekf_odom")
        # Age above which the EKF stream is considered stale, in seconds.
        self.declare_parameter("ekf_odom_timeout", 1.0)
        self.declare_parameter("acceptance_radius", 0.07)
        self.declare_parameter("visualization_rate_hz", 2.0)

        # ---------------- Retry policy ----------------
        # How many times a single waypoint is attempted before it is finally
        # given up on. 0 means "never give up", which is what you want if a
        # waypoint must not be skipped under any circumstance. Be aware that
        # with 0 the mission can block forever on an unreachable waypoint.
        self.declare_parameter("max_attempts_per_waypoint", 0)
        # Seconds to wait between two attempts at the same waypoint.
        self.declare_parameter("retry_delay", 3.0)
        # Clear the costmaps before re-sending. A failure caused by a stale
        # lethal cell (planner "starting point in lethal space") repeats
        # identically otherwise.
        self.declare_parameter("clear_costmaps_on_retry", True)
        self.declare_parameter(
            "local_costmap_clear_service",
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.declare_parameter(
            "global_costmap_clear_service",
            "/global_costmap/clear_entirely_global_costmap",
        )

        # Pause the same way even when Nav2 reports the waypoint as missed.
        # False = a failed waypoint is skipped immediately, no science stop.
        # Only reachable once the retries are exhausted.
        self.declare_parameter("wait_on_failed_waypoint", False)

        # Nav2's own WaitAtWaypoint plugin would add its pause on top of the
        # per-waypoint one handled here, so this node tries to zero it out.
        self.declare_parameter("disable_nav2_pause", True)
        self.declare_parameter("waypoint_follower_node", "/waypoint_follower")
        self.declare_parameter(
            "nav2_pause_parameter",
            "wait_at_waypoint.waypoint_pause_duration",
        )

        self.waypoint_input_coordinates = self.get_parameter(
            "waypoint_input_coordinates"
        ).value

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.ekf_odom_topic = self.get_parameter("ekf_odom_topic").value
        self.ekf_odom_timeout = float(self.get_parameter("ekf_odom_timeout").value)
        self.acceptance_radius = float(self.get_parameter("acceptance_radius").value)
        self.visualization_rate_hz = float(
            self.get_parameter("visualization_rate_hz").value
        )
        self.max_attempts = int(
            self.get_parameter("max_attempts_per_waypoint").value
        )
        self.retry_delay = float(self.get_parameter("retry_delay").value)
        self.clear_costmaps_on_retry = bool(
            self.get_parameter("clear_costmaps_on_retry").value
        )
        self.local_costmap_clear_service = self.get_parameter(
            "local_costmap_clear_service"
        ).value
        self.global_costmap_clear_service = self.get_parameter(
            "global_costmap_clear_service"
        ).value
        self.wait_on_failed_waypoint = bool(
            self.get_parameter("wait_on_failed_waypoint").value
        )
        self.disable_nav2_pause = bool(self.get_parameter("disable_nav2_pause").value)
        self.waypoint_follower_node = self.get_parameter("waypoint_follower_node").value
        self.nav2_pause_parameter = self.get_parameter("nav2_pause_parameter").value

        if self.waypoint_input_coordinates not in ["map", "erc_map"]:
            self.get_logger().warn(
                f"Invalid waypoint_input_coordinates='{self.waypoint_input_coordinates}'. "
                "Using 'map'. Valid options are: 'map', 'erc_map'."
            )
            self.waypoint_input_coordinates = "map"

        self.get_logger().info("=======================================================")
        self.get_logger().info("Waypoint coordinate convention")
        self.get_logger().info(f"  ROS/Nav2/RViz frame:          {self.map_frame}")
        self.get_logger().info(
            f"  Waypoint input coordinates:   {self.waypoint_input_coordinates}"
        )
        self.get_logger().info("  ERC -> map conversion:        x_map=x_erc, y_map=-y_erc, yaw_map=-yaw_erc")
        self.get_logger().info("=======================================================")

        attempts_txt = (
            "unlimited" if self.max_attempts <= 0 else str(self.max_attempts)
        )
        self.get_logger().info(
            f"Retry policy: {attempts_txt} attempts per waypoint, "
            f"{self.retry_delay:.1f} s between attempts, "
            f"clear_costmaps_on_retry={self.clear_costmaps_on_retry}"
        )

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self.specs, self.waypoints = self.create_waypoints()
        self.curr_waypoint_index = 0
        self.missed_waypoints = []

        # Attempts already made on self.curr_waypoint_index.
        self._attempt = 0
        # Total retries over the whole mission, for the final summary.
        self._total_retries = 0

        # ---------------- Navigation EKF state ----------------
        # NavEKF3D publishes twist in child_frame_id = base_link, so the
        # linear velocities are body frame: x forward, y left, z up.
        self.current_speed = 0.0        # horizontal ground speed [m/s]
        self.forward_speed = 0.0        # signed, negative while reversing
        self.lateral_speed = 0.0
        self.vertical_speed = 0.0
        self.yaw_rate = 0.0

        self.ekf_roll = 0.0
        self.ekf_pitch = 0.0
        self.ekf_yaw = 0.0
        self.ekf_tilt = 0.0             # total tilt from vertical [rad]

        self.ekf_sigma_xy = None        # 1-sigma horizontal position error [m]
        self.ekf_sigma_yaw = None       # 1-sigma heading error [rad]

        self.last_ekf_time = None       # arrival time of the last EKF message

        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

        self._pause_timer = None
        self._pause_index = None
        self._pause_end_time = None

        self._retry_timer = None
        self._retry_index = None

        self._mission_finished = False

        # ---------------- Nav2 action client ----------------
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        # ---------------- Costmap clearing clients ----------------
        self._clear_local_client = self.create_client(
            ClearEntireCostmap, self.local_costmap_clear_service
        )
        self._clear_global_client = self.create_client(
            ClearEntireCostmap, self.global_costmap_clear_service
        )

        # ---------------- Visualization publishers ----------------
        viz_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.path_pub = self.create_publisher(Path, "/waypoints_path", viz_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", viz_qos)
        self.posearray_pub = self.create_publisher(PoseArray, "/waypoints_posearray", viz_qos)

        # ---------------- Subscriptions ----------------
        # Match the NavEKF3D publisher QoS: KeepLast(1), reliable.
        ekf_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            Odometry,
            self.ekf_odom_topic,
            self.ekf_odom_callback,
            ekf_qos,
        )

        # ---------------- Timers ----------------
        viz_period = 1.0 / max(self.visualization_rate_hz, 0.1)
        self.viz_timer = self.create_timer(viz_period, self.publish_visualizations)
        self.status_timer = self.create_timer(1.0, self.print_current_position)

        # ---------------- Start the mission ----------------
        self.disable_nav2_waypoint_pause()
        self.start_mission()

    # =====================================================================
    # Coordinate conversion
    # =====================================================================
    def erc_to_map(self, x_erc, y_erc, yaw_erc):
        """
        Convert ERC-map convention coordinates into ROS/Nav2 map coordinates.
        """
        x_map = x_erc
        y_map = -y_erc
        yaw_map = -yaw_erc
        return x_map, y_map, yaw_map

    def map_to_erc(self, x_map, y_map, yaw_map):
        """
        Convert ROS/Nav2 map coordinates into ERC-map convention coordinates.
        """
        x_erc = x_map
        y_erc = -y_map
        yaw_erc = -yaw_map
        return x_erc, y_erc, yaw_erc

    def input_to_map(self, x_in, y_in, yaw_in):
        """
        Convert the user-entered waypoint list into the actual ROS/Nav2 map frame.
        """
        if self.waypoint_input_coordinates == "erc_map":
            return self.erc_to_map(x_in, y_in, yaw_in)

        return x_in, y_in, yaw_in

    # =====================================================================
    # Waypoints
    # =====================================================================
    def create_waypoints(self):
        poses = []

        def make_pose_in_ros_map(x_map, y_map, yaw_map):
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = float(x_map)
            pose.pose.position.y = float(y_map)
            pose.pose.position.z = 0.0

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_map)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            return pose

        # =====================================================================
        # Hardcoded waypoints
        # =====================================================================
        #
        # Positions are interpreted according to:
        #
        #   self.waypoint_input_coordinates
        #
        # If waypoint_input_coordinates == "map":
        #   WP([x_map, y_map, yaw_map], ...)
        #
        # If waypoint_input_coordinates == "erc_map":
        #   WP([x_erc, y_erc, yaw_erc], ...)
        #
        # The values actually sent to Nav2 are always converted to ROS/Nav2 map.
        #
        # ERC_WPT and wait_time are independent: an intermediate waypoint may
        # still pause (ERC_WPT=False, wait_time=5.5), and an ERC waypoint may
        # be driven through (ERC_WPT=True, wait_time=0.0).
        #
        # =====================================================================

        # ---------------- ERC competition waypoints ----------------
        # The third value is a heading in radians. The original list carried
        # altitudes there, which would have been used as yaw, so it is 0.0.
        W1 = WP([9.759, 1.951, 0.0], ERC_WPT=True, wait_time=15.0)
        W2 = WP([19.499, 4.170, 0.0], ERC_WPT=True, wait_time=15.0)
        W3 = WP([23.149, 7.013, 0.0], ERC_WPT=True, wait_time=15.0)
        W4 = WP([18.665, -1.701, 0.0], ERC_WPT=True, wait_time=15.0)
        W5 = WP([19.034, -5.339, 0.0], ERC_WPT=True, wait_time=15.0)
        W6 = WP([8.197, -7.675, 0.0], ERC_WPT=True, wait_time=15.0)
        W7 = WP([7.682, -2.108, 0.0], ERC_WPT=True, wait_time=15.0)
        W8 = WP([6.790, 5.939, 0.0], ERC_WPT=True, wait_time=15.0)
        W9 = WP([13.092, 5.088, 0.0], ERC_WPT=True, wait_time=15.0)
        P1 = WP([25.901, -5.997, 0.0], ERC_WPT=True, wait_time=15.0)

        waypoint_list = [W1, W2, W3, W4]

        # ---------------- Current test route ----------------

        waypoint_list = [
            WP([0.0, 8.0, 0.0],   ERC_WPT=True,   wait_time=15.0),
            WP([-7.0, 5.0, 0.0],  ERC_WPT=True,   wait_time=15.0),
            WP([-15.0, 8.5, 0.0],  ERC_WPT=True,   wait_time=15.0),
            WP([-23.0, 6.0, 0.0], ERC_WPT=True,   wait_time=15.0),
            WP([-15.0, -3.0, 0.0],   ERC_WPT=True,   wait_time=15.0),
            WP([0.0, 0.0, 0.0],   ERC_WPT=True,   wait_time=15.0),
        ]

        # Example of a route that mixes scored stops and shaping waypoints:
        #
        # waypoint_list = [
        #     WP([9.759, 1.951, 0.0], ERC_WPT=True, wait_time=15.0),
        #     WP([14.0, 3.0], ERC_WPT=False, wait_time=0.0),
        #     WP([19.499, 4.170, 0.0], ERC_WPT=True, wait_time=15.0),
        #     WP([21.0, 6.0], ERC_WPT=False, wait_time=5.5),
        #     WP([23.149, 7.013, 0.0], ERC_WPT=True, wait_time=15.0),
        # ]

        specs = []
        erc_count = 0

        for i, wp in enumerate(waypoint_list):
            x_map, y_map, yaw_map = self.input_to_map(wp.x, wp.y, wp.yaw)
            x_erc, y_erc, yaw_erc = self.map_to_erc(x_map, y_map, yaw_map)

            # Store the spec in ROS/Nav2 map coordinates, like the poses.
            specs.append(wp._replace(x=x_map, y=y_map, yaw=yaw_map))
            poses.append(make_pose_in_ros_map(x_map, y_map, yaw_map))

            if wp.erc_wpt:
                erc_count += 1
                kind = f"ERC #{erc_count}"
            else:
                kind = "intermediate"

            self.get_logger().info(
                f"WP {i:02d} [{kind}, wait={wp.wait_time:.1f}s] | "
                f"input[{self.waypoint_input_coordinates}]: "
                f"x={wp.x:.2f}, y={wp.y:.2f}, yaw={wp.yaw:.2f} | "
                f"sent_to_nav2[map]: "
                f"x={x_map:.2f}, y={y_map:.2f}, yaw={yaw_map:.2f} | "
                f"erc_equivalent: "
                f"x={x_erc:.2f}, y={y_erc:.2f}, yaw={yaw_erc:.2f}"
            )

        total_wait = sum(wp.wait_time for wp in waypoint_list)
        self.get_logger().info(
            f"{len(waypoint_list)} waypoints ({erc_count} ERC), "
            f"total programmed pause {total_wait:.1f} s"
        )

        return specs, poses

    def erc_number(self, index):
        """
        1-based position of `index` among the ERC waypoints, or None if that
        waypoint is only an intermediate one.
        """
        if not self.specs[index].erc_wpt:
            return None

        return sum(1 for spec in self.specs[: index + 1] if spec.erc_wpt)

    def waypoint_label(self, index):
        erc_no = self.erc_number(index)
        kind = f"ERC {erc_no}" if erc_no is not None else "intermediate"
        return f"WP {index + 1}/{len(self.specs)} ({kind})"

    # =====================================================================
    # Visualization helpers
    # =====================================================================
    def make_color(self, marker, r, g, b, a):
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)

    def waypoint_color(self, index):
        if index < self.curr_waypoint_index:
            return 0.0, 1.0, 0.0, 0.65      # visited: green
        if index == self.curr_waypoint_index:
            return 1.0, 0.7, 0.0, 1.0       # current: orange
        return 0.1, 0.8, 1.0, 0.80          # future: cyan

    def get_robot_position(self):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.robot_frame,
            rclpy.time.Time(),
        )
        return tf.transform.translation

    def publish_visualizations(self):
        now = self.get_clock().now().to_msg()

        # ------------------------------------------------------------
        # 1) Path polyline, always in ROS/Nav2 map frame
        # ------------------------------------------------------------
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = now

        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = now
            ps.pose = wp.pose
            path.poses.append(ps)

        self.path_pub.publish(path)

        # ------------------------------------------------------------
        # 2) PoseArray, always in ROS/Nav2 map frame
        # ------------------------------------------------------------
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = now
        pose_array.poses = [wp.pose for wp in self.waypoints]
        self.posearray_pub.publish(pose_array)

        # ------------------------------------------------------------
        # 3) MarkerArray, always rendered in ROS/Nav2 map frame
        # ------------------------------------------------------------
        markers = MarkerArray()

        # Delete old markers first, useful if waypoint count changes
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = now
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        # ------------------------------------------------------------
        # Route line through all waypoints
        # ------------------------------------------------------------
        route_line = Marker()
        route_line.header.frame_id = self.map_frame
        route_line.header.stamp = now
        route_line.ns = "waypoints_route_line"
        route_line.id = 0
        route_line.type = Marker.LINE_STRIP
        route_line.action = Marker.ADD
        route_line.pose.orientation.w = 1.0
        route_line.scale.x = 0.08

        self.make_color(route_line, 0.0, 0.8, 1.0, 0.75)

        for wp in self.waypoints:
            route_line.points.append(
                Point(
                    x=wp.pose.position.x,
                    y=wp.pose.position.y,
                    z=0.05,
                )
            )

        markers.markers.append(route_line)

        # ------------------------------------------------------------
        # Waypoint markers
        # ------------------------------------------------------------
        for i, wp in enumerate(self.waypoints):
            spec = self.specs[i]
            r, g, b, a = self.waypoint_color(i)
            is_current = i == self.curr_waypoint_index

            x_map = wp.pose.position.x
            y_map = wp.pose.position.y

            # ---------------- Orientation arrow ----------------
            # ERC waypoints are drawn larger than intermediate ones.
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = now
            arrow.ns = "waypoint_arrows"
            arrow.id = 200 + i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = wp.pose

            if is_current:
                arrow.scale.x = 0.9
                arrow.scale.y = 0.25
                arrow.scale.z = 0.25
            elif spec.erc_wpt:
                arrow.scale.x = 0.45
                arrow.scale.y = 0.15
                arrow.scale.z = 0.15
            else:
                arrow.scale.x = 0.30
                arrow.scale.y = 0.09
                arrow.scale.z = 0.09

            self.make_color(arrow, r, g, b, a)
            markers.markers.append(arrow)

            # ---------------- Acceptance radius ----------------
            radius = Marker()
            radius.header.frame_id = self.map_frame
            radius.header.stamp = now
            radius.ns = "waypoint_acceptance_radius"
            radius.id = 400 + i
            radius.type = Marker.CYLINDER
            radius.action = Marker.ADD
            radius.pose.position.x = x_map
            radius.pose.position.y = y_map
            radius.pose.position.z = 0.01
            radius.pose.orientation.w = 1.0
            radius.scale.x = 2.0 * self.acceptance_radius
            radius.scale.y = 2.0 * self.acceptance_radius
            radius.scale.z = 0.02

            self.make_color(radius, r, g, b, 0.18)
            markers.markers.append(radius)

            # ---------------- Label with kind and pause ----------------
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = now
            label.ns = "waypoint_labels"
            label.id = 600 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x_map
            label.pose.position.y = y_map
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35 if spec.erc_wpt else 0.25

            erc_no = self.erc_number(i)
            name = f"ERC {erc_no}" if erc_no is not None else f"i{i + 1}"
            text = f"{name} | {spec.wait_time:.1f}s"

            remaining = self.pause_remaining()
            if self._pause_index == i and remaining is not None:
                text = f"{name} | PAUSED {remaining:.0f}s"
            elif is_current and self._attempt > 1:
                # Make retries obvious in RViz.
                text = f"{name} | RETRY {self._attempt}"

            label.text = text

            self.make_color(label, r, g, b, max(a, 0.9))
            markers.markers.append(label)

        # ------------------------------------------------------------
        # Rover-to-current-waypoint line + distance/ETA text
        # ------------------------------------------------------------
        try:
            robot_pos = self.get_robot_position()

            if self.curr_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.curr_waypoint_index].pose.position

                # ---------------- Line to current target ----------------
                to_target = Marker()
                to_target.header.frame_id = self.map_frame
                to_target.header.stamp = now
                to_target.ns = "rover_to_current_waypoint"
                to_target.id = 900
                to_target.type = Marker.LINE_STRIP
                to_target.action = Marker.ADD
                to_target.pose.orientation.w = 1.0
                to_target.scale.x = 0.06

                self.make_color(to_target, 1.0, 1.0, 0.0, 1.0)

                to_target.points.append(
                    Point(
                        x=robot_pos.x,
                        y=robot_pos.y,
                        z=0.25,
                    )
                )
                to_target.points.append(
                    Point(
                        x=target.x,
                        y=target.y,
                        z=0.25,
                    )
                )

                markers.markers.append(to_target)

        except Exception:
            # Avoid spamming logs here because visualization runs often.
            pass

        self.marker_pub.publish(markers)

    # =====================================================================
    # Status / odometry
    # =====================================================================
    def ekf_odom_callback(self, msg):
        """
        Consume the fused navigation EKF output (NavEKF3D.cpp).

        Only velocity, attitude and covariance are taken from here. The pose
        in this message is expressed in the `odom` frame, while the waypoints
        live in `map`, so the rover position still comes from the
        map->base_link TF in get_robot_position().
        """
        # Body-frame velocities, per child_frame_id = base_link.
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        self.forward_speed = vx
        self.lateral_speed = vy
        self.vertical_speed = vz
        self.current_speed = math.hypot(vx, vy)
        self.yaw_rate = msg.twist.twist.angular.z

        # Attitude of the filter, useful to spot a rover fighting a slope.
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.ekf_roll = roll
        self.ekf_pitch = pitch
        self.ekf_yaw = yaw
        self.ekf_tilt = math.acos(
            max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))
        )

        # Diagonal of the 6x6 pose covariance: xx=0, yy=7, yaw=35.
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        var_yaw = msg.pose.covariance[35]

        self.ekf_sigma_xy = math.sqrt(max(var_x, 0.0) + max(var_y, 0.0))
        self.ekf_sigma_yaw = math.sqrt(max(var_yaw, 0.0))

        self.last_ekf_time = self.get_clock().now()

    def ekf_age(self):
        """
        Seconds since the last EKF message, or None if none was received yet.
        """
        if self.last_ekf_time is None:
            return None

        return (self.get_clock().now() - self.last_ekf_time).nanoseconds / 1e9

    def ekf_is_fresh(self):
        age = self.ekf_age()
        return age is not None and age <= self.ekf_odom_timeout

    def print_current_position(self):
        try:
            position = self.get_robot_position()

        except Exception as e:
            self.get_logger().error(
                f"Could not get {self.map_frame}->{self.robot_frame} TF: {e}"
            )
            return

        x_erc, y_erc, _ = self.map_to_erc(position.x, position.y, 0.0)

        self.get_logger().info(
            f"ROVER POSITION | "
            f"map: x={position.x:.2f}, y={position.y:.2f} | "
            f"erc_map: x={x_erc:.2f}, y={y_erc:.2f}"
        )

        # ---------------- Fused EKF state ----------------
        age = self.ekf_age()

        if age is None:
            self.get_logger().warn(
                f"No {self.ekf_odom_topic} message yet: the navigation EKF is not "
                "publishing, so speed, attitude and ETA are unavailable"
            )
        elif age > self.ekf_odom_timeout:
            self.get_logger().warn(
                f"EKF ODOM STALE | last {self.ekf_odom_topic} message {age:.1f} s ago "
                f"(timeout {self.ekf_odom_timeout:.1f} s), the values below are frozen"
            )

        if age is not None:
            direction = "forward" if self.forward_speed >= 0.0 else "REVERSE"

            self.get_logger().info(
                f"EKF STATE | speed={self.current_speed:.2f} m/s ({direction}) | "
                f"vx={self.forward_speed:+.2f}, vy={self.lateral_speed:+.2f}, "
                f"vz={self.vertical_speed:+.2f} m/s | "
                f"yaw={math.degrees(self.ekf_yaw):.1f} deg, "
                f"yaw_rate={math.degrees(self.yaw_rate):.1f} deg/s | "
                f"tilt={math.degrees(self.ekf_tilt):.1f} deg "
                f"(roll={math.degrees(self.ekf_roll):.1f}, "
                f"pitch={math.degrees(self.ekf_pitch):.1f}) | "
                f"sigma_xy={self.ekf_sigma_xy:.2f} m, "
                f"sigma_yaw={math.degrees(self.ekf_sigma_yaw):.1f} deg"
            )

        remaining = self.pause_remaining()
        if remaining is not None:
            self.get_logger().info(
                f"PAUSED at {self.waypoint_label(self._pause_index)} | "
                f"{remaining:.1f} s left of "
                f"{self.specs[self._pause_index].wait_time:.1f} s"
            )
            return

        if self.curr_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.curr_waypoint_index].pose.position
            spec = self.specs[self.curr_waypoint_index]

            dx = target.x - position.x
            dy = target.y - position.y
            distance = math.sqrt(dx * dx + dy * dy)

            target_x_erc, target_y_erc, _ = self.map_to_erc(
                target.x,
                target.y,
                0.0,
            )

            attempt_txt = ""
            if self._attempt > 1:
                limit = "inf" if self.max_attempts <= 0 else str(self.max_attempts)
                attempt_txt = f" | attempt {self._attempt}/{limit}"

            self.get_logger().info(
                f"TARGET {self.waypoint_label(self.curr_waypoint_index)} | "
                f"map: x={target.x:.2f}, y={target.y:.2f} | "
                f"erc_map: x={target_x_erc:.2f}, y={target_y_erc:.2f} | "
                f"distance={distance:.2f} m | "
                f"pause_on_arrival={spec.wait_time:.1f} s"
                f"{attempt_txt}"
            )

            if not self.ekf_is_fresh():
                self.get_logger().info("ETA: no fresh EKF odometry")
            elif self.current_speed > 0.01:
                eta = distance / self.current_speed
                self.get_logger().info(
                    f"ETA to waypoint {self.curr_waypoint_index}: {eta:.2f} s"
                )
            else:
                self.get_logger().info("ETA: rover speed too low")

        else:
            self.get_logger().info("***** NO MORE WAYPOINTS, TASK FINISHED *****")

    # =====================================================================
    # Nav2 WaitAtWaypoint neutralisation
    # =====================================================================
    def disable_nav2_waypoint_pause(self):
        """
        Ask the Nav2 waypoint_follower to stop adding its own fixed pause.

        This node pauses by itself between single-waypoint goals, so the
        WaitAtWaypoint plugin duration must be 0 or every stop lasts
        wait_time + waypoint_pause_duration. Humble's WaitAtWaypoint reads
        its parameter once in initialize(), so this request may be accepted
        and still have no effect: if the pauses come out too long, set
        waypoint_pause_duration to 0 in the Nav2 params file.
        """
        if not self.disable_nav2_pause:
            self.get_logger().warn(
                "disable_nav2_pause is false: Nav2's WaitAtWaypoint pause adds "
                "up to every per-waypoint wait_time."
            )
            return

        service = f"{self.waypoint_follower_node}/set_parameters"
        self._param_client = self.create_client(SetParameters, service)

        if not self._param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"{service} unavailable, could not zero "
                f"'{self.nav2_pause_parameter}'. Set waypoint_pause_duration to 0 "
                "in the Nav2 params file, otherwise pauses are too long."
            )
            return

        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name=self.nav2_pause_parameter,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=0,
                ),
            )
        ]

        future = self._param_client.call_async(request)
        future.add_done_callback(self.nav2_pause_response_callback)

    def nav2_pause_response_callback(self, future):
        try:
            results = future.result().results
        except Exception as e:
            self.get_logger().warn(f"Could not zero the Nav2 waypoint pause: {e}")
            return

        if results and results[0].successful:
            self.get_logger().info(
                f"Requested {self.waypoint_follower_node} "
                f"'{self.nav2_pause_parameter}' = 0. Humble caches this value at "
                "configure time, so check the first pause length on the rover."
            )
        else:
            reason = results[0].reason if results else "no result"
            self.get_logger().warn(
                f"Nav2 refused '{self.nav2_pause_parameter}' = 0 ({reason}). "
                "Set waypoint_pause_duration to 0 in the Nav2 params file."
            )

    # =====================================================================
    # Costmap clearing between retries
    # =====================================================================
    def clear_costmaps(self):
        """
        Fire-and-forget clear of both costmaps.

        A planner failure caused by a stale lethal cell around the rover
        repeats identically on every retry, so the costmaps are wiped before
        the next attempt. Note this also drops the accumulated live patch in
        the global costmap; that data is rebuilt as soon as the local
        traversability map publishes again.
        """
        if not self.clear_costmaps_on_retry:
            return

        for name, client in (
            ("local", self._clear_local_client),
            ("global", self._clear_global_client),
        ):
            if not client.service_is_ready():
                self.get_logger().warn(
                    f"{name} costmap clear service not available, skipping it"
                )
                continue

            client.call_async(ClearEntireCostmap.Request())
            self.get_logger().info(f"Requested {name} costmap clear before retry")

    # =====================================================================
    # Mission sequencing
    # =====================================================================
    #
    # One FollowWaypoints goal per waypoint. Nav2's waypoint_follower only
    # knows about a single pose at a time, which is what makes a different
    # pause per waypoint possible.
    #
    # A waypoint is only given up on once max_attempts_per_waypoint attempts
    # have failed. With that parameter at 0 it is never given up on.
    #
    # =====================================================================
    def start_mission(self):
        if not self.waypoints:
            self.get_logger().warn("Waypoint list is empty, nothing to do.")
            self._mission_finished = True
            return

        self.get_logger().info("Waiting for FollowWaypoints action server...")
        self._action_client.wait_for_server()

        self.send_waypoint(0)

    def send_waypoint(self, index):
        # Moving on to a different waypoint resets the attempt counter;
        # re-sending the same index counts as another attempt.
        if index != self.curr_waypoint_index:
            self._attempt = 0

        self.curr_waypoint_index = index
        self._attempt += 1

        spec = self.specs[index]

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.waypoints[index]]

        limit = "inf" if self.max_attempts <= 0 else str(self.max_attempts)
        self.get_logger().info(
            f"Sending {self.waypoint_label(index)} to Nav2 in ROS frame "
            f"'{self.map_frame}': x={spec.x:.2f}, y={spec.y:.2f}, "
            f"pause_on_arrival={spec.wait_time:.1f} s "
            f"[attempt {self._attempt}/{limit}]"
        )

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.handle_failure(self.curr_waypoint_index, "goal rejected by Nav2")
            return

        self._goal_handle = goal_handle
        self.get_logger().info(
            f"{self.waypoint_label(self.curr_waypoint_index)} accepted"
        )

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        index = self.curr_waypoint_index

        wrapped_result = future.result()
        result = wrapped_result.result
        missed = list(getattr(result, "missed_waypoints", []))

        reached = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and not missed

        if not reached:
            self.handle_failure(
                index,
                f"status={wrapped_result.status}, missed={missed}",
            )
            return

        self.report_waypoint_reached(index)
        self.start_pause(index)

    # =====================================================================
    # Failure handling: retry instead of skipping
    # =====================================================================
    def handle_failure(self, index, reason):
        """
        Called for every way a waypoint can fail. Re-sends the same waypoint
        unless the attempt budget is exhausted.
        """
        budget_left = self.max_attempts <= 0 or self._attempt < self.max_attempts

        if budget_left:
            limit = "inf" if self.max_attempts <= 0 else str(self.max_attempts)
            self._total_retries += 1

            self.get_logger().warn(
                f"{self.waypoint_label(index)} FAILED ({reason}) on attempt "
                f"{self._attempt}/{limit}, retrying in {self.retry_delay:.1f} s"
            )

            self.clear_costmaps()
            self.schedule_retry(index)
            return

        # Attempt budget exhausted. Only here is a waypoint given up on.
        self.missed_waypoints.append(index)
        self.get_logger().error(
            f"{self.waypoint_label(index)} ABANDONED after {self._attempt} "
            f"attempts (last reason: {reason})"
        )

        if self.wait_on_failed_waypoint:
            self.start_pause(index)
        else:
            self.advance(index)

    def schedule_retry(self, index):
        self.cancel_retry_timer()

        self._retry_index = index
        self._retry_timer = self.create_timer(
            max(self.retry_delay, 0.1),
            self.retry_timer_callback,
        )

    def retry_timer_callback(self):
        index = self._retry_index
        self.cancel_retry_timer()

        if index is None or self._mission_finished:
            return

        self.send_waypoint(index)

    def cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)

        self._retry_timer = None
        self._retry_index = None

    def start_pause(self, index):
        wait_time = self.specs[index].wait_time

        if wait_time <= 0.0:
            self.advance(index)
            return

        self._pause_index = index
        self._pause_end_time = self.get_clock().now() + Duration(seconds=wait_time)
        self._pause_timer = self.create_timer(wait_time, self.pause_done_callback)

        self.get_logger().info(
            f"Pausing {wait_time:.1f} s at {self.waypoint_label(index)}"
        )

    def pause_done_callback(self):
        index = self._pause_index

        self._pause_timer.cancel()
        self.destroy_timer(self._pause_timer)
        self._pause_timer = None
        self._pause_index = None
        self._pause_end_time = None

        self.get_logger().info(f"Pause over at {self.waypoint_label(index)}")
        self.advance(index)

    def pause_remaining(self):
        """
        Seconds left in the current pause, or None when the rover is driving.
        """
        if self._pause_end_time is None:
            return None

        remaining = (self._pause_end_time - self.get_clock().now()).nanoseconds / 1e9
        return max(remaining, 0.0)

    def advance(self, index):
        next_index = index + 1

        if next_index < len(self.waypoints):
            self.send_waypoint(next_index)
            return

        self.curr_waypoint_index = len(self.waypoints)
        self._attempt = 0
        self._mission_finished = True
        self.cancel_retry_timer()

        if self.missed_waypoints:
            self.get_logger().warn(
                "\n"
                "========================================\n"
                f"  ROUTE FINISHED, MISSED WAYPOINTS: "
                f"{[i + 1 for i in self.missed_waypoints]}\n"
                f"  TOTAL RETRIES: {self._total_retries}\n"
                "========================================"
            )
        else:
            self.get_logger().info(
                "\n"
                "========================================\n"
                "  ALL WAYPOINTS COMPLETED\n"
                f"  TOTAL RETRIES: {self._total_retries}\n"
                "========================================"
            )

    def report_waypoint_reached(self, waypoint_index):
        erc_no = self.erc_number(waypoint_index)

        if erc_no is not None:
            headline = f"  ERC WAYPOINT {erc_no} REACHED (WP {waypoint_index + 1})"
        else:
            headline = f"  INTERMEDIATE WAYPOINT {waypoint_index + 1} REACHED"

        attempts_txt = ""
        if self._attempt > 1:
            attempts_txt = f"  AFTER {self._attempt} ATTEMPTS\n"

        self.get_logger().info(
            "========================================\n"
            "\n"
            "\n"
            "\n"
            "\n"
            "\n"
            "\n"
            f"{headline}\n"
            f"{attempts_txt}"
            f"  PAUSE: {self.specs[waypoint_index].wait_time:.1f} s\n"
            "\n"
            "\n"
            "\n"
            "\n"
            "\n"
            "========================================"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()