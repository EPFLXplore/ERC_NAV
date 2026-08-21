#!/usr/bin/env python3

# Work around the removed `np.float` alias in NumPy >= 1.20
import numpy as _np
if not hasattr(_np, "float"):
    _np.float = float

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
import tf_transformations


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

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
        self.declare_parameter("wheel_odom_topic", "/fused_nav_ekf_odom")
        self.declare_parameter("acceptance_radius", 0.15)
        self.declare_parameter("visualization_rate_hz", 2.0)

        self.waypoint_input_coordinates = self.get_parameter(
            "waypoint_input_coordinates"
        ).value

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.wheel_odom_topic = self.get_parameter("wheel_odom_topic").value
        self.acceptance_radius = float(self.get_parameter("acceptance_radius").value)
        self.visualization_rate_hz = float(
            self.get_parameter("visualization_rate_hz").value
        )

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

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self.waypoints = self.create_waypoints()
        self.curr_waypoint_index = 0
        self.current_speed = 0.0

        # ---------------- Nav2 action client ----------------
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

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
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self.wheel_odom_callback,
            10,
        )

        # ---------------- Timers ----------------
        viz_period = 1.0 / max(self.visualization_rate_hz, 0.1)
        self.viz_timer = self.create_timer(viz_period, self.publish_visualizations)
        self.status_timer = self.create_timer(1.0, self.print_current_position)

        # ---------------- Send goal ----------------
        self.send_waypoints()

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
        # These values are interpreted according to:
        #
        #   self.waypoint_input_coordinates
        #
        # If waypoint_input_coordinates == "map":
        #   waypoint_list contains:
        #       x_map, y_map, yaw_map
        #
        # If waypoint_input_coordinates == "erc_map":
        #   waypoint_list contains:
        #       x_erc, y_erc, yaw_erc
        #
        # The values actually sent to Nav2 are always converted to ROS/Nav2 map.
        #
        # =====================================================================

        # waypoint_list = [
        #     (6.6, 0.0, 0.0),
        #     (6.6, 4.2, 0.0),
        #     (13.3, 4.9, 0.0),
        #     (13.3, 14.1, 0.0),
        #     (2.8, 14.1, 0.0),
        #     (2.8, 11.0, 0.0),
        #     (-0.7, 11.0, 0.0),
        #     (0.0, 0.0, 0.0),
        # ]

        # Avec z
        # W1 = (9.759, 1.951, -0.053)
        # W2 = (19.499, 4.170, 0.105)
        # W3 = (23.149, 7.013, 0.205)
        # W4 = (18.665, -1.701, -0.156)
        # W5 = (19.034, -5.339, -0.259)
        # W6 = (8.197, -7.675, -0.179)
        # W7 = (7.682, -2.108, -0.163)
        # W8 = (6.790, 5.939, -0.065)
        # W9 = (13.092, 5.088, 0.011)
        # P1 = (25.901, -5.997, -0.160)

        # Sans z
        W1 = (9.759, 1.951, 0.0)
        W2 = (19.499, 4.170, 0.0)
        W3 = (23.149, 7.013, 0.0)
        W4 = (18.665, -1.701, 0.0)
        W5 = (19.034, -5.339, 0.0)
        W6 = (8.197, -7.675, 0.0)
        W7 = (7.682, -2.108, 0.0)
        W8 = (6.790, 5.939, 0.0)
        W9 = (13.092, 5.088, 0.0)
        P1 = (25.901, -5.997, 0.0)

        waypoint_list = [W1, W2, W3, W4]

        waypoint_list = [
            (2.2, 0.0, 0.0),
            (1.6, -5.7, 0.0),
            (-1.4, -5.7, 0.0),
            (0.0, 0.0, 0.0),
        ]

        # waypoint_list = [
        #     (1.6, -5.7, 0.0),
        # ]
        

        for i, (x_in, y_in, yaw_in) in enumerate(waypoint_list):
            x_map, y_map, yaw_map = self.input_to_map(x_in, y_in, yaw_in)
            x_erc, y_erc, yaw_erc = self.map_to_erc(x_map, y_map, yaw_map)

            poses.append(make_pose_in_ros_map(x_map, y_map, yaw_map))

            self.get_logger().info(
                f"WP {i:02d} | "
                f"input[{self.waypoint_input_coordinates}]: "
                f"x={x_in:.2f}, y={y_in:.2f}, yaw={yaw_in:.2f} | "
                f"sent_to_nav2[map]: "
                f"x={x_map:.2f}, y={y_map:.2f}, yaw={yaw_map:.2f} | "
                f"erc_equivalent: "
                f"x={x_erc:.2f}, y={y_erc:.2f}, yaw={yaw_erc:.2f}"
            )

        return poses

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
            r, g, b, a = self.waypoint_color(i)
            is_current = i == self.curr_waypoint_index

            x_map = wp.pose.position.x
            y_map = wp.pose.position.y

            # ---------------- Orientation arrow ----------------
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
            else:
                arrow.scale.x = 0.45
                arrow.scale.y = 0.15
                arrow.scale.z = 0.15

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
    def wheel_odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

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

        if self.curr_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.curr_waypoint_index].pose.position

            dx = target.x - position.x
            dy = target.y - position.y
            distance = math.sqrt(dx * dx + dy * dy)

            target_x_erc, target_y_erc, _ = self.map_to_erc(
                target.x,
                target.y,
                0.0,
            )

            self.get_logger().info(
                f"TARGET WP {self.curr_waypoint_index} | "
                f"map: x={target.x:.2f}, y={target.y:.2f} | "
                f"erc_map: x={target_x_erc:.2f}, y={target_y_erc:.2f} | "
                f"distance={distance:.2f} m"
            )

            if self.current_speed > 0.01:
                eta = distance / self.current_speed
                self.get_logger().info(
                    f"ETA to waypoint {self.curr_waypoint_index}: {eta:.2f} s"
                )
            else:
                self.get_logger().info("ETA: rover speed too low")

        else:
            self.get_logger().info("***** NO MORE WAYPOINTS, TASK FINISHED *****")

    # =====================================================================
    # Nav2 FollowWaypoints action
    # =====================================================================
    def send_waypoints(self):
        self.get_logger().info("Waiting for FollowWaypoints action server...")
        self._action_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info(
            f"Sending {len(self.waypoints)} waypoints to Nav2 in ROS frame '{self.map_frame}'."
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        current_wp = feedback.feedback.current_waypoint
        self.curr_waypoint_index = int(current_wp)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"FollowWaypoints finished with result: {result}")


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
