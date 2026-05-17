#!/usr/bin/env python3

# Work around the removed `np.float` alias in NumPy ≥1.20
import numpy as _np
if not hasattr(_np, 'float'):
    _np.float = float

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
from tf2_ros import Buffer, TransformListener
import tf_transformations
from nav_msgs.msg import Odometry

#to visualize waypoints in rviz2:
# ADD imports
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.send_waypoints()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = self.create_waypoints()

        #for waypoints visualization:
        viz_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.path_pub = self.create_publisher(Path, '/waypoints_path', viz_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', viz_qos)
        self.posearray_pub = self.create_publisher(PoseArray, '/waypoints_posearray', viz_qos)

        # publish once (or you can also republish on a timer if you prefer)
        self.publish_visualizations()


        self.curr_waypoint_index = 0

        #timer to periodically print the current position and distance to the current target waypoint
        self.timer = self.create_timer(1.0, self.print_current_position)

        #subscribe to /wheel_odom to get speed
        self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_odom_callback,
            10
        )

        self.current_speed = 0.0

    def publish_visualizations(self):
        now = self.get_clock().now().to_msg()

        # 1) Path polyline
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = now
        path.poses = self.waypoints  # already PoseStamped
        self.path_pub.publish(path)

        # 2) Markers: arrows at each waypoint, index labels, and a connecting line
        marr = MarkerArray()

        # Connecting line
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = now
        line.ns = 'waypoints_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08  # line width (m)
        line.color.g = 1.0
        line.color.a = 0.8
        line.pose.orientation.w = 1.0
        for p in self.waypoints:
            line.points.append(Point(x=p.pose.position.x, y=p.pose.position.y, z=0.05))
        marr.markers.append(line)

        # Waypoint arrows + labels
        for i, ps in enumerate(self.waypoints, start=1):
            # Arrow showing orientation
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'waypoint_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = ps.pose
            arrow.scale.x = 0.4   # shaft length
            arrow.scale.y = 0.2  # shaft diameter
            arrow.scale.z = 0.2  # head diameter
            arrow.color.r = 0.1
            arrow.color.g = 0.8
            arrow.color.b = 1.0
            arrow.color.a = 1.0
            marr.markers.append(arrow)

            # Text label with index
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'waypoint_labels'
            text.id = 1000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = ps.pose.position.x
            text.pose.position.y = ps.pose.position.y
            text.pose.position.z = ps.pose.position.z + 0.5
            text.scale.z = 0.4  # text height (m)
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'{i}'
            marr.markers.append(text)

        self.marker_pub.publish(marr)

        # 3) PoseArray (optional)
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = now
        pa.poses = [p.pose for p in self.waypoints]
        self.posearray_pub.publish(pa)


    def wheel_odom_callback(self, msg):
        # Extract the linear velocity from the odometry message
        self.current_speed = msg.twist.twist.linear.x
        
    def print_current_position(self):
        try:
            # Get the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            self.get_logger().info(f'POSITION in erc_map: X={position.x}, Y={(-1.0)*position.y}')
        except Exception as e:
            self.get_logger().error(f'could NOT get map->base_link TF: {e}')

        #check the distance to the current target waypoint
        if self.curr_waypoint_index < len(self.create_waypoints()):
            target_waypoint = self.waypoints[self.curr_waypoint_index]
            target_position = target_waypoint.pose.position
            distance = math.sqrt((target_position.x - position.x) ** 2 + (target_position.y - position.y) ** 2)
            self.get_logger().info(f'Distance to waypoint {self.curr_waypoint_index}: {distance:.2f} meters')

        else:
            self.get_logger().info('***** !!!! NO MORE WAYPOINTS, TASK FINISHED !!!! ******')

        #compute ETA (Estimated Time of Arrival) to the current target waypoint
        if abs(self.current_speed) > 0:
            eta = abs(distance / self.current_speed)
            self.get_logger().info(f'ETA to waypoint {self.curr_waypoint_index}: {eta:.2f} seconds')


    def send_waypoints(self):
        # Wait for the action server
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.create_waypoints()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def create_waypoints(self):
        poses = []

        def make_pose(x, y, yaw_rad):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Convert yaw (in degrees) to quaternion

            q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            return pose

        # Hardcoded waypoints (x, y, yaw_rad) in the ERC map frame !!!
        # THE LAST WAYPOINT NEEDS TO BE THE START POSITION BECAUSE WE NEED TO GO THERE TO COMPLETE THE TASK


        # waypoint_list = [ #(x, y, yaw) in the ERC_MAP FRAME !!!!!
        #     (12.328, 6.779,    0.0),
        #     (6.807, 10.375, -3.141),
        #     (15.116, -3.085, -2.61),
        #     (19.727, 5.238, 0.785),
        #     (18.6625, 10.8159, 1.57)
        # ]

        # waypoint_list = [ #(x, y, yaw) in the ERC_MAP FRAME !!!!!
        #     (13.2623, 8.671, 3.1415),
        #     (6.8073, 10.1746, 2.8),
        #     (0.1, 8.37, -1.57),
        #     (1.4, 1.0, -0.17),
        #     (10.0126, 0.0, 0.0),
        #     (15.66, -0.76, 0.0),
        #     (25.2349, 2.0235, 0.0),
        #     (20.0, 5.75, 0.785),
        #     (24.4818, 7.9578, 0.785),
        #     (20.0, 5.75, 0.785),
        #     (18.6625, 10.8159, 1.57)
        # ]

        # waypoint_list = [ #(x, y, yaw) in the ERC_MAP FRAME !!!!! y is flip
        #     (3.0, 0.0, 0.0),
        #     (3.0, -4.0, 0.0),
        #     (3.0, 0.0, 0.0),
        #     (0.0, 0.0, 0.0),
        # ]

        waypoint_list = [ 
            (11.2, 5.2, 0.0),
            (11.2, 7.2, 0.0),
            (2.94, 7.2, 0.0),
            (12.1, 7.2, 0.0),
            (9.8, 20.8, 0.0),
            (8.1, 24.05, 0.0),
            (5.0, 22.25, 0.0),
            (8.1, 24.05, 0.0),
            (9.8, 20.8, 0.0),
            (12.1, 7.2, 0.0),
            (11.2, 7.2, 0.0),
            (11.2, 5.2, 0.0),
            (0.0, 0.0, 0.0)
        ]



        for x, y, yaw in waypoint_list:
            poses.append(make_pose(x, y, yaw)) # THIS IS BRUGG
            # poses.append(make_pose(x, (-1.0)*y, (-1.0)*yaw)) THIS IS ERC
            self.get_logger().info(f"waypoint in map NOT erc_map: X={x}, Y={-y}, yaw : {-yaw}")

        return poses

    def feedback_callback(self, feedback):
        current_wp = feedback.feedback.current_waypoint
        self.curr_waypoint_index = current_wp
        #self.get_logger().info(f'Reached waypoint index: {current_wp}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'FollowWaypoints finished with result code: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



############################################################################


