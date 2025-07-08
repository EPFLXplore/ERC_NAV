import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
from tf2_ros import Buffer, TransformListener
import tf_transformations
from nav_msgs.msg import Odometry


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.send_waypoints()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = self.create_waypoints()

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

    def wheel_odom_callback(self, msg):
        # Extract the linear velocity from the odometry message
        self.current_speed = msg.twist.twist.linear.x
        
    def print_current_position(self):
        try:
            # Get the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            self.get_logger().info(f'POSITION in ERC MAP: X={position.x}, Y={position.y}')
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
        if self.current_speed > 0:
            eta = distance / self.current_speed
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

        def make_pose(x, y, yaw_deg):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Convert yaw (in degrees) to quaternion

            q = quaternion_from_euler(0, 0, math.radians(yaw_deg))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            return pose

        # Hardcoded waypoints (x, y, yaw_deg) in the ERC map frame !!!
        # THE LAST WAYPOINT NEEDS TO BE THE START POSITION BECAUSE WE NEED TO GO THERE TO COMPLETE THE TASK
        waypoint_list = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 90.0),
            (0.0, 1.0, 180.0),
            (5.3, -2.7),
            (0.0, 0.0, 0.0),
        ]

        for x, y, yaw in waypoint_list:
            poses.append(make_pose(x, y, yaw))

        return poses

    def feedback_callback(self, feedback):
        current_wp = feedback.feedback.current_waypoint
        self.curr_waypoint_index = current_wp
        self.get_logger().info(f'Reached waypoint index: {current_wp}')

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