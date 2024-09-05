import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Point
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares

import numpy as np
from scipy import stats

def euclidean_distance(x1, y1, x2, y2):
    p1 = np.array((x1 ,y1))
    p2 = np.array((x2, y2))
    return np.linalg.norm(p1-p2)
    
# Mean Square Error
# locations: [ (lat1, long1), ... ]
# distances: [ distance1, ... ]
def mse(x, locations, distances):
    mse = 0.0
    for location, distance in zip(locations, distances):
        distance_calculated = euclidean_distance(x[0], x[1], location[0], location[1])
        mse += math.pow(distance_calculated - distance, 2.0)
    return mse / len(distances)


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.declare_parameter('sim', False) 
        self.declare_parameter('initial_pose', 'start') 
        self.declare_parameter('x', 0.0) 
        self.declare_parameter('y', 0.0) 

        sim = self.get_parameter('sim').get_parameter_value().bool_value 
        initial_pose = self.get_parameter('initial_pose').get_parameter_value().string_value 
        x = self.get_parameter('x').get_parameter_value().double_value 
        y = self.get_parameter('y').get_parameter_value().double_value 

        self.x_estimate = 0.0
        self.y_estimate = 0.0

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.sim = sim
        self.subscription 

        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0.0

        # Initial guess for optimization
        if initial_pose == 'start':
            self.initial_estimate = np.array([0.0, 0.0])
        elif initial_pose == 'known':
            self.initial_estimate = np.array([x, y])
        else: # by default
            self.initial_estimate = np.array([0.0, 0.0])


        if self.sim:
            # known absolute positions of landmarks (only x,y)
            self.landmark_poses = [
                    (9.80, 0.00),
                    (9.80, 3.5),
                    (34.00, 1.50),
                    (23.63, -4.62),
                    (10.27, 9.76),
                    (10.10, -21.38),
                    (5.44, -15.17),
                    (31.00, -9.13),
                    (18.37, 11.00),
                    (1.36, 9.60),
                    (17.00, -22.46),
                    (19.63, -0.02),
                    (18.29, -13.90),
                    (3.02, -17.34)
                ]
            
        else: 
            self.landmark_poses=[(63., 21.33), 
                    (63., -2.37),
                    (63., -21.33)]


    def timer_callback(self):

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        # odom_msg.pose.pose.position = Point(self.x_estimate, self.y_estimate, 0.0)
        odom_msg.pose.pose.position = Point(x=self.count, y=self.count, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        odom_msg.pose.covariance = [0.0] * 36  

        # Twist
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [0.0] * 36  

        # Publish the message
        self.publisher_.publish(odom_msg)
        self.count += 0.1

        
    def listener_callback(self, msg):
        
        marker_ids = list(msg.marker_ids)

        base_pose_msg = PoseWithCovarianceStamped()
        base_pose_msg.header.stamp = self.get_clock().now().to_msg()
        base_pose_msg.header.frame_id = 'map'

        # Need 3 landmarks to estimate pose
        if len(marker_ids)>=3:

            self.get_logger().info('Estimating')

            distance_estimates = [np.linalg.norm([pose.position.x, pose.position.y]) for pose in msg.poses]
            landmarks_ordered = [self.landmark_poses[i] for i in marker_ids]

            base_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'lm', args=(landmarks_ordered, distance_estimates))

            self.initial_estimate = base_estimate.x

            # Only care about x and y
            self.x_estimate = base_estimate.x[0] 
            self.y_estimate = base_estimate.x[1]

            
        else:
            self.get_logger().info('Not enough markers detected')


    def cost_function(self, estimate, landmarks, measured_distances):
        x_r, y_r = estimate
        residuals = []
        for (x_i, y_i), d_i in zip(landmarks, measured_distances):
            predicted_distance = np.sqrt((x_r - x_i)**2 + (y_r - y_i)**2)
            residuals.append(predicted_distance - d_i)
        return residuals


def main(args=None):
    rclpy.init(args=args)
    my_node = PoseEstimatorNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
