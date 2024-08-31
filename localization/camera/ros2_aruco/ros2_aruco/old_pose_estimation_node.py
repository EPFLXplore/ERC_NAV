import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
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
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'aruco_rover_pose_estimate', 10)
      

        self.subscription 
        # Initial guess for optimization
        self.initial_estimate = np.array([0.0, 0.0])
        # known absolute positions of landmarks (only x,y for now)
        self.landmark_poses=[(-2.5, 0), # simple sim
                            (-3.3, -1),
                            (-2.5, 1.2)]

        self.landmark_poses=[(1.66, 0), # DLL
                            (3.06, -0.22),
                            (2.64, -1.42),
                            (2.53, -2.46)]
        
        # self.cam_to_base_transform = [0.25, 0.02]
        self.cam_to_base_transform = [0.25, 0.02] # sim
        self.base_height = 0.5
        


    def listener_callback(self, msg):
        
        marker_ids = list(msg.marker_ids)
        # self.get_logger().info('marker_ids: %s' % marker_ids )

        base_pose_msg = PoseWithCovarianceStamped()
        base_pose_msg.header.stamp = self.get_clock().now().to_msg()
        base_pose_msg.header.frame_id = 'map'


        if len(marker_ids)>=3:

            self.get_logger().info('Estimating')


            distance_estimates = [np.linalg.norm([pose.position.x, pose.position.y]) for pose in msg.poses]
            landmarks_ordered = [self.landmark_poses[50-i] for i in marker_ids]
            # self.get_logger().info('landmarks: %s' % landmarks_ordered )
            # self.get_logger().info('estimate: %s' % distance_estimates )

            cam_pose_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'lm', args=(landmarks_ordered, distance_estimates))


            self.initial_estimate = cam_pose_estimate.x

            # Use transform from camera frame to baselink to find rover position
            base_pose_msg.pose.pose.position.x = cam_pose_estimate.x[0] #- self.cam_to_base_transform[0]
            base_pose_msg.pose.pose.position.y = cam_pose_estimate.x[1] #- self.cam_to_base_transform[1]
            base_pose_msg.pose.pose.position.z = self.base_height 

            base_pose_msg.pose.pose.orientation.x = 0.0
            base_pose_msg.pose.pose.orientation.y = 0.0
            base_pose_msg.pose.pose.orientation.z = 0.0
            base_pose_msg.pose.pose.orientation.w = 1.0

            covariance = np.zeros((6, 6))
            covariance[0, 0] = 0.5  # Variance in x
            covariance[1, 1] = 0.5  # Variance in y
            
            covariance[2, 2] = 0.1  # Variance in z
            covariance[3, 3] = 0.1  # Variance in roll
            covariance[4, 4] = 0.1  # Variance in pitch
            covariance[5, 5] = 0.1  # Variance in yaw

            base_pose_msg.pose.covariance = covariance.flatten().tolist()
            self.publisher_.publish(base_pose_msg)

            
        else:
            self.get_logger().info('Not enough markers detected')




        # # Initial point: the point with the closest distance
        # min_distance  = float('inf')
        # closest_location = None
        # for member in data:
        #     # A new closest point!
        #     if member['distance'] < min_distance:
        #         min_distance = member['distance']
        #         closest_location = member['location']
        # initial_location = closest_location



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
