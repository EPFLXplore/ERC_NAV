import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Point
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares

#for the yaw estimation we use itertools combinations for generating every pair of arucos possible
from itertools import combinations
from scipy.spatial.transform import Rotation as R

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

def yaw_to_quat(yaw):
    quat = R.from_euler('z', yaw).as_quat()  # x, y, z, w
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

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
        self.yaw_estimate = 0.0

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.sim = sim
        self.subscription 

        # timer_period = 2.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initial guess for optimization
        if initial_pose == 'start':
            self.initial_estimate = np.array([0.0, 0.0])
        elif initial_pose == 'known':
            self.initial_estimate = np.array([x, y])
        else: # by default
            self.initial_estimate = np.array([0.0, 0.0])


        #if self.sim: #fuck the simulation
            # known absolute positions of landmarks (only x,y)


        # ArUco ID 51 → index 0
        # ArUco ID 52 → index 1
        # ArUco ID 53 → index 2
        # ...
        #the position is relative to the map frame which is given to us by the ERC
        self.landmark_poses = [
                (0.0, 5.05),
                (0.0, 1.25),
                (0.0, 0.0),
                (11.28846, 11.16958),
                (2.93746, 6.18086),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
            ]
            
        # else: 
        #     self.landmark_poses=[(63., 21.33), 
        #             (63., -2.37),
        #             (63., -21.33)]


    # def timer_callback(self):

    #     odom_msg = Odometry()
    #     odom_msg.header.stamp = self.get_clock().now().to_msg()
    #     odom_msg.header.frame_id = 'map'

    #     odom_msg.pose.pose.position.x = self.x_estimate
    #     odom_msg.pose.pose.position.y = self.y_estimate
    #     odom_msg.pose.pose.position.z = 0.0

    #     odom_msg.pose.pose.orientation = yaw_to_quat(self.yaw_estimate)
    #     odom_msg.pose.covariance = [0.00001] * 36

    #     # Publish the message
    #     self.publisher_.publish(odom_msg)

        
    def listener_callback(self, msg):
        
        marker_ids = list(msg.marker_ids)
        #print(f"marker ids pose est: {marker_ids}, msg: {msg.marker_ids}")

        base_pose_msg = PoseWithCovarianceStamped()
        base_pose_msg.header.stamp = self.get_clock().now().to_msg()
        #base_pose_msg.header.frame_id = 'map'   #put me back later
        base_pose_msg.header.frame_id = 'map'


        # Estimate the yaw using the detected arucos

        #for every pair of arucos (limited to 5) )calculate atan2(ar2_map_y - ar1_map_y, ar2_map_x - ar1_map_x)
        #then using the aruco message use the received positions of the same arucos in the rover frame:
        #atan2(ar2_rover_y - ar1_rover_y, ar2_rover_x - ar1_rover_x)

        aruco_idx_pairs = list(combinations(range(len(marker_ids)), 2))
        yaw_offsets = [] #store the offsets of the rover odometry yaw compared to map yaw

        for i, j in aruco_idx_pairs:
            id1 = marker_ids[i]
            id2 = marker_ids[j]

            # Skip if either landmark is invalid
            if id1 >= len(self.landmark_poses) or id2 >= len(self.landmark_poses):
                continue
            
            lm1 = self.landmark_poses[id1]
            lm2 = self.landmark_poses[id2]
            if lm1[0] > 9000 or lm2[0] > 9000:  #just check if they have actually been hardcoded in the code
                continue

            dx_map = lm2[0] - lm1[0]
            dy_map = lm2[1] - lm1[1]
            angle_map = math.atan2(dy_map, dx_map)

            pose1 = msg.poses[i].position
            pose2 = msg.poses[j].position

            dx_rover = pose2.x - pose1.x
            dy_rover = pose2.y - pose1.y
            angle_rover = math.atan2(dy_rover, dx_rover)
            
            yaw_diff = angle_map - angle_rover
            yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
            yaw_offsets.append(yaw_diff)
            self.get_logger().info(f"--> Yaw (deg): {yaw_diff:.3f} for ids {id1}-{id2}")

        
        if len(yaw_offsets) > 0:
            avg_yaw = np.arctan2(np.mean(np.sin(yaw_offsets)), np.mean(np.cos(yaw_offsets)))
            self.get_logger().info(f"--> Estimated Yaw (deg): {(avg_yaw*180/3.141592):.3f}")
            self.yaw_estimate = avg_yaw


        # Need 3 landmarks to triangulate pose
        if len(marker_ids)>=3:

            #self.get_logger().info('Estimating')

            distance_estimates = [np.linalg.norm([pose.position.x, pose.position.y]) for pose in msg.poses]
            landmarks_ordered = [self.landmark_poses[i] for i in marker_ids]

            base_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'lm', args=(landmarks_ordered, distance_estimates))

            self.initial_estimate = base_estimate.x

            # Only care about x and y
            self.x_estimate = base_estimate.x[0] 
            self.y_estimate = base_estimate.x[1]

            self.get_logger().info(f"--> X: {self.x_estimate}")
            self.get_logger().info(f"--> Y: {self.y_estimate}")

        # else:
        #     self.get_logger().info('Not enough markers detected')


        #PUBLISH THE GLOBAL POSEEEEEE
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = self.x_estimate
        odom_msg.pose.pose.position.y = self.y_estimate
        odom_msg.pose.pose.orientation = yaw_to_quat(self.yaw_estimate)
        odom_msg.pose.covariance = [0.002] * 36

        self.publisher_.publish(odom_msg)



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
