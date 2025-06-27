import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares

#for the yaw estimation we use itertools combinations for generating every pair of arucos possible
from itertools import combinations
from scipy.spatial.transform import Rotation as R

import numpy as np

#for the tf map->odom transform

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


# def euclidean_distance(x1, y1, x2, y2):
#     p1 = np.array((x1 ,y1))
#     p2 = np.array((x2, y2))
#     return np.linalg.norm(p1-p2)
    
# # Mean Square Error
# # locations: [ (lat1, long1), ... ]
# # distances: [ distance1, ... ]
# def mse(x, locations, distances):
#     mse = 0.0
#     for location, distance in zip(locations, distances):
#         distance_calculated = euclidean_distance(x[0], x[1], location[0], location[1])
#         mse += math.pow(distance_calculated - distance, 2.0)
#     return mse / len(distances)

def yaw_to_quat(yaw):
    quat = R.from_euler('z', yaw).as_quat()  # x, y, z, w
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.x_estimate = 0.0
        self.y_estimate = 0.0
        self.yaw_estimate = 0.0
        self.triangulated_new_pose = False
        self.triangulated_new_xy = False
        self.measured_new_yaw = False
        self.time_of_last_pose = 0.0
        self.time_of_last_yaw_meas = 0.0

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/fused_nav_ekf_odom',   
            self.odometry_callback,
            10)
        self.odometry_subscription

        self.odom_pos_x = 0.0
        self.odom_pos_y = 0.0
        self.odom_yaw = 0.0

        #map->base_link pose in between updates
        self.curr_map_odom_base_x = 0.0
        self.curr_map_odom_base_y = 0.0 
        self.curr_map_odom_base_yaw = 0.0

        self.high_cov = [1e6 if i in (0,7,35) else 0. for i in range(36)]
        self.low_cov  = [0.005 if i in (0,7) else 0.002 if i==35 else 0. for i in range(36)]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        #create a timer to publish the map->odom transform at a fixed rate to avoid 
        #bugs and warnings
        self.prev_map_odom_tf = None
        self.tf_timer = self.create_timer(0.1, self.republish_map_odom_transform)

        self.publisher_ = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.subscription 

        # timer_period = 2.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initial guess for optimization
        self.initial_estimate = np.array([0.0, 0.0])



        # ArUco ID 51 → index 0
        # ArUco ID 52 → index 1
        # ArUco ID 53 → index 2
        # ...
        #the position is relative to the map frame which is given to us by the ERC
        self.landmark_poses = [
                (-2.8, 3.86),
                (-2.8, -1.94),
                (-2.8, 2.36),
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
                (999999, 999999),
                (999999, 999999),
            ]
        
    def odometry_callback(self, msg):
        #odom->base_link pose from the EKF
        self.odom_pos_x = msg.pose.pose.position.x
        self.odom_pos_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        # Convert quaternion to yaw angle
        _, _, self.odom_yaw = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz')
        
    def republish_map_odom_transform(self):
        # Publish the last stored map->odom transform
        if self.prev_map_odom_tf is not None:
            # Update the timestamp of the transform
            self.prev_map_odom_tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)

    def listener_callback(self, msg):


        #get the current map->odom transform then add it to current odom->base_link from the EKF
        #this will give us the current map->base_link pose
        try:
            now = self.get_clock().now().to_msg()
            transform = self.tf_buffer.lookup_transform('map','odom', now, timeout=rclpy.duration.Duration(seconds=0.2))
            self.get_logger().info("here1")
            odom_base_pose_map = (
                transform.transform.translation.x + self.odom_pos_x,
                transform.transform.translation.y + self.odom_pos_y
            )
            self.curr_map_odom_base_x = odom_base_pose_map[0]
            self.curr_map_odom_base_y = odom_base_pose_map[1]
            self.curr_map_odom_base_yaw = self.odom_yaw + R.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]).as_euler('xyz')[2]  # yaw angle
            self.get_logger().info("here2")
        except TransformException as e:
            if self.prev_map_odom_tf is None:
                self.curr_map_odom_base_x = self.odom_pos_x
                self.curr_map_odom_base_y = self.odom_pos_y
                self.curr_map_odom_base_yaw = self.odom_yaw
                #odom_base_pose_map = (self.curr_map_odom_base_x, self.curr_map_odom_base_y)
                self.get_logger().warn("Initializing TF: assuming MAP = ODOM.")


        marker_ids = list(msg.marker_ids)

        # Estimate the yaw using the detected arucos

        #for every pair of arucos (limited to 5) )calculate atan2(ar2_map_y - ar1_map_y, ar2_map_x - ar1_map_x)
        #then using the aruco message use the received positions of the same arucos in the rover frame:
        #atan2(ar2_rover_y - ar1_rover_y, ar2_rover_x - ar1_rover_x)

        aruco_idx_pairs = list(combinations(range(len(marker_ids)), 2))
        yaw_offsets = [] #store the offsets of the rover odometry yaw compared to map yaw


        #validate markers
        valid_markers = []
        for idx, pose in zip(msg.marker_ids, msg.poses):
            if idx < len(self.landmark_poses) and self.landmark_poses[idx][0] < 300 and self.landmark_poses[idx][1] < 300:
                valid_markers.append((idx, pose))

        if len(valid_markers) < 2:
            self.get_logger().warn("Not enough valid markers detected for yaw estimation.")
            return

        for i, j in aruco_idx_pairs:
            id1 = marker_ids[i]
            id2 = marker_ids[j]

            # Skip if either landmark is invalid
            if id1 >= len(self.landmark_poses) or id2 >= len(self.landmark_poses):
                continue
            
            lm1 = self.landmark_poses[id1]
            lm2 = self.landmark_poses[id2]
            if lm1[0] > 300 or lm2[0] > 300:  #spicy magic number =300meters just to check if they have actually been hardcoded in the code
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
            #self.get_logger().info(f"--> Yaw (deg): {(yaw_diff*180/3.141592):.3f} for ids [{id1}-{id2}]")

        
        if len(yaw_offsets) > 0:
            avg_yaw = np.arctan2(np.mean(np.sin(yaw_offsets)), np.mean(np.cos(yaw_offsets)))
            self.yaw_estimate = avg_yaw
            self.measured_new_yaw = True
            self.time_of_last_yaw_meas = self.get_clock().now()
        


        # we can make a precise guess using only two landmarks because we know roughly where we are already
        # when the rover starts: We are at (0, 0) assuming we dont fuck up the rover placement at the start of the task.
        # this problem is basically first about finding the intersection of two circles
        # then finding the point that is closest to our current location (given by the current map->odom + odom->base_link transform).

        if(len(valid_markers) == 2):  
            #find the (x, y) coordinates of the two intersections of the circles

            #valid_marker[i] = (idx, pose)
            id1, pose1 = valid_markers[0]
            id2, pose2 = valid_markers[1]

            #positions of the centers of the center of the aruco boxes in the map frame
            X1, Y1 = self.landmark_poses[id1]
            X2, Y2 = self.landmark_poses[id2]
            
            #the radiuses are given by the norm of the Pose msg which gives the vector from base_link to the detected aruco tag
            X1_base, Y1_base = pose1.position.x, pose1.position.y
            X2_base, Y2_base = pose2.position.x, pose2.position.y

            #measured readiuses
            R1 = math.hypot(X1_base, Y1_base)
            R2 = math.hypot(X2_base, Y2_base)

            Dx = X2-X1
            Dy = Y2-Y1
            D = math.sqrt(Dx**2 + Dy**2)
            self.get_logger().info(f"theoretical dist btw arucos: {abs(D)}")


            #validate if the the distance between the two aruco tags matches what the ERC map says.
            D_meas_dx = X2_base - X1_base
            D_meas_dy = Y2_base - Y1_base
            D_meas = math.sqrt(D_meas_dx**2 + D_meas_dy**2)
            if D != 0:
                if((abs(D_meas - D)/D) > 0.2): #allow 10% variation of the theoretical lenghth in the measurements.
                    self.get_logger().warn(f"more than 30% variation of the theoreical aruco distance from measurements")
                    self.get_logger().info(f"measurement error: {abs(D_meas - D)}")
                    return

            self.get_logger().info(f"measurement error: {abs(D_meas - D)}")

            if not(D > R1 + R2) and not(D < math.fabs(R2 - R1)) and not(D == 0 and R1 == R2):

                chorddistance = (R1**2 - R2**2 + D**2)/(2*D)
                # distance from 1st circle's centre to the chord between intersects
                halfchordlength = math.sqrt(R1**2 - chorddistance**2)
                chordmidpointx = X1 + (chorddistance*Dx)/D
                chordmidpointy = Y1 + (chorddistance*Dy)/D

                I1 = (chordmidpointx + (halfchordlength*Dy)/D,
                      chordmidpointy - (halfchordlength*Dx)/D)
                theta1 = math.degrees(math.atan2(I1[1]-Y1, I1[0]-X1))

                I2 = (chordmidpointx - (halfchordlength*Dy)/D,
                     chordmidpointy + (halfchordlength*Dx)/D)
                theta2 = math.degrees(math.atan2(I2[1]-Y1, I2[0]-X1))
                if theta2 > theta1:
                    I1, I2 = I2, I1
                
                #self.get_logger().info(f"--> Intersect 1: {I1}, Intersect 2: {I2}")

                # Now we have two intersection points I1 and I2
                # We need to find the one that is closest to our current position

                #need to subscribe to the local EKF output, this will give us odom -> base_link
                #we also need to listen to the map -> odom transform
                #we can then compare the current pose estimate in the map frame with the two intersection points

                # Calculate distances to both intersection points
                dist_I1 = math.hypot(I1[0] - self.curr_map_odom_base_x, I1[1] - self.curr_map_odom_base_y)
                dist_I2 = math.hypot(I2[0] - self.curr_map_odom_base_x, I2[1] - self.curr_map_odom_base_y)

                if dist_I1 <= dist_I2:
                    self.x_estimate, self.y_estimate = I1
                    self.get_logger().info(f"--> Selected Intersect 1: {I1}")
                    self.triangulated_new_xy = True
                    self.time_of_last_pose = self.get_clock().now()

                else:
                    self.x_estimate, self.y_estimate = I2
                    self.get_logger().info(f"--> Selected Intersect 2: {I2}")
                    self.triangulated_new_xy = True
                    self.time_of_last_pose = self.get_clock().now()


        # if >=3 landmarks we can triangulate the pose using least squares
        if len(valid_markers)>=3:

            distance_estimates = [math.hypot(p.position.x, p.position.y) for _, p in valid_markers]
            landmarks_ordered = [self.landmark_poses[idx] for idx, _ in valid_markers]

            base_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'lm', args=(landmarks_ordered, distance_estimates))

            self.initial_estimate = base_estimate.x

            # Only care about x and y
            self.x_estimate = base_estimate.x[0] 
            self.y_estimate = base_estimate.x[1]

            self.get_logger().info(f"--> triang X: {self.x_estimate}")
            self.get_logger().info(f"--> triang Y: {self.y_estimate}")
            self.triangulated_new_pose = True
            self.triangulated_new_xy = True

            self.time_of_last_pose = self.get_clock().now()

        # else:
        #     self.get_logger().info('Not enough markers detected')

        # We need to filter the pose estimation because if we dont and are static(or relatively static)
        # measurement noise and vibrations will cause the pose to jump which is annyoing for control and nav2.
        # An easy way to do this is by tracking the time between the last pose estimate and the current one
        # and if the time is less that a certain threshold, we can apply a low pass filter to the pose estimate
        # However if the time is greater than the threshold we should not apply the filter
        # but we should still check if the pose estimate is valid (meaning not too far away from where we currently are according to the previous map->odom and the most recente odom->base_link pose)
        # if the error is too high we should not apply the filter becuase the odometry eventually needs to be corrected.

        if self.triangulated_new_xy or self.triangulated_new_pose or self.measured_new_yaw:
            #if the delta time since last pose is less than 0.5 second, apply low pass
            current_time = self.get_clock().now()
            delta_time = (current_time - self.time_of_last_pose).nanoseconds / 1e9  # convert to seconds
            delta_yaw_time = (current_time - self.time_of_last_yaw_meas).nanoseconds / 1e9  # convert to seconds

            if delta_time < 0.5:
                #self.get_logger().info(f"lowpass filtering the map->odom TF")
                # IIR 1st Order Low Pass Filter
                #H(e^jw) = alpha / (1 + (alpha - 1)e^(-jw))
                #w --> infinity => H = alpha
                #w --> 0 => H = 1
                #response time will depend on the frequency of the updates
                alpha = 0.2 #low alpha = more smoothing, high alpha = less smoothing
                self.x_estimate = alpha * self.x_estimate + (1 - alpha) * self.curr_map_odom_base_x
                self.y_estimate = alpha * self.y_estimate + (1 - alpha) * self.curr_map_odom_base_y
                if delta_yaw_time < 0.5:
                    #self.get_logger().info(f"lowpass filtering the yaw in map frame")
                    #apply low pass filter to yaw estimate as well
                    self.yaw_estimate = alpha * self.yaw_estimate + (1 - alpha) * self.curr_map_odom_base_yaw
        else:
            #time since last update is too high so we need to trust the aruco pose
            #however we should still do some "outlier" detection
            #because the aruco map pose should be be too far away from the current map->odom->base_link pose

            if self.triangulated_new_xy:
                #calculate the distance between the current pose estimate and the last pose estimate
                dist = math.hypot(self.x_estimate - self.curr_map_odom_base_x, self.y_estimate - self.curr_map_odom_base_y)
                if dist > 1.5:
                    #if the distance is too high, we should not trust the pose estimate
                    self.get_logger().warn(f"Pose estimate is too far away from the current pose estimate: {dist:.2f}m")
                    self.x_estimate = self.curr_map_odom_base_x
                    self.y_estimate = self.curr_map_odom_base_y
                    self.triangulated_new_xy = False
                    self.triangulated_new_pose = False
            if self.measured_new_yaw:
                #calculate the difference between the current yaw estimate and the last yaw estimate
                yaw_diff = self.yaw_estimate - self.curr_map_odom_base_yaw
                yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
                if abs(yaw_diff) > 0.5:  # 0.5 radians = 28.65 degrees
                    #if the difference is too high, we should not trust the yaw estimate
                    self.get_logger().warn(f"Yaw estimate is too far away from the current yaw estimate: {yaw_diff*180/3.141592:.2f} degrees")
                    self.yaw_estimate = self.curr_map_odom_base_yaw
                    self.measured_new_yaw = False

        #PUBLISH THE GLOBAL POSEEEEEE : map -> base_link. This is the position of the Rover in the ERC coordinate system
        #this will be fed to the EKF that will calculate the map->odom transform.
        # the transform tree will thus be complete: we will have map->odom and odom->base_link, which is what we need
        #for nav2 and to be able to give it the correct waypoints
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        cov = [0.]*36

        odom_msg.pose.pose.position.x = self.x_estimate
        odom_msg.pose.pose.position.y = self.y_estimate
        odom_msg.pose.pose.orientation = yaw_to_quat(self.yaw_estimate)

        if self.triangulated_new_pose:
            #cov[0] = cov[7] = 0.005    # var(x), var(y)
            cov = self.low_cov.copy()
            self.triangulated_new_pose = False
        else:
            #set very high covariance for x and y since we were not able to triangulate
            #cov[0] = cov[7] = 1e6
            cov = self.high_cov.copy()
        
        if not hasattr(self, 'last_orientation'):
            self.last_orientation = Quaternion(w=1.0)  # identity

        if self.measured_new_yaw:
            q = yaw_to_quat(self.yaw_estimate)
            self.last_orientation = q
            cov[35] = 0.002
            self.measured_new_yaw = False
        else:
            q = self.last_orientation
            cov[35] = 1e6
        odom_msg.pose.pose.orientation = q

                    
        odom_msg.pose.covariance = cov
        self.publisher_.publish(odom_msg)



        #now publish the corrected map->odom transform (since we already have odom->base_link done by the EKF)
        #we can use the tf2_ros library to do this

        # Publish the transform from map to odom

        # T_map_base = <from ArUco triangulation in map frame>
        # T_odom_base = <from ekf_odom topic: odom→base_link> which is stored in self.odom_pos_x, self.odom_pos_y, self.odom_yaw
        # T_map_odom = T_map_base * inv(T_odom_base)

        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'odom'

        # Calculate the transform from map to odom
        T_map_base  = self.pose_to_mat(self.x_estimate, self.y_estimate, self.yaw_estimate)
        T_odom_base = self.pose_to_mat(self.odom_pos_x, self.odom_pos_y, self.odom_yaw)

        # Calculate the transformation matrix from map to odom
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        transform_msg.transform.translation.x = T_map_odom[0, 3]
        transform_msg.transform.translation.y = T_map_odom[1, 3]
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation = yaw_to_quat(math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
        #broadcast the transform
        self.tf_broadcaster.sendTransform(transform_msg)
        self.get_logger().info(f"Broadcasted map->odom transform, tx :{transform_msg.transform.translation.x}, ty{transform_msg.transform.translation.y}")
        self.get_logger().info(f"--> Estimated Yaw (deg): {(self.yaw_estimate*180/3.141592):.3f}")


        # Store the last pose for the next iteration
        self.prev_map_odom_tf = transform_msg


    # Helper function to convert pose to transformation matrix
    # This function creates a 4x4 transformation matrix from x, y, and yaw
    # It is used to convert the pose estimates into a transformation matrix for publishing
    def pose_to_mat(self, x, y, yaw):
                c, s = math.cos(yaw), math.sin(yaw)
                M = np.eye(4) # 4x4 identity matrix
                #rotation matrix for 2D rotation
                M[0:2,0:2] = [[c, -s],
                              [s,  c]]
                #translation vector
                M[0,3], M[1,3] = x, y
                return M


    def cost_function(self, estimate, landmarks, measured_distances):
        x_r, y_r = estimate
        residuals = []
        for (x_i, y_i), d_i in zip(landmarks, measured_distances):
            predicted_distance = np.sqrt((x_r - x_i)**2 + (y_r - y_i)**2)
            residuals.append(predicted_distance - d_i)
        return residuals


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    executor = MultiThreadedExecutor(
        # number of worker threads; None==number of CPU cores
        num_threads=3
    )
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()