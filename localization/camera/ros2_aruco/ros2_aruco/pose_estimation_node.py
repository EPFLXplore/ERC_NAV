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
        self.time_of_last_pose = self.get_clock().now()
        self.time_of_last_yaw_meas = self.get_clock().now()

        self.MAP_SIZE = 300.0

        self.nbr_init_callbacks_for_avg = 20  #20 measurements on initialization to have a good estimate of the map->odom transform with outlier rejection
        # then after that we limit the rate of the listener with the following parameters:
        self.init_callback_counter = 0
        self.initialized_map_odom_tf = False
        self.last_callback_time = self.get_clock().now()
        self.callback_freq_limit = 0.2 #seconds = 5Hz
        self.avg_initialization_tfs = []

        self.max_translation_jump = 1.7 #meters
        self.max_yaw_jump = math.radians(35) #23 degrees

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
        self.tf_timer = self.create_timer(0.1, self.republish_map_odom_transform) #10hz

        self.publisher_ = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.subscription 

        # timer_period = 2.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initial guess for optimization
        self.initial_estimate = np.array([0.0, 0.0])

        # ArUco ID 51 → index 0 of the landmark_poses list
        # ArUco ID 52 → index 1
        # ArUco ID 53 → index 2
        # ...
        #the positions are relative to the map frame which is given to us by the ERC task description
        self.erc_start_pos = [0.0, 0.0]  #x, y

        self.landmark_poses = [
                (-2.39, 1.475),
                (-2.39, -1.055),
                (-1.735, 2.15),
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

        now = self.get_clock().now()
        if ((now - self.last_callback_time).nanoseconds/1e9 < self.callback_freq_limit  and self.initialized_map_odom_tf == True):
            return #limit the rate at which we run the callback since it is computationnally expensive and we dont need it very often.
        self.last_callback_time = now


        #get the current map->odom transform then add it to current odom->base_link from the EKF
        #this will give us the current map->base_link pose
        try:
            now = self.get_clock().now().to_msg()
            transform = self.tf_buffer.lookup_transform('map','odom', now, timeout=rclpy.duration.Duration(seconds=0.2))
            T_map_odom = self.pose_to_mat(
                transform.transform.translation.x,
                transform.transform.translation.y,
            R.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
                ]).as_euler('xyz')[2]
            )

            T_odom_base = self.pose_to_mat(
                self.odom_pos_x,
                self.odom_pos_y,
                self.odom_yaw
            )

            # latest map->odom + latest odom->base_link
            T_map_base = T_map_odom @ T_odom_base

            self.curr_map_odom_base_x   = T_map_base[0, 3]
            self.curr_map_odom_base_y   = T_map_base[1, 3]
            self.curr_map_odom_base_yaw = math.atan2(T_map_base[1,0], T_map_base[0,0])

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
            if idx < len(self.landmark_poses) and self.landmark_poses[idx][0] < self.MAP_SIZE and self.landmark_poses[idx][1] < self.MAP_SIZE:
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
            if abs(lm1[0]) > self.MAP_SIZE or abs(lm2[0]) > self.MAP_SIZE:  #spicy magic number =300meters just to check if they have actually been hardcoded in the code
                continue

            dx_map = lm2[0] - lm1[0]
            dy_map = lm2[1] - lm1[1]
            angle_map = math.atan2(dy_map, dx_map) #angle formed by the vector pointing from an aruco to another, in the map frame. this is a theoretical angle

            #poses of the same aruco tags as seen by the rover, so this is in the rover frame
            pose1 = msg.poses[i].position
            pose2 = msg.poses[j].position

            dx_rover = pose2.x - pose1.x
            dy_rover = pose2.y - pose1.y
            angle_rover = math.atan2(dy_rover, dx_rover)
            
            yaw_diff = angle_map - angle_rover
            yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi

            #the rover's yaw in the map frame is thus the yaw difference

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

                #on initialization this will select the interseciton which is closest to the ERC starting pose

                if self.initialized_map_odom_tf:
                    dist_I1 = math.hypot(I1[0] - self.curr_map_odom_base_x, I1[1] - self.curr_map_odom_base_y)
                    dist_I2 = math.hypot(I2[0] - self.curr_map_odom_base_x, I2[1] - self.curr_map_odom_base_y)
                else:
                    dist_I1 = math.hypot(I1[0] - self.erc_start_pos[0], I1[1] - self.erc_start_pos[1])
                    dist_I2 = math.hypot(I2[0] - self.erc_start_pos[0], I2[1] - self.erc_start_pos[1])

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
            else:
                self.get_logger().info(f"no circle intersection")

        # if >=3 landmarks we can triangulate the pose using least squares
        if len(valid_markers)>=3:

            distance_estimates = [math.hypot(p.position.x, p.position.y) for _, p in valid_markers]
            landmarks_ordered = [self.landmark_poses[idx] for idx, _ in valid_markers]

            if self.initialized_map_odom_tf:
                map_pos_x = self.curr_map_odom_base_x
                map_pos_y = self.curr_map_odom_base_y
                #bounds = window of size 2*2=4m centered on the current position
                base_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'trf', loss='soft_l1', f_scale=0.2, 
                                              bounds=([map_pos_x - 2, map_pos_y -2], [map_pos_x + 2, map_pos_y + 2]), 
                                              args=(landmarks_ordered, distance_estimates))
            else:
                #the initial position should be around self.erc_start_pos, so we create a square window centered on that position with a width of 2*2=4m
                base_estimate = least_squares(self.cost_function, self.initial_estimate, method= 'trf', loss='soft_l1', f_scale=0.2, 
                                              bounds=([self.erc_start_pos[0]-2, self.erc_start_pos[1]-2], [self.erc_start_pos[0]+2, self.erc_start_pos[1]+2]), 
                                              args=(landmarks_ordered, distance_estimates))
            
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

            if delta_time < 0.5: #if we are recieving pose estimations at >2hz
                #self.get_logger().info(f"lowpass filtering the map->odom TF")
                # IIR 1st Order Low Pass Filter
                #H(e^jw) = alpha / (1 + (alpha - 1)e^(-jw))
                #w --> infinity => H = alpha
                #w --> 0 => H = 1
                #response time will depend on the frequency of the updates
                alpha = 0.5 #low alpha = more smoothing, high alpha = less smoothing
                self.x_estimate = alpha * self.x_estimate + (1 - alpha) * self.curr_map_odom_base_x
                self.y_estimate = alpha * self.y_estimate + (1 - alpha) * self.curr_map_odom_base_y
                if delta_yaw_time < 0.5:
                    #self.get_logger().info(f"lowpass filtering the yaw in map frame")
                    #apply low pass filter to yaw estimate as well
                    self.yaw_estimate = alpha * self.yaw_estimate + (1 - alpha) * self.curr_map_odom_base_yaw

        ########################################### WE DONT REALLY NEED TO PUBLISH THIS, IT IS JUST HERE FOR CONVENIENCE ############################3
        # code to publish map -> base_link. This is the position of the Rover in the ERC coordinate system
        # this will be fed to the EKF that will calculate the map->odom transform.
        # the transform tree will thus be complete: we will have map->odom and odom->base_link, which is what we need
        # for nav2 and to be able to give it the correct waypoints
        # odom_msg = Odometry()
        # odom_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_msg.header.frame_id = 'map'
        # odom_msg.child_frame_id = 'base_link'
        # cov = [0.]*36

        # odom_msg.pose.pose.position.x = self.x_estimate
        # odom_msg.pose.pose.position.y = self.y_estimate
        # odom_msg.pose.pose.orientation = yaw_to_quat(self.yaw_estimate)

        # if self.triangulated_new_pose:
        #     #cov[0] = cov[7] = 0.005    # var(x), var(y)
        #     cov = self.low_cov.copy()
        #     self.triangulated_new_pose = False
        # else:
        #     #set very high covariance for x and y since we were not able to triangulate
        #     #cov[0] = cov[7] = 1e6
        #     cov = self.high_cov.copy()
        
        # if not hasattr(self, 'last_orientation'):
        #     self.last_orientation = Quaternion(w=1.0)  # identity

        # if self.measured_new_yaw:
        #     q = yaw_to_quat(self.yaw_estimate)
        #     self.last_orientation = q
        #     cov[35] = 0.002
        #     self.measured_new_yaw = False
        # else:
        #     q = self.last_orientation
        #     cov[35] = 1e6
        # odom_msg.pose.pose.orientation = q

                    
        # odom_msg.pose.covariance = cov

        # if self.initialized_map_odom_tf == True:
        #     self.publisher_.publish(odom_msg)
        #############################################################################33

        #now publish the corrected map->odom transform (since we already have odom->base_link done by the EKF)
        #we can use the tf2_ros library to do this
        #we dont directly publish map->base_link because it makes it much easier to reject bad noisy data once the map->odom tf has been initialized
        #because assuming perfect odometry this map->odom should be constant.

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
        
        #gate against outlier/garbage/very noisy map->odom tf values that are incoherent after initialization (aka when moving during the task)
        if self.initialized_map_odom_tf:
            old_t = np.array([self.prev_map_odom_tf.transform.translation.x,
                            self.prev_map_odom_tf.transform.translation.y])
            new_t = np.array([T_map_odom[0,3], T_map_odom[1,3]])
            delta_t = new_t - old_t
            dist_jump = np.linalg.norm(delta_t)

            old_yaw = math.atan2(self.prev_map_odom_tf.transform.rotation.y,
                                self.prev_map_odom_tf.transform.rotation.x)
            new_yaw = math.atan2(T_map_odom[1,0], T_map_odom[0,0])
            yaw_jump = abs(((new_yaw - old_yaw)+np.pi)%(2*np.pi)-np.pi)

            if dist_jump > self.max_translation_jump or yaw_jump > self.max_yaw_jump:
                self.get_logger().warn(f"Rejected map→odom jump {dist_jump:.2f} meters, {math.degrees(yaw_jump):.1f}° deg")
                return

        transform_msg.transform.translation.x = T_map_odom[0, 3]
        transform_msg.transform.translation.y = T_map_odom[1, 3]
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation = yaw_to_quat(math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
        #broadcast the transform
        if self.initialized_map_odom_tf == True:
            self.tf_broadcaster.sendTransform(transform_msg)
            self.get_logger().info(f"Broadcasted map->odom transform, tx :{transform_msg.transform.translation.x}, ty{transform_msg.transform.translation.y}")
            self.get_logger().info(f"--> Estimated Yaw (deg): {(self.yaw_estimate*180/3.141592):.3f}")
            self.get_logger().info(f"--> Estimate X in map: {(self.x_estimate):.3f}")
            self.get_logger().info(f"--> Estimated Y in map: {(self.y_estimate):.3f}")
            self.get_logger().info(f"-------------------------------------")

        # Store the last pose for the next iteration
        self.prev_map_odom_tf = transform_msg


        if(self.init_callback_counter < self.nbr_init_callbacks_for_avg):

            self.init_callback_counter += 1
            self.avg_initialization_tfs.append(transform_msg)

            if (self.init_callback_counter == self.nbr_init_callbacks_for_avg):

                self.initialized_map_odom_tf = True
                self.prev_map_odom_tf = self.calculate_robust_tf_avg(self.avg_initialization_tfs)
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
                self.get_logger().info(f"--> INITIALIZED FIRST MAP->ODOM TF, NOW RATE LIMITING THIS NODE @ {(1/(self.callback_freq_limit)):.3f} Hz")


    def calculate_robust_tf_avg(self, tf_list, max_trans_outlier=0.5, max_rot_outlier=np.deg2rad(10)):
        #stack the [x, y] translations for each tf in the list
        t = np.array([[tf.transform.translation.x, tf.transform.translation.y] for tf in tf_list])

        #compute median and MAD (median absolute deviation = median of |x_i - median|)
        med_t = np.median(t, axis=0)
        mad_t = np.median(np.linalg.norm(t - med_t, axis=1))
        # keep only those within k * MAD (e.g. k=3)
        inliers_t = np.linalg.norm(t - med_t, axis=1) < max(3 * mad_t, max_trans_outlier)

        #stack the quaternions for each tf in the list
        qs = np.array([[tf.transform.rotation.w,
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z]
                    for tf in tf_list])
        
        # convert to rotation vectors for geodesic distance
        rots = R.from_quat(qs[:, [1,2,3,0]])  # (x,y,z,w)        
        angvecs = rots.as_rotvec()
        # compute their “median” rotvec
        med_rotvec = np.median(angvecs, axis=0)
        # measure angular deviations
        ang_dev = np.linalg.norm(angvecs - med_rotvec, axis=1)
        inliers_r = ang_dev < max(3 * np.median(ang_dev), max_rot_outlier)

        #combine inlier masks
        inliers = inliers_t & inliers_r
        if inliers.sum() < 3:
            # fallback to simple average if too many outliers
            inliers[:] = True

        #final translation
        final_t = t[inliers].mean(axis=0)

        #final rotation
        qs_in = qs[inliers]
        # pick the first inlier as reference
        q_ref = qs_in[0]
        for i in range(len(qs_in)):
            if np.dot(qs_in[i], q_ref) < 0:
                qs_in[i] *= -1
        q_mean = qs_in.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)


        
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = float(final_t[0])
        tf.transform.translation.y = float(final_t[1])
        tf.transform.translation.z = 0.0
   
        tf.transform.rotation.x = q_mean[1]
        tf.transform.rotation.y = q_mean[2]
        tf.transform.rotation.z = q_mean[3]
        tf.transform.rotation.w = q_mean[0]

        return tf



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
        # number of worker threads; None==number of CPU cores i think
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