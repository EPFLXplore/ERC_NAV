import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares, fsolve


import cv2


from ros2_aruco.triangulation import triangulate #custom geometric triangulation code
from ros2_aruco.triangulation_opti import triangulate_opti #custom optimization triangulation code

#for generating combinations of arucos (pairs for yaw estimation and triplets for triangulation)
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


def wrap(a):
    return (a + np.pi) % (2*math.pi) - math.pi

def bearing_range_residuals(state, landmarks, bearings, ranges, w_b, w_rs):
    x_m, y_m, θ = state
    res = []
    for (x_i, y_i), β_i, t_i, w_r in zip(landmarks, bearings, ranges, w_rs):
        # predicted bearing residual
        psi = math.atan2(y_i - y_m, x_i - x_m) - θ
        res.append(w_b * wrap(psi - β_i))
        # predicted range residual
        pred_r = math.hypot(x_i - x_m, y_i - y_m)
        res.append(w_r * (pred_r - t_i))
    return res


def solve_position_from_two_bearings(A, B, phiA, phiB, psi_map, cond_thresh=1e-6):
    θA = wrap(psi_map + phiA)
    θB = wrap(psi_map + phiB)
    vA = np.array([math.cos(θA), math.sin(θA)])
    vB = np.array([math.cos(θB), math.sin(θB)])
    M = np.column_stack([vA, -vB])
    if abs(np.linalg.det(M)) < cond_thresh:
        return None
    tA, _ = np.linalg.solve(M, A - B)
    return A - tA * vA




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
        self.time_of_last_good_triangulation = self.get_clock().now()
        self.measured_good_triang = False
        self.min_least_squares_dt_from_triang = 15.0#seconds

        self.MAP_SIZE = 300.0

        self.nbr_init_callbacks_for_avg = 35  #35 measurements on initialization to have a good estimate of the map->odom transform with outlier rejection
        # then after that we limit the rate of the listener with the following parameters:
        self.init_callback_counter = 0
        self.initialized_map_odom_tf = False
        self.last_callback_time = self.get_clock().now()
        self.callback_period_limit = 1/15.0 #seconds (cannot be faster than the refresh rate of the realsense cameras = 15fps)
        self.avg_initialization_tfs = []
        self.yaw_init_list = []
        self.min_yaw_dt = 30.0#seconds


        self.max_translation_jump = 0.5 #meters
        self.max_yaw_jump = math.radians(40) #degrees
        self.max_aruco_dist_for_tvec_use = 4.0 #meters
        self.start_pose_tolerance = 0.2 #meters

        self.w_bearing = 1.0
        self.w_range   = 0.05
        self.alpha = 0.1 #weight on range per landmark = weight_base/(1+alpha*range)

        self.max_bearing_diff = 179.0
        self.min_bearing_diff = 2.0

        self.max_nbr_triplets = 5
        self.max_nbr_pairs = 10

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
        self.lpf_coeff = 0.7
        self.delta_conv = 2.0

        self.high_cov = [1e6 if i in (0,7,35) else 0. for i in range(36)]
        self.low_cov  = [0.005 if i in (0,7) else 0.002 if i==35 else 0. for i in range(36)]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        #create a timer to publish the map->odom transform at a fixed rate to avoid timeout errors
        self.prev_map_odom_tf = None
        self.tf_timer = self.create_timer(0.2, self.republish_map_odom_transform) #5hz

        self.subscription

        # ArUco ID 51 → index 0 of the landmark_poses list
        # ArUco ID 52 → index 1
        # ArUco ID 53 → index 2
        # ...
        #the positions are relative to the map frame origin which is given to us by the ERC task description
        self.erc_start_pos = [0.655, 2.515]  #x, y

        self.landmark_poses = [
                (-0.585, 0.0),
                (2.62, 0.505),
                (1.46, 8.45),
                (-2.28, 15.81),
                (3.74, 19.07),
                (7.04, 14.67),
                (11.46, 19.78),
                (15.51, 19.33),
                (16.3, 14.87),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
                (999999, 999999),
            ]


                # (-0.75, 0.46),
                # (2.59, 1.11),
                # (1.36, 6.79),
                # (-1.4, 17.07),
                # (7.39, 19.4),
                # (10.39, 14.99),
                # (-5.1, 6.79),
        
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
        
        
    def are_points_counterclockwise(self, vertices):
        vertices = np.asarray(vertices)

        if len(vertices) < 3:
            return None
        #find the vertex with the smallest y
        min_y = min(v[1] for v in vertices)
        candidates = [i for i, (x,y) in enumerate(vertices) if y == min_y]

        min_y_idx = max(candidates, key=lambda i: vertices[i][0]) #tie breaker for the largest x
        A = np.array([vertices[min_y_idx]])
        B = np.array([vertices[min_y_idx - 1]])
        C = np.array([vertices[(min_y_idx + 1)%len(vertices)]])

        AB = (B - A).flatten()
        AC = (C - A).flatten()
        
        #cross product of ABxAC        
        cross_prod = AB[0]*AC[1] - AB[1]*AC[0]

        return cross_prod < 0


    def get_circle_intersect(self, id1, pose1, id2, pose2, return_both=False):
        #returns the position of the two circle intersection that is closest to the current position (self.erc_start_pos if the system is not initialized yet, and self.curr_map_odom_base_x/y if it is initialized)
        #id1, id2 are the aruco ids
        #pose1 and pose2 are the corresponding rover->aruco poses
        # 1) landmark centers in map frame (given by ERC)
        X1, Y1 = self.landmark_poses[id1]
        X2, Y2 = self.landmark_poses[id2]

        # 2) measured radii in rover frame
        X1_base, Y1_base = pose1.position.x, pose1.position.y
        X2_base, Y2_base = pose2.position.x, pose2.position.y
        R1 = math.hypot(X1_base, Y1_base)
        R2 = math.hypot(X2_base, Y2_base)

        # 3) theoretical distance between landmarks
        Dx = X2 - X1
        Dy = Y2 - Y1
        D = math.hypot(Dx, Dy)
        if D == 0.0:
            return None

        # 4) measured distance between detections
        D_meas = math.hypot(X2_base - X1_base, Y2_base - Y1_base)
        if abs(D_meas - D) / D > 0.20:  # 20% threshold
            return None

        # 5) reject non‐intersecting circles
        if D > (R1 + R2) or D < abs(R2 - R1):
            return None

        # 6) chord parameters
        chorddistance   = (R1**2 - R2**2 + D**2) / (2 * D)
        halfchordlength = math.sqrt(R1**2 - chorddistance**2)
        chordmid_x      = X1 + (chorddistance * Dx) / D
        chordmid_y      = Y1 + (chorddistance * Dy) / D

        # 7) two intersections
        I1 = (
            chordmid_x + (halfchordlength * Dy) / D,
            chordmid_y - (halfchordlength * Dx) / D
        )
        I2 = (
            chordmid_x - (halfchordlength * Dy) / D,
            chordmid_y + (halfchordlength * Dx) / D
        )

        # 8) same swap logic by comparing theta1, theta2
        theta1 = math.degrees(math.atan2(I1[1] - Y1, I1[0] - X1))
        theta2 = math.degrees(math.atan2(I2[1] - Y1, I2[0] - X1))
        if theta2 > theta1:
            I1, I2 = I2, I1

        self.get_logger().info(f"I1: {I1}, I2: {I2}")

        # 9) pick the one closest to reference pose
        if self.initialized_map_odom_tf:
            rx, ry = self.curr_map_odom_base_x, self.curr_map_odom_base_y
        else:
            rx, ry = self.erc_start_pos

        d1 = math.hypot(I1[0] - rx, I1[1] - ry)
        d2 = math.hypot(I2[0] - rx, I2[1] - ry)

        chosen_intersect = I1 if d1 <= d2 else I2
        chosen_dist_to_ref = min(d1, d2)

        if not return_both:
            if chosen_dist_to_ref > self.max_translation_jump:
                return None
            else:
                return chosen_intersect
        else:
            if chosen_dist_to_ref > self.max_translation_jump:
                return None, [I1, I2]
            else:
                return chosen_intersect, [I1, I2]

    def bearing_only_geometric_intersection(self, A, B, phiA, phiB):
        A = np.array(A)
        B = np.array(B)
        d = np.linalg.norm(B - A)

        if d == 0:
            return None, None

        u = (B - A) / d
        n = np.array([-u[1], u[0]])  # perpendicular to AB

        phi = wrap(phiB - phiA)
        if np.isclose(np.tan(phi), 0):
            return None, None

        h = d / (2 * np.tan(phi))
        M = (A + B) / 2

        P1 = M + h * n
        P2 = M - h * n

        return P1, P2


    def listener_callback(self, msg):

        now = self.get_clock().now()
        if ((now - self.last_callback_time).nanoseconds/1e9 < self.callback_period_limit  and self.initialized_map_odom_tf == True):
            return #limit the rate at which we run the callback since it is computationnally expensive and we dont need it very often.
        self.last_callback_time = now


        #get the current map->odom transform then add it to current odom->base_link from the EKF
        #this will give us the current map->base_link pose
        try:
            now = self.get_clock().now().to_msg()
            transform = self.tf_buffer.lookup_transform('odom','map', Time())
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

            if abs(self.odom_pos_x)<1e-4 and abs(self.odom_pos_y)<1e-4:
                #self.get_logger().info(f"kalman did not move !")
                self.curr_map_odom_base_x = self.x_estimate
                self.curr_map_odom_base_y = self.y_estimate
                self.curr_map_odom_base_yaw = self.yaw_estimate
                T_odom_base = np.eye(4)
            else:
                #self.get_logger().info(f"kalman moveddd !")
                # latest map->odom + latest odom->base_link
                T_map_base = T_map_odom @ T_odom_base
                self.curr_map_odom_base_x   = T_map_base[0, 3]
                self.curr_map_odom_base_y   = T_map_base[1, 3]
                self.curr_map_odom_base_yaw = math.atan2(T_map_base[1,0], T_map_base[0,0])

                if not self.triangulated_new_xy:
                    self.x_estimate   = self.curr_map_odom_base_x
                    self.y_estimate   = self.curr_map_odom_base_y
                    self.yaw_estimate = self.curr_map_odom_base_yaw

        except TransformException as e:
            if self.prev_map_odom_tf is None:
                self.curr_map_odom_base_x = self.odom_pos_x
                self.curr_map_odom_base_y = self.odom_pos_y
                self.curr_map_odom_base_yaw = self.odom_yaw
                #odom_base_pose_map = (self.curr_map_odom_base_x, self.curr_map_odom_base_y)
                #self.get_logger().warn("Initializing TF: assuming MAP = ODOM.")
            #self.get_logger().info(f"FAILEDDD TF LOOKUP")

        #sort by increasing id
        marker_ids = list(msg.marker_ids)
        self.get_logger().info(f"msg marker ids: {list(msg.marker_ids)}")
        
        
        # Estimate the yaw using the detected arucos

        #for every pair of arucos calculate atan2(ar2_map_y - ar1_map_y, ar2_map_x - ar1_map_x)
        #then using the aruco message use the received positions of the same arucos in the rover frame:
        #atan2(ar2_rover_y - ar1_rover_y, ar2_rover_x - ar1_rover_x)


        #self.get_logger().info(f"list of pairs of ids: {aruco_idx_pairs}")
        yaw_offsets = [] #store the offsets of the rover odometry yaw compared to map yaw


        #validate markers
        valid_markers = []
        # for idx, pose in zip(msg.marker_ids, msg.poses):
        #     if idx < len(self.landmark_poses) and self.landmark_poses[idx][0] < self.MAP_SIZE and self.landmark_poses[idx][1] < self.MAP_SIZE:
        #         valid_markers.append((idx, pose))
        for k, (idx, pose) in enumerate(zip(msg.marker_ids, msg.poses)):
            x_map, y_map = self.landmark_poses[idx] if idx < len(self.landmark_poses) else (None, None)
            if x_map is not None and abs(x_map) < self.MAP_SIZE and abs(y_map) < self.MAP_SIZE:
                valid_markers.append((idx, pose, k))

        ids = [idx for idx,_,_ in valid_markers]
        #self.get_logger().info(f"Valid marker IDs: {ids}")


        #limit the number of pairs in case a lot of arucos are detected and only keep the pairs with the arucos that are the closest to the rover
        pair_scores = []
        for i, j in combinations(range(len(valid_markers)), 2):
            id1, _, _ = valid_markers[i]
            id2, _, _ = valid_markers[j]

            x1, y1 = self.landmark_poses[id1]
            x2, y2 = self.landmark_poses[id2]

            ref_x = self.curr_map_odom_base_x if self.initialized_map_odom_tf else self.erc_start_pos[0]
            ref_y = self.curr_map_odom_base_y if self.initialized_map_odom_tf else self.erc_start_pos[1]

            d1 = math.hypot(x1 - ref_x, y1 - ref_y)
            d2 = math.hypot(x2 - ref_x, y2 - ref_y)

            avg_d = 0.5*(d1 + d2)
            if d1 < self.max_aruco_dist_for_tvec_use and d2 < self.max_aruco_dist_for_tvec_use:
                pair_scores.append(((i, j), avg_d))
        pair_scores.sort(key=lambda item: item[1])
        aruco_idx_pairs = [pair for (pair, _) in pair_scores[:self.max_nbr_pairs]]

        msg_poses_list = list(msg.poses)
        aruco_id_to_pose_dict = {}
        for i in range(len(msg_poses_list)):    
            aruco_id = marker_ids[i]
            x = msg_poses_list[i].position.x
            y = msg_poses_list[i].position.y
            z = msg_poses_list[i].position.z

            aruco_id_to_pose_dict[aruco_id] = [x, y]  
            #we map the aruco id to its pose transofrm from base_link to itself
        
        #self.get_logger().info(f"dictionnary: {aruco_id_to_pose_dict}")


        if len(valid_markers) < 2:
            #self.get_logger().warn("Not enough valid markers detected for yaw estimation.")
            return

        # for i, j in aruco_idx_pairs:

        #     id1 = marker_ids[i]
        #     id2 = marker_ids[j]

        #     if id1 == id2 :
        #         continue

        #     # Skip if either landmark is invalid
        #     if id1 >= len(self.landmark_poses) or id2 >= len(self.landmark_poses):
        #         continue
            
        #     lm1 = self.landmark_poses[id1]
        #     lm2 = self.landmark_poses[id2]
        #     if abs(lm1[0]) > self.MAP_SIZE or abs(lm2[0]) > self.MAP_SIZE:  #just to check if they have actually been hardcoded in the code
        #         continue

        #     dx_map = lm2[0] - lm1[0]
        #     dy_map = lm2[1] - lm1[1]
        #     angle_map = math.atan2(dy_map, dx_map) #angle formed by the vector pointing from an aruco to another, in the map frame. this is a theoretical angle

        #     #poses of the same aruco tags as seen by the rover, so this is in the rover frame
        #     # pose1 = msg.poses[id1].position
        #     # pose2 = msg.poses[id2].position
        #     pose1 = aruco_id_to_pose_dict[id1]
        #     pose2 = aruco_id_to_pose_dict[id2]

        #     dx_rover = pose2[0] - pose1[0]
        #     dy_rover = pose2[1] - pose1[1]
        #     angle_rover = math.atan2(dy_rover, dx_rover)
            
        #     yaw_diff = angle_map - angle_rover
        #     yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi

        #     #the rover's yaw in the map frame is thus the yaw difference
        #     yaw_offsets.append(yaw_diff)
        #     #self.get_logger().info(f"--> Yaw (deg): {(yaw_diff*180/3.141592):.3f} for ids [{id1}-{id2}]")

        
        # if len(yaw_offsets) > 0:
        #     avg_yaw = sum(yaw_offsets)/len(yaw_offsets)
        #     #self.get_logger().info(f"--> Yaw (deg): {(avg_yaw*180/3.141592):.3f} ")
        #     self.yaw_estimate = avg_yaw
        #     self.time_of_last_yaw_meas = self.get_clock().now()
        


        n = len(valid_markers)

        # helper: do all triplets triangulation → return mean P or None
        def try_all_triplets():
            sols = []
            for (iA,_,kA),(iB,_,kB),(iC,_,kC) in combinations(valid_markers, 3):
                lmA = self.landmark_poses[iA]
                lmB = self.landmark_poses[iB]
                lmC = self.landmark_poses[iC]
                # order CW:
                if self.are_points_counterclockwise([lmA,lmB,lmC]):
                    lms = [lmC,lmB,lmA]
                    phs = [msg.ar_angles_list[kC],
                        msg.ar_angles_list[kB],
                        msg.ar_angles_list[kA]]
                else:
                    lms = [lmA,lmB,lmC]
                    phs = [msg.ar_angles_list[kA],
                        msg.ar_angles_list[kB],
                        msg.ar_angles_list[kC]]
                
                bearing_A = phs[0]
                bearing_B = phs[1]
                bearing_C = phs[2]

                phi_1 = min(abs(bearing_A - bearing_B), abs(360.0 - abs(bearing_A) - abs(bearing_B)))
                phi_2 = min(abs(bearing_B - bearing_C), abs(360.0 - abs(bearing_B) - abs(bearing_C)))
                phi_3 = min(abs(bearing_C - bearing_A), abs(360.0 - abs(bearing_C) - abs(bearing_A)))

                phs = [phi_1, phi_2, phi_3]

                ref = ([self.curr_map_odom_base_x, self.curr_map_odom_base_y]
                    if self.initialized_map_odom_tf else self.erc_start_pos)
                P = triangulate_opti(lms, phs, ref)
                if P is not None:
                    sols.append(P)
            return np.mean(sols, axis=0) if sols else None
                
        def try_filtered_ls(yaw):
            # build lists
            lms = []
            phs = []
            for (_,_,k) in valid_markers:
                lms.append(self.landmark_poses[ msg.marker_ids[k] ])
                phs.append(math.radians(msg.ar_angles_list[k]))
            # filter pairs with ≥20° baseline
            good = [False]*len(phs)
            for i in range(len(phs)):
                for j in range(i+1, len(phs)):
                    δ = abs(wrap((yaw+phs[i]) - (yaw+phs[j]) - math.pi))
                    if δ >= math.radians(20.0):
                        good[i] = good[j] = True
            lms_f = [L for L,ok in zip(lms,good) if ok]
            phs_f = [φ for φ,ok in zip(phs,good) if ok]
            if len(phs_f) < 2:
                return None

            def resid(xy):
                x,y = xy
                r = []
                for (xi,yi),φ in zip(lms_f, phs_f):
                    r.append(wrap(math.atan2(yi-y, xi-x) - yaw - φ))
                return r

            x0 = ( np.array([self.curr_map_odom_base_x,
                            self.curr_map_odom_base_y])
                if self.initialized_map_odom_tf else self.erc_start_pos )
            sol = least_squares(resid, x0, method='trf', loss='huber')
            return sol.x if sol.success else None
        


        ############### POSE ESTIMATION LOGIC ###################
        is_measurement_valid = False

        if not self.initialized_map_odom_tf:
            if n == 2:
                # only solve yaw from two bearings at ERC start
                # we assume the start position is correct enough
                
                (iA,_,kA),(iB,_,kB) = valid_markers
                A = self.landmark_poses[iA]
                B = self.landmark_poses[iB]
                φA = math.radians(msg.ar_angles_list[kA])
                φB = math.radians(msg.ar_angles_list[kB])
                x0,y0 = self.erc_start_pos
                yawA = wrap(math.atan2(A[1]-y0, A[0]-x0) - φA)
                yawB = wrap(math.atan2(B[1]-y0, B[0]-x0) - φB)
                self.yaw_estimate = wrap(0.5*(yawA + yawB))
                self.measured_new_yaw = True
                self.time_of_last_yaw_meas = self.get_clock().now()
                self.get_logger().info(f"[INIT] yaw = {math.degrees(self.yaw_estimate):.2f}°")
                #bogus tf
                T_map_base = self.pose_to_mat(self.erc_start_pos[0], self.erc_start_pos[1], self.yaw_estimate)
                T_odom_base = np.eye(4)  # assume odom==base_link for INIT
                T_map_odom  = T_map_base @ np.linalg.inv(T_odom_base)
                transform_msg = TransformStamped()
                transform_msg.header.stamp = self.get_clock().now().to_msg()
                transform_msg.header.frame_id = 'map'
                transform_msg.child_frame_id = 'odom'
                transform_msg.transform.translation.x = T_map_odom[0,3]
                transform_msg.transform.translation.y = T_map_odom[1,3]
                transform_msg.transform.rotation    = yaw_to_quat(
                    math.atan2(T_map_odom[1,0], T_map_odom[0,0])
                )
                is_measurement_valid = True

            elif n >= 3:
                # first try all triplets
                Ptri = try_all_triplets()
                if Ptri is not None:
                    # update position
                    dx = Ptri[0] - self.erc_start_pos[0]
                    dy = Ptri[1] - self.erc_start_pos[1]
                    if math.sqrt(dx*dx + dy*dy) < 0.4:
                        self.x_estimate, self.y_estimate = Ptri
                        self.get_logger().info(f"[INIT] triplet P = {Ptri}")
                        self.triangulated_new_xy = True
                        self.time_of_last_pose   = self.get_clock().now()
                        self.time_of_last_good_triangulation = self.get_clock().now()
                        self.measured_good_triang = True

                        # deduce map yaw from new P
                        yaw_list = []
                        for (idx, _, k) in valid_markers:
                            lm = self.landmark_poses[idx]
                            measured_phi = math.radians(msg.ar_angles_list[k])
                            bearing_map  = math.atan2(lm[1]-self.y_estimate, lm[0]-self.x_estimate)
                            yaw_list.append(wrap(bearing_map - measured_phi))
                        # circular mean
                        self.yaw_estimate = math.atan2(
                            sum(math.sin(y) for y in yaw_list),
                            sum(math.cos(y) for y in yaw_list)
                        )
                        self.measured_new_yaw = True
                        self.get_logger().info(f"[INIT] triang yaw = {math.degrees(self.yaw_estimate):.2f}°")
                        T_map_base = self.pose_to_mat(self.x_estimate, self.y_estimate, self.yaw_estimate)
                        T_odom_base = np.eye(4)  # assume odom==base_link for INIT
                        T_map_odom  = T_map_base @ np.linalg.inv(T_odom_base)
                        transform_msg = TransformStamped()
                        transform_msg.header.stamp = self.get_clock().now().to_msg()
                        transform_msg.header.frame_id = 'map'
                        transform_msg.child_frame_id = 'odom'
                        transform_msg.transform.translation.x = T_map_odom[0,3]
                        transform_msg.transform.translation.y = T_map_odom[1,3]
                        transform_msg.transform.rotation    = yaw_to_quat(
                            math.atan2(T_map_odom[1,0], T_map_odom[0,0])
                        )
                        is_measurement_valid = True

                else:
                    # fallback to filtered LS
                    Pls = try_filtered_ls(self.yaw_estimate)
                    if Pls is not None:
                        dx = Pls[0] - self.erc_start_pos[0]
                        dy = Pls[1] - self.erc_start_pos[1]
                        if math.sqrt(dx*dx + dy*dy) < 0.4:
                            self.x_estimate, self.y_estimate = Pls
                            self.get_logger().info(f"[INIT] LS-only P = {Pls}")
                            self.triangulated_new_xy = True
                            self.time_of_last_pose   = self.get_clock().now()

                            # deduce map yaw from LS result
                            yaw_list = []
                            for (idx, _, k) in valid_markers:
                                lm = self.landmark_poses[idx]
                                measured_phi = math.radians(msg.ar_angles_list[k])
                                bearing_map  = math.atan2(lm[1]-self.y_estimate, lm[0]-self.x_estimate)
                                yaw_list.append(wrap(bearing_map - measured_phi))
                            self.yaw_estimate = math.atan2(
                                sum(math.sin(y) for y in yaw_list),
                                sum(math.cos(y) for y in yaw_list)
                            )
                            self.measured_new_yaw = True
                            self.get_logger().info(f"[INIT] yaw = {math.degrees(self.yaw_estimate):.2f}°")
                            T_map_base = self.pose_to_mat(self.x_estimate, self.y_estimate, self.yaw_estimate)
                            T_odom_base = np.eye(4)
                            T_map_odom  = T_map_base @ np.linalg.inv(T_odom_base)
                            transform_msg = TransformStamped()
                            transform_msg.header.stamp = self.get_clock().now().to_msg()
                            transform_msg.header.frame_id = 'map'
                            transform_msg.child_frame_id = 'odom'
                            transform_msg.transform.translation.x = T_map_odom[0,3]
                            transform_msg.transform.translation.y = T_map_odom[1,3]
                            transform_msg.transform.rotation    = yaw_to_quat(
                                math.atan2(T_map_odom[1,0], T_map_odom[0,0])
                            )
                            is_measurement_valid = True


        # else:
        #     # -- normal operation after initialization --
        #     if n == 2:
        #         # solve (x,y) from two bearings + trusted yaw
        #         (iA,_,kA),(iB,_,kB) = valid_markers
        #         A = np.array(self.landmark_poses[iA])
        #         B = np.array(self.landmark_poses[iB])
        #         φA = math.radians(msg.ar_angles_list[kA])
        #         φB = math.radians(msg.ar_angles_list[kB])
        #         P2 = solve_position_from_two_bearings(A, B, φA, φB, self.yaw_estimate)
        #         if P2 is not None:
        #             dx = P2[0] - self.curr_map_odom_base_x
        #             dy = P2[1] - self.curr_map_odom_base_y
        #             if math.sqrt(dx*dx + dy*dy) < 2.0:
        #                 prev = np.array([self.curr_map_odom_base_x, self.curr_map_odom_base_y])
        #                 new  = P2
        #                 self.x_estimate, self.y_estimate = ((1-self.lpf_coeff)*prev + self.lpf_coeff*new)
        #                 self.triangulated_new_xy = True
        #                 self.time_of_last_pose   = self.get_clock().now()
        #                 self.get_logger().info(f"2 marker P = {self.x_estimate, self.y_estimate}")

        #     elif n >= 3:
        #         # try both methods
        #         Ptri = try_all_triplets()
        #         Pls  = try_filtered_ls(self.yaw_estimate)

        #         # whichever gives a solution (or average them), then update yaw
        #         # if Ptri is not None and Pls is not None:
        #         #     Pmix = 0.5*(Ptri + Pls)
        #         #     self.x_estimate, self.y_estimate = Pmix
        #         #     self.get_logger().info(f"mix(Ptri,Pls) = {Pmix}")
        #         dt_since_last_triang = (self.get_clock().now() - self.time_of_last_good_triangulation).nanoseconds * 1e-9

        #         if Ptri is not None:
        #             self.get_logger().info(f"TRIANGUUUU")
        #             dx = Ptri[0] - self.curr_map_odom_base_x
        #             dy = Ptri[1] - self.curr_map_odom_base_y
        #             self.get_logger().info(f"curr map odom: X= {self.curr_map_odom_base_x}, Y= {self.curr_map_odom_base_y}")
        #             if math.sqrt(dx*dx + dy*dy) < 2.0:
        #                 self.x_estimate, self.y_estimate = Ptri
        #                 self.get_logger().info(f"tri P = {Ptri}")
        #                 self.triangulated_new_xy = True
        #                 self.time_of_last_good_triangulation = self.get_clock().now()
        #                 self.measured_good_triang = True

        #         elif Pls is not None and dt_since_last_triang >= self.min_least_squares_dt_from_triang:
        #             self.get_logger().info(f"LEAST SQUAREEES")
        #             dx = Pls[0] - self.curr_map_odom_base_x
        #             dy = Pls[1] - self.curr_map_odom_base_y
        #             if math.sqrt(dx*dx + dy*dy) < 1.0:
        #                 self.x_estimate, self.y_estimate = Pls
        #                 self.triangulated_new_xy = True
        #                 self.get_logger().info(f"LS P = {Pls}")

        #         if Ptri is not None or Pls is not None and self.triangulated_new_xy:
        #             self.triangulated_new_xy = True
        #             self.time_of_last_pose   = self.get_clock().now()

        #             # deduce yaw from whichever position we used
        #             yaw_list = []
        #             for (idx, _, k) in valid_markers:
        #                 lm = self.landmark_poses[idx]
        #                 measured_phi = math.radians(msg.ar_angles_list[k])
        #                 bearing_map  = math.atan2(lm[1]-self.y_estimate, lm[0]-self.x_estimate)
        #                 yaw_list.append(wrap(bearing_map - measured_phi))
                    
        #             dt = (self.get_clock().now() - self.time_of_last_yaw_meas).nanoseconds * 1e-9
        #             if dt >= self.min_yaw_dt:
        #                 self.yaw_estimate = math.atan2(
        #                     sum(math.sin(y) for y in yaw_list),
        #                     sum(math.cos(y) for y in yaw_list)
        #                 )
        #                 self.measured_new_yaw = True
        #                 self.get_logger().info(f" n>=3 deduced yaw = {math.degrees(self.yaw_estimate):.2f}°")



        ####################################
        # #apply a simple IIR low pass
        # self.x_estimate = (1-self.lpf_coeff)*self.curr_map_odom_base_x + self.lpf_coeff*self.x_estimate
        # self.y_estimate = (1-self.lpf_coeff)*self.curr_map_odom_base_y + self.lpf_coeff*self.y_estimate


        #now publish the corrected map->odom transform (since we already have odom->base_link done by the EKF)
        #we can use the tf2_ros library to do this
        #we dont directly publish map->base_link because it makes it much easier to reject bad noisy data once the map->odom tf has been initialized
        #because assuming perfect odometry this map->odom should be constant.

        # Publish the transform from map to odom

        # T_map_base = <from ArUco triangulation in map frame>
        # T_odom_base = <from ekf topic: odom→base_link> which is stored in self.odom_pos_x, self.odom_pos_y, self.odom_yaw
        # T_map_odom = T_map_base * inv(T_odom_base)

        # Calculate the transform from map to odom when a new measurement has arrived
        if self.triangulated_new_xy and self.initialized_map_odom_tf:

            dx = self.x_estimate - self.curr_map_odom_base_x
            dy = self.y_estimate - self.curr_map_odom_base_y
            jump = math.hypot(dx, dy)
            if jump > self.max_translation_jump:
                return
            else:
                is_measurement_valid = True

                T_map_base  = self.pose_to_mat(self.x_estimate, self.y_estimate, self.yaw_estimate)
                T_odom_base = self.pose_to_mat(self.odom_pos_x, self.odom_pos_y, self.odom_yaw)

                # Calculate the transformation matrix from map to odom
                T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

                transform_msg = TransformStamped()
                transform_msg.header.stamp = self.get_clock().now().to_msg()
                transform_msg.header.frame_id = 'map'
                transform_msg.child_frame_id = 'odom'
                transform_msg.transform.translation.x = T_map_odom[0, 3]
                transform_msg.transform.translation.y = T_map_odom[1, 3]
                transform_msg.transform.translation.z = 0.0
                transform_msg.transform.rotation = yaw_to_quat(math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))

                # Store the last pose for the next iteration
                self.prev_map_odom_tf = transform_msg

            #gate against outlier/garbage/very noisy map->odom tf values that are incoherent after initialization (aka when moving during the task)
            # if self.initialized_map_odom_tf:
            #     old_t = np.array([self.prev_map_odom_tf.transform.translation.x,
            #                       self.prev_map_odom_tf.transform.translation.y])
            #     new_t = np.array([T_map_odom[0,3], T_map_odom[1,3]])
            #     delta_t = new_t - old_t
            #     dist_jump = np.linalg.norm(delta_t)

            #     q = self.prev_map_odom_tf.transform.rotation
            #     old_yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]


            #     new_yaw = math.atan2(T_map_odom[1,0], T_map_odom[0,0])
            #     yaw_jump = abs(((new_yaw - old_yaw) + np.pi) % (2*np.pi) - np.pi)

            #     if dist_jump > self.max_translation_jump or yaw_jump > self.max_yaw_jump:
            #         self.get_logger().warn(f"Rejected map→odom jump {dist_jump:.2f} meters, {math.degrees(yaw_jump):.2f}° deg")
            #         return
            


        #broadcast the transform
        if self.initialized_map_odom_tf:
            self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
            # self.get_logger().info(f"Broadcasted map->odom transform, tx :{transform_msg.transform.translation.x}, ty{transform_msg.transform.translation.y}")
            if self.triangulated_new_xy:
                self.get_logger().info(f"--> Estimated Yaw (deg): {(self.yaw_estimate*180/3.141592):.3f}")
                self.get_logger().info(f"--> Estimate X in map: {(self.x_estimate):.3f}")
                self.get_logger().info(f"--> Estimated Y in map: {(self.y_estimate):.3f}")
            #self.get_logger().info(f"-------------------------------------")



        if(self.init_callback_counter < self.nbr_init_callbacks_for_avg and is_measurement_valid):

            self.init_callback_counter += 1
            self.avg_initialization_tfs.append(transform_msg)
            self.yaw_init_list.append(self.yaw_estimate)
            self.get_logger().info(f"--> Estimated yaw in map DURING INIT: {(self.yaw_estimate*180/3.141592):.3f}")

            if (self.init_callback_counter == self.nbr_init_callbacks_for_avg):

                self.initialized_map_odom_tf = True
                self.prev_map_odom_tf = self.calculate_robust_tf_avg(self.avg_initialization_tfs, self.yaw_init_list)
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)

                self.get_logger().info(f"--> INITIALIZED FIRST MAP->ODOM TF, NOW RATE LIMITING THIS NODE @ {(1/(self.callback_period_limit)):.3f} Hz")
                self.get_logger().info(f"########################################################################")
                self.get_logger().info(f"########################################################################")
                self.get_logger().info(f"########################################################################")
                
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
                self.get_logger().info(f"SENT TRANSFORM AFTER INIT")


        #reset flags
        self.triangulated_new_xy = False
        self.measured_new_yaw = False
        self.measured_good_triang = False
        is_measurement_valid = False



    def calculate_robust_tf_avg(self, tf_list: list[TransformStamped], yaw_list: list[float]):
        t = np.array([[tf.transform.translation.x, tf.transform.translation.y] for tf in tf_list])

        # Compute median translation, MAD filtering
        med_t = np.median(t, axis=0)
        mad_t = np.median(np.linalg.norm(t - med_t, axis=1))
        inliers = np.linalg.norm(t - med_t, axis=1) < max(3 * mad_t, 0.5)
        if inliers.sum() < 3:
            inliers[:] = True

        final_t = t[inliers].mean(axis=0)

        # Average yaw using circular mean
        self.get_logger().info(f"yaw list robust initialization: {yaw_list}")
        avg_yaw = np.mean(yaw_list)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = float(final_t[0])
        tf.transform.translation.y = float(final_t[1])
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(avg_yaw)
        
        self.get_logger().info(f"--> INITIALIZED YAW: {(avg_yaw * 180/3.141592):.3f}")

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
            residuals.append((predicted_distance - d_i)**2)
        return residuals


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()