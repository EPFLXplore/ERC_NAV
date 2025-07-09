import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares
import cv2

from ros2_aruco.triangulation import triangulate #custom triangulation code
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

        self.nbr_init_callbacks_for_avg = 25  #25 measurements on initialization to have a good estimate of the map->odom transform with outlier rejection
        # then after that we limit the rate of the listener with the following parameters:
        self.init_callback_counter = 0
        self.initialized_map_odom_tf = False
        self.last_callback_time = self.get_clock().now()
        self.callback_freq_limit = 0.5 #seconds
        self.avg_initialization_tfs = []
        self.yaw_init_list = []

        self.max_translation_jump = 2.0 #meters
        self.max_yaw_jump = math.radians(70) #degrees

        self.max_nbr_triplets = 4
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

        self.high_cov = [1e6 if i in (0,7,35) else 0. for i in range(36)]
        self.low_cov  = [0.005 if i in (0,7) else 0.002 if i==35 else 0. for i in range(36)]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        #create a timer to publish the map->odom transform at a fixed rate to avoid 
        #bugs and warnings
        self.prev_map_odom_tf = None
        self.tf_timer = self.create_timer(0.2, self.republish_map_odom_transform) #5hz

        self.publisher_ = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.subscription 

        # ArUco ID 51 → index 0 of the landmark_poses list
        # ArUco ID 52 → index 1
        # ArUco ID 53 → index 2
        # ...
        #the positions are relative to the map frame which is given to us by the ERC task description
        self.erc_start_pos = [-3.2, 1.48]  #x, y

        self.landmark_poses = [
                (0.705, -3.9),
                (4.0, 3.3),
                (3.2, 0.0),
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

    def get_circle_intersect(self, id1, pose1, id2, pose2):
        #returns the position of the two circle intersection that is closest to the current position (self.erc_start_pos if the system is not initialized yet, and self.curr_map_odom_base_x/y if it is initialized)
        
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

        # 9) pick the one closest to reference pose
        if self.initialized_map_odom_tf:
            rx, ry = self.curr_map_odom_base_x, self.curr_map_odom_base_y
        else:
            rx, ry = self.erc_start_pos

        d1 = math.hypot(I1[0] - rx, I1[1] - ry)
        d2 = math.hypot(I2[0] - rx, I2[1] - ry)

        chosen_intersect = I1 if d1 <= d2 else I2
        chosen_dist = min(d1, d2)

        if chosen_dist > self.max_translation_jump:
            return None
        else:
            return chosen_intersect


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
        for idx, pose in zip(msg.marker_ids, msg.poses):
            if idx < len(self.landmark_poses) and self.landmark_poses[idx][0] < self.MAP_SIZE and self.landmark_poses[idx][1] < self.MAP_SIZE:
                valid_markers.append((idx, pose))

        ids = [idx for idx, _ in valid_markers]
        #self.get_logger().info(f"Valid marker IDs: {ids}")


                #limit the number of pairs in case a lot of arucos are detected and only keep the pairs with the arucos that are the closest to the rover
        pair_scores = []
        for i, j in combinations(range(len(valid_markers)), 2):
            id1, _ = valid_markers[i]
            id2, _ = valid_markers[j]

            x1, y1 = self.landmark_poses[id1]
            x2, y2 = self.landmark_poses[id2]

            ref_x = self.curr_map_odom_base_x if self.initialized_map_odom_tf else self.erc_start_pos[0]
            ref_y = self.curr_map_odom_base_y if self.initialized_map_odom_tf else self.erc_start_pos[1]

            d1 = math.hypot(x1 - ref_x, y1 - ref_y)
            d2 = math.hypot(x2 - ref_x, y2 - ref_y)

            avg_d = 0.5*(d1 + d2)
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

        for i, j in aruco_idx_pairs:

            id1 = marker_ids[i]
            id2 = marker_ids[j]

            if id1 == id2 :
                continue

            # Skip if either landmark is invalid
            if id1 >= len(self.landmark_poses) or id2 >= len(self.landmark_poses):
                continue
            
            lm1 = self.landmark_poses[id1]
            lm2 = self.landmark_poses[id2]
            if abs(lm1[0]) > self.MAP_SIZE or abs(lm2[0]) > self.MAP_SIZE:  #just to check if they have actually been hardcoded in the code
                continue

            dx_map = lm2[0] - lm1[0]
            dy_map = lm2[1] - lm1[1]
            angle_map = math.atan2(dy_map, dx_map) #angle formed by the vector pointing from an aruco to another, in the map frame. this is a theoretical angle

            #poses of the same aruco tags as seen by the rover, so this is in the rover frame
            # pose1 = msg.poses[id1].position
            # pose2 = msg.poses[id2].position
            pose1 = aruco_id_to_pose_dict[id1]
            pose2 = aruco_id_to_pose_dict[id2]

            dx_rover = pose2[0] - pose1[0]
            dy_rover = pose2[1] - pose1[1]
            angle_rover = math.atan2(dy_rover, dx_rover)
            
            yaw_diff = angle_map - angle_rover
            yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi

            #the rover's yaw in the map frame is thus the yaw difference
            yaw_offsets.append(yaw_diff)
            #self.get_logger().info(f"--> Yaw (deg): {(yaw_diff*180/3.141592):.3f} for ids [{id1}-{id2}]")

        
        if len(yaw_offsets) > 0:
            avg_yaw = sum(yaw_offsets)/len(yaw_offsets)
            #self.get_logger().info(f"--> Yaw (deg): {(avg_yaw*180/3.141592):.3f} ")
            self.yaw_estimate = avg_yaw
            self.measured_new_yaw = True
            self.time_of_last_yaw_meas = self.get_clock().now()
        


        # we can make a precise guess using only two landmarks because we know roughly where we are already
        # when the rover starts: We are at self.erc_start_pos assuming we dont fuck up the rover placement at the start of the task.
        # this problem is about finding the intersections of two circles
        # then finding the intersection that is closest to our current location (given by the current map--(aruco)-->odom + odom--(EKF)-->base_link transform).

        if(len(valid_markers) == 2):  
            #find the (x, y) coordinates of the two intersections of the circles

            #valid_marker[i] = (idx, pose)
            id1, pose1 = valid_markers[0]
            id2, pose2 = valid_markers[1]

            intersect_pos = self.get_circle_intersect(id1, pose1, id2, pose2)
            if intersect_pos:
                self.x_estimate, self.y_estimate = intersect_pos
                self.triangulated_new_xy = True
                self.time_of_last_pose = self.get_clock().now()

        if len(valid_markers) == 3: #do triangulation directly but if it fails do 2x circle intersection and take the mean
            (id0, p0), (id1, p1), (id2, p2) = valid_markers
            # build landmarks list and φ-angles
            bearings = [msg.ar_angles_list[i] for i in (0,1,2)]
            bearings_sorted = sorted(bearings) #C, B, A
            bearing_C, bearing_B, bearing_A = bearings_sorted
            phi_1 = abs(bearing_A - bearing_B)
            phi_2 = abs(bearing_B - bearing_C)
            phi_3 = 2*math.pi - phi_1 - phi_2
            phi_angles = [phi_1, phi_2, phi_3]

            #map frame landmakrs in the same C, B, A order
            #need to map the sorted bearings back to indices
            idx_map = {b:i for i,b in enumerate(bearings)}
            #this maps the unsorted bearings value to their original index

            ordered_ids = [ 
                    (id0, idx_map[bearing_C]),
                    (id1, idx_map[bearing_B]),
                    (id2, idx_map[bearing_A]) 
            ]
            landmarks_ordered = [
                (msg.landmark_map_pos_x[idx], msg.landmark_map_pos_y[idx]) for _, idx in ordered_ids
            ]

            # try the analytic 3-point triangulation
            if self.initialized_map_odom_tf:
                ref = [self.curr_map_odom_base_x, self.curr_map_odom_base_y]
            else:
                ref = self.erc_start_pos
            xy = triangulate(landmarks_ordered, phi_angles, ref)

            if xy is not None:
                self.x_estimate, self.y_estimate = xy
                self.triangulated_new_xy      = True
                self.time_of_last_pose        = self.get_clock().now()

            else:
                #fallback: average two circle intersections
                #id0, id1 and id2 are the original indexes in the ArucoMarkers msg
                p01 = self.get_circle_intersect(id0, p0, id1, p1)
                p02 = self.get_circle_intersect(id0, p0, id2, p2)
                candidates = [pt for pt in (p01, p02) if pt is not None]

                if candidates:
                    mx = sum(pt[0] for pt in candidates) / len(candidates)
                    my = sum(pt[1] for pt in candidates) / len(candidates)
                    self.x_estimate, self.y_estimate = mx, my
                    self.triangulated_new_xy         = True
                    self.time_of_last_pose           = self.get_clock().now()
                
            return


        # if >3 landmarks we can triangulate the pose (almost) analytically by using carefully chosen triplets of aruco tags
        if len(valid_markers)>3 and len(valid_markers) <= 7:

            #when there are more than 3 arucos we need to find every pair of triplets of arucos (well limit them to 4 to save computation time)
            #to do that we need to calculate each phi angles for every triplet, then sort every triplet by its lowest phi angle. we will only take the triplets with "the biggest lowest" phi angles.
            #we want the highest minimum phi angles because the smaller the phi angles get the lower our current position estimate error needs to be otherwise the non-linear function (law of Sines)
            #we solve numerically may not converge.

                                                
            #The triangulation code relies on the angles of the aruco tags relative to the rover's frame.
            #we first need to order the detect tags clockwise
            #we then need the position of the detected arucos in the same order
            #we also need the angles between the pairs of arucos ordered the same way
            #we also need the current position (x, y) estimate in the map frame
            #we also need to order the detected arucos in a clockwise manner

            #ordering the aruco tags clockwise by sorting the angles at which they are detected
            #assuming we get the angles of each aruco tag in the rover's ros2 frame in the right hand rule convention of the 
            #ros2 rover frame which is x=forwards, y=left, z=up

            # 1) build all triplets of valid aruco tags

            # we need to limit the number of triplets if there are a lot of arucos detected because it wont be real time otherwise.
            scores = []
            for trip in combinations(range(len(valid_markers)), 3):
                bearings = sorted(msg.ar_angles_list[idx] for idx in trip) #sorted in increasing order, smallest angle = C = index 0, biggest angle = A = index 2
                phi_1 = abs(bearings[2] - bearings[1])
                phi_2 = abs(bearings[1] - bearings[0])
                phi_3 = 2*math.pi - phi_1 - phi_2
                min_phi = min(phi_1, phi_2, phi_3)
                scores.append((trip, min_phi))

            scores.sort(key=lambda x:x[1], reverse=True) #store (trip, min_phi) in decreasing order according to the min_phi angles
            triplet_idxs = [t for t, _ in scores[:self.max_nbr_triplets]]


            # 2) score each triplet by its smallest φ
            triplet_scores = []
            for (i, j, k) in triplet_idxs:
                angles = [msg.ar_angles_list[i], msg.ar_angles_list[j], msg.ar_angles_list[k]]
                angles.sort()  # C, B, A

                theta_C, theta_B, theta_A = angles

                # compute the three interior angles
                phi1 = abs(theta_A - theta_B)                    # APB
                phi2 = abs(theta_B - theta_C)                    # BPC
                phi3 = 2*math.pi - (phi1 + phi2)                 # CPA

                min_phi = min(phi1, phi2, phi3)
                if abs(min_phi)>=10: #guard for low phi angles because the scipy fsolve that solves the non linear function of the triangulation does not like small phi angles
                    triplet_scores.append(((i, j, k), min_phi))

            # 3) take the triplet(s) with the largest minimum‐angle, item[1] is thus the min_phi from above and it is the element we want to sort the triplets by
            #reverse means in descending order so highest min_phi triplet first
            triplet_scores.sort(key=lambda item: item[1], reverse=True)
            top_k = 3 #keep the top 3 triplets and we will average their triangulation later
            best_triplets = [t for t,_ in triplet_scores[:top_k]]

            #list to store the triangulated positions from each triplet
            ar_triplet_pos = []

            for (i, j, k) in best_triplets:
                triplet_bearings = [(msg.ar_angles_list[idx], idx) for idx in (i, j, k)]
                triplet_bearings.sort(key=lambda x: x[0])  # sort by angle in increasing order
                ordered_idxs = [idx for (_, idx) in triplet_bearings]

                 # build landmarks_ordered = [(x_map,y_map), …]
                landmarks_ordered = [
                    (msg.landmark_map_pos_x[idx], msg.landmark_map_pos_y[idx])
                    for idx in ordered_idxs
                ]

                #recompute the phis
                #yaw/bearings of the i, j, and k aruco tags sorted in descending order so highest angle first
                #the biggest angle corresponds to A
                #the medium angle corresponds to B
                #the smallest angle corresponds to C

                bearing_a, bearing_b, bearing_c = [t for (t, _) in sorted(triplet_bearings, reverse=True)] #in decreasing order
                phi_1 = abs(bearing_a - bearing_b)
                phi_2 = abs(bearing_b - bearing_c)
                phi_3 = 2*math.pi - phi_1 - phi_2

                phi_angles_ordered = [phi_1, phi_2, phi_3]

                if self.initialized_map_odom_tf:
                    ref = [self.curr_map_odom_base_x, self.curr_map_odom_base_y]
                else:
                    ref = self.erc_start_pos

                xy = triangulate(
                    landmarks_ordered,
                    phi_angles_ordered, 
                    [ref[0], ref[1]]
                )
            
                if xy is not None:
                    #guard from garbage values:
                    dx = xy[0] - ref[0]
                    dy = xy[1] - ref[1]
                    if math.sqrt(dx*dx + dy*dy) <= 1.5:
                        ar_triplet_pos.append(xy)
                else:
                    #fallback to circle intersections
                    #indexes of valid markers of the triplet that failed
                    a, b, c = (valid_markers[i][0], valid_markers[j][0], valid_markers[k][0])

                    pa = self.get_circle_intersect(a, valid_markers[i][1], b, valid_markers[j][1])
                    pb = self.get_circle_intersect(a, valid_markers[i][1], c, valid_markers[k][1])
                    xy = None
                    if pa is not None and pb is not None:
                        xy = (
                                (pa[0] + pb[0]) * 0.5,
                                (pa[1] + pb[1]) * 0.5
                             )
                    if xy is not None:
                        ar_triplet_pos.append(xy)

            if ar_triplet_pos:
                pos_arr = np.stack(ar_triplet_pos, axis=0)
                mean_xy = pos_arr.mean(axis=0)
                self.x_estimate, self.y_estimate = float(mean_xy[0]), float(mean_xy[1])
                self.triangulated_new_pose = True
                self.triangulated_new_xy   = True
                self.time_of_last_pose     = self.get_clock().now()


        # if >7 aruco landmarks (this will probably never happen but the code is good to have just in case) we can trilaterate our position using recursive least squares
        if len(valid_markers)>7:
            
            #The precision of it heavily depends on the accurate estimation of distance from each camera to the arucos
            #the distance accuracy gets worse if the aruco tags dont face the cameras relatively straight
            #experimentally the distance estimate starts to degrade from 6m onwards.

            distance_estimates = [math.hypot(p.position.x, p.position.y) for _, p in valid_markers]
            landmarks_ordered = [self.landmark_poses[idx] for idx, _ in valid_markers]

            if self.initialized_map_odom_tf:
                map_pos_x = self.curr_map_odom_base_x
                map_pos_y = self.curr_map_odom_base_y
                #bounds = window of size 2*2=4m centered on the current position
                try:
                    # base_estimate = least_squares(self.cost_function, np.array([map_pos_x, map_pos_y]), method= 'trf', loss='soft_l1', f_scale=0.2, 
                    #                             bounds=([map_pos_x - 2.7, map_pos_y -2.7], [map_pos_x + 2.7, map_pos_y + 2.7]), 
                    #                             args=(landmarks_ordered, distance_estimates))

                    base_estimate = least_squares(self.cost_function, np.array([map_pos_x, map_pos_y]), method= 'lm',
                                                args=(landmarks_ordered, distance_estimates))
                except Exception as e:
                    return
            else:
                #the initial position should be around self.erc_start_pos, so we create a square window centered on that position with a width of 2*4=8m
                try:
                    # base_estimate = least_squares(self.cost_function, np.array([self.erc_start_pos[0], self.erc_start_pos[1]]), method= 'trf', loss='soft_l1', f_scale=0.2, 
                    #                             bounds=([self.erc_start_pos[0]-4, self.erc_start_pos[1]-4], [self.erc_start_pos[0]+4, self.erc_start_pos[1]+4]), 
                    #                             args=(landmarks_ordered, distance_estimates))

                    base_estimate = least_squares(self.cost_function, np.array([self.erc_start_pos[0], self.erc_start_pos[1]]), method= 'lm',
                                                args=(landmarks_ordered, distance_estimates))             
                except Exception as e:
                    return

            

            # Only care about x and y
            self.x_estimate = base_estimate.x[0] 
            self.y_estimate = base_estimate.x[1]
            self.triangulated_new_pose = True
            self.triangulated_new_xy = True

            self.time_of_last_pose = self.get_clock().now()


        #now publish the corrected map->odom transform (since we already have odom->base_link done by the EKF)
        #we can use the tf2_ros library to do this
        #we dont directly publish map->base_link because it makes it much easier to reject bad noisy data once the map->odom tf has been initialized
        #because assuming perfect odometry this map->odom should be constant.

        # Publish the transform from map to odom

        # T_map_base = <from ArUco triangulation in map frame>
        # T_odom_base = <from ekf topic: odom→base_link> which is stored in self.odom_pos_x, self.odom_pos_y, self.odom_yaw
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

            old_yaw = math.atan2(self.prev_map_odom_tf.transform.translation.y,
                                self.prev_map_odom_tf.transform.translation.x)
            #self.get_logger().warn(f"Old yaw {old_yaw*180/3.1415:.3f}° deg")

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
        if self.initialized_map_odom_tf:
            self.tf_broadcaster.sendTransform(transform_msg)
            # self.get_logger().info(f"Broadcasted map->odom transform, tx :{transform_msg.transform.translation.x}, ty{transform_msg.transform.translation.y}")
            self.get_logger().info(f"--> Estimated Yaw (deg): {(self.yaw_estimate*180/3.141592):.3f}")
            self.get_logger().info(f"--> Estimate X in map: {(self.x_estimate):.3f}")
            self.get_logger().info(f"--> Estimated Y in map: {(self.y_estimate):.3f}")
            #self.get_logger().info(f"-------------------------------------")

        # Store the last pose for the next iteration
        self.prev_map_odom_tf = transform_msg


        if(self.init_callback_counter < self.nbr_init_callbacks_for_avg):

            self.init_callback_counter += 1
            self.avg_initialization_tfs.append(transform_msg)
            self.yaw_init_list.append(self.yaw_estimate)
            #self.get_logger().info(f"--> Estimated yaw in map DURING INIT: {(self.yaw_estimate*180/3.141592):.3f}")


            if (self.init_callback_counter == self.nbr_init_callbacks_for_avg):

                self.initialized_map_odom_tf = True
                
                self.prev_map_odom_tf = self.calculate_robust_tf_avg(self.avg_initialization_tfs, self.yaw_init_list)
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
                self.get_logger().info(f"--> INITIALIZED FIRST MAP->ODOM TF, NOW RATE LIMITING THIS NODE @ {(1/(self.callback_freq_limit)):.3f} Hz")


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
        avg_yaw = np.mean(yaw_list)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = float(final_t[0])
        tf.transform.translation.y = float(final_t[1])
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(avg_yaw)
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