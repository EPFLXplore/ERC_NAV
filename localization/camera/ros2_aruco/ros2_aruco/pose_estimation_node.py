import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import math
from scipy.optimize import least_squares
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
        self.callback_freq_limit = 0.2 #seconds
        self.avg_initialization_tfs = []
        self.yaw_init_list = []

        self.max_translation_jump = 1.5 #meters
        self.max_yaw_jump = math.radians(70) #degrees

        self.max_bearing_diff = 150.0
        self.min_bearing_diff = 20.0

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
        self.erc_start_pos = [0.0, 0.0]  #x, y

        self.landmark_poses = [
                (3.27, -0.8),
                (-3.58, 2.49),
                (-1.4, -0.72),
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
        
        
    def are_points_clockwise(self, vertices):
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
        self.get_logger().info(f"AB: {AB}, AC: {AC}")
        
        cross_prod = AB[0]*AC[1] - AB[1]*AC[0]

        return cross_prod < 0 #true if clockwise, false if ccw


    def get_circle_intersect(self, id1, pose1, id2, pose2):
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

        # 9) pick the one closest to reference pose
        if self.initialized_map_odom_tf:
            rx, ry = self.curr_map_odom_base_x, self.curr_map_odom_base_y
        else:
            rx, ry = self.erc_start_pos

        d1 = math.hypot(I1[0] - rx, I1[1] - ry)
        d2 = math.hypot(I2[0] - rx, I2[1] - ry)

        chosen_intersect = I1 if d1 <= d2 else I2
        chosen_dist_to_ref = min(d1, d2)

        if chosen_dist_to_ref > self.max_translation_jump:
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
            self.get_logger().info(f"2 CIRCLE INTERSECT")

            #valid_marker[i] = (idx, pose)
            id1, pose1 = valid_markers[0]
            id2, pose2 = valid_markers[1]

            intersect_pos = self.get_circle_intersect(id1, pose1, id2, pose2)
            if intersect_pos:
                self.x_estimate, self.y_estimate = intersect_pos
                self.triangulated_new_xy = True
                self.time_of_last_pose = self.get_clock().now()




        if len(valid_markers) == 3: #do triangulation directly but if it fails do 2x circle intersection and take the mean
            self.get_logger().info(f"1 TRIANGULATION")
            #id0, id1, id2 are aruco ids (0 =>aruco tag nbr 51, 1=>52 etc...)
            (id0, p0), (id1, p1), (id2, p2) = valid_markers #[(aruco id, pose), ...]
            
            are_aruco_ids_ccw = False

            #check the orientation of the tags:
            are_aruco_ids_ccw = self.are_points_clockwise([
                                        self.landmark_poses[id0],
                                        self.landmark_poses[id1],
                                        self.landmark_poses[id2]
                                    ])

            if not are_aruco_ids_ccw:
                #id0, id1, id2 are oriented clockwise
                #we can thus set A = aruco at map pose self.landmark_poses[id0] etc...
                #but we also need to find the corresponding angles
                bearings_clockwise = [msg.ar_angles_list[i] for i in (0,1,2)] 
                landmarks_ordered = [self.landmark_poses[id0], self.landmark_poses[id1], self.landmark_poses[id2]]

            else:
                bearings_clockwise = [msg.ar_angles_list[i] for i in (0,1,2)] 
                bearings_clockwise.reverse()
                landmarks_ordered = [self.landmark_poses[id2], self.landmark_poses[id1], self.landmark_poses[id0]]


            self.get_logger().info(f"clockwise bearings: {bearings_clockwise}")
            # build landmarks list and φ-angles
            bearing_A, bearing_B, bearing_C = bearings_clockwise

            phi_1 = abs(bearing_A - bearing_B)
            phi_2 = abs(bearing_B - bearing_C)
            phi_3 = (360.0 - phi_1 - phi_2)
            phi_angles = [phi_1, phi_2, phi_3]
            #sanity check to see if it will probably not converge numerically:
            if (self.min_bearing_diff > abs(phi_1) > self.max_bearing_diff) or 
               (self.min_bearing_diff > abs(phi_2) > self.max_bearing_diff) or 
               (self.min_bearing_diff > abs(phi_3) > self.max_bearing_diff):

                self.get_logger().info(f"triangulation will most probably fail")


            # try the optimization triangulation
            if self.initialized_map_odom_tf:
                ref = [self.curr_map_odom_base_x, self.curr_map_odom_base_y]
            else:
                ref = self.erc_start_pos

            self.get_logger().info(f"sent to opti triangulation: landmarks {landmarks_ordered}, phi_angles: {phi_angles}, ref pos {ref}")
            xy = triangulate_opti(landmarks_ordered, phi_angles, ref)

            if xy is not None:
                self.x_estimate, self.y_estimate = xy
                self.triangulated_new_xy      = True
                self.time_of_last_pose        = self.get_clock().now()

            else:
                self.get_logger().info(f"opti triang failed, fallback to circle intersects")
                # fallback of the fallback: average two circle intersections
                # id0, id1 and id2 are the original indexes in the ArucoMarkers msg
                p01 = self.get_circle_intersect(id0, p0, id1, p1)
                p02 = self.get_circle_intersect(id0, p0, id2, p2)
                candidates = [pt for pt in (p01, p02) if pt is not None]

                if candidates:
                    mx = sum(pt[0] for pt in candidates) / len(candidates)
                    my = sum(pt[1] for pt in candidates) / len(candidates)
                    self.x_estimate, self.y_estimate = mx, my
                    self.get_logger().info(f"circle fallback: x={mx}, y={my}")
                    self.triangulated_new_xy         = True
                    self.time_of_last_pose           = self.get_clock().now()
                
            return




        # if >3 landmarks we can triangulate the pose analytically by using carefully chosen triplets of aruco tags
        if len(valid_markers)>3 and len(valid_markers) <= 7:
            self.get_logger().info(f"MULTIPLE TRIANGULATIONS")


            #when there are more than 3 arucos we need to find every pair of triplets of arucos (well limit them to self.max_nbr_triplets to save computation time)
            #to do that we need to calculate each phi angles (angle between two aruco tags) for every triplet, then sort every triplet by its lowest phi angle. 
            #we will only take the triplets with the biggest minimum phi angles because the smaller the phi angles get the lower our current position estimate error 
            #needs to be otherwise the non-linear function (law of Sines) we solve numerically may not converge.
  
            #The triangulation code relies on the angles of the aruco tags relative to the rover's frame given by muli_aruco_node.py's ArucoMarkers msg
            #we first need to order the detect tags clockwise in the map frame, this can be done by comparing the angles of each tag in the rover frame
            #we then need the position of the detected arucos in the same order as the ordered angles
            #we also need the current position (x, y) estimate in the map frame

            # 1) build all triplets of valid aruco tags

            
            

            # we need to limit the number of triplets if there are a lot of arucos detected because it wont be real time otherwise.
            scores = []
            for trip in combinations(range(len(valid_markers)), 3):

                bearings = sorted(msg.ar_angles_list[idx] for idx in trip) #sorted in increasing order, smallest angle = C = index 0, biggest angle = A = index 2
                #pre-filter arucos by throwing away any two markers whose bearing difference
                # is below a certain threshold, say 20° but we dont want it to be too big (close to 180°) either because it is the singularity of the system, so
                # the non-linear equation of the geometric triangulation won't converge and the triangulation using triangulation won't converge either.
                # so between 20° and 150° seems like a good fit.  
                triplet_is_valid = True
                for a, b in ((trip[0], trip[1]), (trip[0],  trip[2]), (trip[1], trip[2])):
                    delta_bearing= abs(msg.ar_angles_list[a] - msg.ar_angles_list[b])
                    delta_bearing = abs((delta_bearing + 180.0) % 360.0 - 180.0) #wrap to [0, 180]

                    if delta_bearing > self.max_bearing_diff or delta_bearing < self.min_bearing_diff:
                        triplet_is_valid = False
                        break
                if not triplet_is_valid:
                    continue #go to the next triplet without doing any more computations to save time

                phi_1 = abs(bearings[2] - bearings[1])
                phi_2 = abs(bearings[1] - bearings[0])
                phi_3 = 360.0 - phi_1 - phi_2
                min_phi = min(phi_1, phi_2, phi_3)
                scores.append((trip, min_phi))

            scores.sort(key=lambda x:x[1], reverse=True) #store (trip, min_phi) in decreasing order according to the min_phi angles
            triplet_idxs = [t for t, _ in scores[:self.max_nbr_triplets]]


            # 2) score each triplet by its smallest φ to select only the best triplets from the pre-filtered ones
            triplet_scores = []
            for (i, j, k) in triplet_idxs:
                angles = [msg.ar_angles_list[i], msg.ar_angles_list[j], msg.ar_angles_list[k]]
                #we get the angles of each aruco tag in the rover's ros2 frame in the right hand rule convention of the 
                #ros2 rover frame which is x=forwards, y=left, z=up
                angles.sort()  # C, B, A

                theta_C, theta_B, theta_A = angles

                # compute the three interior angles
                phi1 = abs(theta_A - theta_B)                 # APB
                phi2 = abs(theta_B - theta_C)                 # BPC
                phi3 = (360.0 - (phi1 + phi2) )               # CPA

                min_phi = min(phi1, phi2, phi3)
                max_phi = max(phi1, phi2, phi3)

                if abs(min_phi) >= self.min_bearing_diff and abs(max_phi) <= self.max_bearing_diff: #guard just in case for low and high phi angles to avoid not converging numerically
                    triplet_scores.append(((i, j, k), min_phi))

            # 3) take the triplet(s) with the largest minimum‐angle, item[1] is thus the min_phi from above and it is the element we want to sort the triplets by
            #reverse means in descending order so highest min_phi triplet first

            #do note that we could also keep the best ones by filtering by the smallest max_phi but since all triplets here are supposed to make the triangulation converge, both are good metrics.
            triplet_scores.sort(key=lambda item: item[1], reverse=True)
            #keep the top 4 triplets and we will average their triangulation later
            best_triplets = [t for t,_ in triplet_scores[:self.max_nbr_triplets-1]]

            #list to store the triangulated positions from each triplet
            ar_triplet_pos = []

            if best_triplets:
                for (i, j, k) in best_triplets:
                    (id0, p0), (id1, p1), (id2, p2) = valid_markers[i], valid_markers[j], valid_markers[k]

                    are_aruco_ids_ccw = False
                    #check the orientation of the tags:
                    are_aruco_ids_ccw = self.are_points_clockwise([
                                                self.landmark_poses[id0],
                                                self.landmark_poses[id1],
                                                self.landmark_poses[id2]
                                            ])
                    if not are_aruco_ids_ccw:
                        #id0, id1, id2 are oriented clockwise
                        #we can thus set A = aruco at map pose self.landmark_poses[id0] etc...
                        #but we also need to find the corresponding angles
                        bearings_clockwise = [msg.ar_angles_list[x] for x in (i,j,k)]
                        landmarks_ordered = [self.landmark_poses[id0], self.landmark_poses[id1], self.landmark_poses[id2]]

                    else:
                        bearings_clockwise = [msg.ar_angles_list[x] for x in (i,j,k)]
                        bearings_clockwise.reverse()
                        landmarks_ordered = [self.landmark_poses[id2], self.landmark_poses[id1], self.landmark_poses[id0]]

                    bearing_A, bearing_B, bearing_C = bearings_clockwise

                    phi_1 = abs(bearing_A - bearing_B)
                    phi_2 = abs(bearing_B - bearing_C)
                    phi_3 = (360.0 - phi_1 - phi_2)
                    phi_angles = [phi_1, phi_2, phi_3]


                    if self.initialized_map_odom_tf:
                        ref = [self.curr_map_odom_base_x, self.curr_map_odom_base_y]
                    else:
                        ref = self.erc_start_pos

                    xy = triangulate_opti(landmarks_ordered, phi_angles, ref)
                
                    if xy is not None:
                        #guard from garbage values:
                        dx = xy[0] - ref[0]
                        dy = xy[1] - ref[1]
                        if math.sqrt(dx*dx + dy*dy) <= 1.5:
                            ar_triplet_pos.append(xy)
                    else:
                        xy = None
                        self.get_logger().info(f"geom triang failed, fallback to circle intersects")
                        # fallback of the fallback: average two circle intersections
                        # id0, id1 and id2 are the original indexes of the failed triplet aruco in the ArucoMarkers msg
                        p01 = self.get_circle_intersect(id0, p0, id1, p1)
                        p02 = self.get_circle_intersect(id0, p0, id2, p2)
                        candidates = [pt for pt in (p01, p02) if pt is not None]

                        if candidates:
                            mx = sum(pt[0] for pt in candidates) / len(candidates)
                            my = sum(pt[1] for pt in candidates) / len(candidates)
                            self.get_logger().info(f"1 trinagulation fallback: x={mx}, y={my}")
                            xy = np.array([mx, my])

                        if xy is not None:
                            ar_triplet_pos.append(xy)

                if ar_triplet_pos:
                    pos_arr = np.stack(ar_triplet_pos, axis=0)
                    mean_xy = pos_arr.mean(axis=0)
                    self.x_estimate, self.y_estimate = float(mean_xy[0]), float(mean_xy[1])
                    self.triangulated_new_pose = True
                    self.triangulated_new_xy   = True
                    self.time_of_last_pose     = self.get_clock().now()

            else: #if there are no good triplets = all triplets have a min_phi <= 20°
                #do classic trilateration via optimization
                self.get_logger().info(f"no good triplets found. doing classic distance trilateration")
                distance_estimates = [math.hypot(p.position.x, p.position.y) for _, p in valid_markers]
                landmarks_ordered = [self.landmark_poses[idx] for idx, _ in valid_markers]

                if self.initialized_map_odom_tf:
                    map_pos_x = self.curr_map_odom_base_x
                    map_pos_y = self.curr_map_odom_base_y
                    #bounds = window of size 2*2=4m centered on the current position
                    try:

                        base_estimate = least_squares(self.cost_function, np.array([map_pos_x, map_pos_y]), method= 'lm',
                                                    args=(landmarks_ordered, distance_estimates))
                    except Exception as e:
                        return
                else:
                    #the initial position should be around self.erc_start_pos, so we create a square window centered on that position with a width of 2*4=8m
                    try:

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

                    base_estimate = least_squares(self.cost_function, np.array([map_pos_x, map_pos_y]), method= 'lm',
                                                args=(landmarks_ordered, distance_estimates))
                except Exception as e:
                    return
            else:
                #the initial position should be around self.erc_start_pos, so we create a square window centered on that position with a width of 2*4=8m
                try:

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
        
        self.get_logger().info(f"--> INITIALIZED YAW: {(avg_yaw):.3f}")

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