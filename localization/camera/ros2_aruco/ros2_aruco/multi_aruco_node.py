import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
from ros2_aruco import transformations
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ros2_aruco_interfaces.msg import ArucoMarkers
from custom_msg.srv import CameraParams
from functools import partial
import math

from scipy.spatial.transform import Rotation as R


class MultiViewArucoNode(Node):

    def __init__(self):
        super().__init__('multi_view_aruco_node')
        self.get_logger().info("Multi camera ")

        #half of the aruco box depth
        self.aruco_box_offset = 0.125
        self.max_aruco_dist = 15.0 #meters

        self.cam_params_received = {"left": False, "right": False}
        self.params_initialized = False
        self.sync_started = False
        
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")

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

            
        #NAV Realsense Cameras
        #camera nav1 : serial number 102122061110
        self.declare_parameter("image_topic_1", '/NAV/feed_camera_nav_1')
        self.declare_parameter("camera_frame_1", "intel_realsense_D415_camera_top_right_1")
        
        #camera nav2: serial number 135322062945
        self.declare_parameter("image_topic_2", '/NAV/feed_camera_nav_2')
        self.declare_parameter("camera_frame_2", "intel_realsense_D415_camera_top_left_1")

        #camera front oakd
        self.declare_parameter("image_topic_3", "/NAV/feed_camera_nav_0")
        self.declare_parameter("camera_frame_3", "NAV_Front_Camera_1")

        #CS Cameras
        #topic names:
        # /ROVER/feed_camera_cs_0
        # /ROVER/feed_camera_cs_1
        # /ROVER/feed_camera_cs_2
        # /ROVER/feed_camera_cs_3

        self.declare_parameter("image_topic_4", '/ROVER/feed_camera_cs_0')
        self.declare_parameter("camera_frame_4", "Logitech_Brio_100_top_right_1")

        self.declare_parameter("image_topic_5", '/ROVER/feed_camera_cs_1')
        self.declare_parameter("camera_frame_5", "Logitech_Brio_100_top_left_1")

        self.declare_parameter("image_topic_6", '/ROVER/feed_camera_cs_2')
        self.declare_parameter("camera_frame_6", "Logitech_Brio_100_front_right_1")

        self.declare_parameter("image_topic_7", '/ROVER/feed_camera_cs_3')
        self.declare_parameter("camera_frame_7", "Logitech_Brio_100_front_left_1")

        self.declare_parameter("marker_size", .144)


        self.base_frame = "base_link"

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value

        # Subscribers for nav and CS cameras
        self.image_sub_1 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_1").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        self.image_sub_2 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_2").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        self.image_sub_3 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_3").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)

        #CS cams:
        #self.image_sub_4 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_4").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_5 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_5").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_6 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_6").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_7 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_7").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)

        # For tf transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # synchronized callback that processes images at the same time, slop = window of time for synced images
        #self.ts = ApproximateTimeSynchronizer([self.image_sub_1, self.image_sub_2, self.image_sub_3, self.image_sub_4, self.image_sub_5, self.image_sub_6], queue_size=2, slop=0.05)
        self.ts = ApproximateTimeSynchronizer([self.image_sub_1, self.image_sub_2, self.image_sub_3], queue_size=2, slop=0.05)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.bridge = CvBridge()
        self.intrinsic_mat_1 = None
        self.intrinsic_mat_2 = None
        self.distortion_1 = None
        self.distortion_2 = None

        self.distortion_oakd = np.array([3.03940603e-01, -2.28313655e+00, -4.79004397e-03,  3.08347206e-03])
        self.intrinsic_mat_oakd = np.array(
            [[1.96685610e+03, 0.00000000e+00, 9.99954556e+02],
            [0.00000000e+00, 1.96039881e+03, 4.79997092e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        )
        #We assume all Logitech Brio 100 cameras have the same parameters because fuck this I am NOT measuring all parameters for all cameras

        # Reprojection RMS error: 0.3174
        # Camera matrix (fx,  0, cx;
        #                 0, fy, cy;
        #                 0,  0,  1):
        # [[2.85166130e+03 0.00000000e+00 1.26999005e+03]
        # [0.00000000e+00 2.84698625e+03 4.62916150e+02]
        # [0.00000000e+00 0.00000000e+00 1.00000000e+00]] 

        # Distortion coefficients [k1, k2, p1, p2, k3]:
        # [0.04712347 1.61133451 0.00499591 0.03045891 3.26455985]

        self.distortion_cs = np.array([0.04712347, 1.61133451, 0.00499591, 0.03045891, 3.26455985])
        self.intrinsic_mat_cs = np.array(
            [[2.85166130e+03, 0, 1.26999005e+03], 
            [0, 2.84698625e+03, 4.62916150e+02], 
            [0, 0, 1]]
        )
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.__getattribute__(dictionary_id_name))
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Camera Intrinsics acquisition
        self.client_left = self.create_client(CameraParams, "/NAV/camera_info_102122061110")
        self.client_right = self.create_client(CameraParams, "/NAV/camera_info_135322062945")
        self.timer = self.create_timer(1.0, self.wait_for_cameras_to_start)


    def wait_for_cameras_to_start(self):
        if not (self.client_left.service_is_ready() and self.client_right.service_is_ready()):
            self.get_logger().debug("Waiting for CameraParams services…")
            return

        # stop retry timer once services become available
        self.timer.cancel()

        request_left = CameraParams.Request()
        future_left = self.client_left.call_async(request_left)
        future_left.add_done_callback(partial(self.callback_cam_params, camera="left"))

        request_right = CameraParams.Request()
        future_right = self.client_right.call_async(request_right)
        future_right.add_done_callback(partial(self.callback_cam_params, camera="right"))


    def callback_cam_params(self, future, camera):
        if self.params_initialized:
            return  # Already good
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed for {camera} camera: {e}")
            return

        if (
            response.fx == 0
            or response.fy == 0
            or response.cx == 0
            or response.cy == 0
            or len(response.distortion_coefficients) == 0
        ):
            #self.get_logger().warn(f"Invalid intrinsics from {camera} camera – retrying…")
            self.timer = self.create_timer(1.0, self.wait_for_cameras_to_start)
            return

        distortion = np.array(response.distortion_coefficients)
        intrinsic_mat = np.array(
            [[response.fx, 0, response.cx], [0, response.fy, response.cy], [0, 0, 1]]
        )

        if camera == "left":
            self.distortion_1 = distortion
            self.intrinsic_mat_1 = intrinsic_mat
        else:
            self.distortion_2 = distortion
            self.intrinsic_mat_2 = intrinsic_mat

        self.cam_params_received[camera] = True
        #self.get_logger().info(f"Received intrinsics for {camera} camera.")

        if all(self.cam_params_received.values()) and not self.sync_started:
            #self.get_logger().info("Both camera intrinsics received – starting synchroniser")
            self.ts.registerCallback(self.synced_callback)
            self.params_initialized = True
            self.sync_started = True


    #def synced_callback(self, img_msg_1, img_msg_2, img_msg_3, img_msg_4, img_msg_5, img_msg_6):
    def synced_callback(self, img_msg_1, img_msg_2, img_msg_3):

        #self.get_logger().warn("sycned callback 2 cams")
        
        if self.intrinsic_mat_1 is None or self.intrinsic_mat_2 is None:
            #self.get_logger().warn("No camera info has been received!")
            return
        
        markers = ArucoMarkers() # custom msg => ID + position
        pose_array = PoseArray() # for vizualization on rviz

        # publish base_link position 
        markers.header.frame_id =  self.base_frame # camera_frame
        pose_array.header.frame_id = self.base_frame #camera_frame

        markers.header.stamp = img_msg_1.header.stamp
        pose_array.header.stamp = img_msg_1.header.stamp

        # Populate the array of positions, by processing each image sequentially
        self.process_image(img_msg_1, self.intrinsic_mat_1, self.distortion_1, self.get_parameter("camera_frame_1").get_parameter_value().string_value, markers, pose_array)
        self.process_image(img_msg_2, self.intrinsic_mat_2, self.distortion_2, self.get_parameter("camera_frame_2").get_parameter_value().string_value, markers, pose_array)
        self.process_image(img_msg_3, self.intrinsic_mat_oakd, self.distortion_oakd, self.get_parameter("camera_frame_3").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_4, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_4").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_5, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_5").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_6, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_6").get_parameter_value().string_value, markers, pose_array)

        # self.get_logger().info("Publishing")

        # Publish
        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(markers) 


    def calculate_aruco_box_bearing(self, tvec, rvec):
        R_tag_to_cam, _ = cv2.Rodrigues(rvec)
        offset = np.array([[0], [0], [-self.aruco_box_offset]])
        offset_cam = R_tag_to_cam @ offset
        box_center_tvec = tvec.reshape((3, 1)) + offset_cam
        x = box_center_tvec[0, 0]
        z = box_center_tvec[2, 0]
        bearing_rad = math.atan2(-x, z)

        yaw_rot = R.from_euler('z', bearing_rad, degrees=False)
        #matrix representing the transform of the aruco tag in the camera frame
        R_yaw = yaw_rot.as_matrix()
        T_cam_box = np.eye(4) #identity 4x4 matrix
        T_cam_box[0:3, 0:3] = R_yaw
        T_cam_box[0:3, 3] = box_center_tvec.flatten()

        return math.degrees(bearing_rad), T_cam_box

    def transform_to_matrix(self, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        T = np.eye(4)
        R_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[0:3, 0:3] = R_mat
        T[0:3, 3] = [t.x, t.y, t.z]
        return T

    # This function creates a 4x4 transformation matrix from x, y, and yaw
    # It is used to convert a pose estimate into a transformation matrix
    def pose_to_mat(self, x, y, yaw):
                c, s = math.cos(yaw), math.sin(yaw)
                M = np.eye(4) # 4x4 identity matrix
                #rotation matrix for 2D rotation
                M[0:2,0:2] = [[c, -s],
                              [s,  c]]
                #translation vector
                M[0,3], M[1,3] = x, y
                return M


######################################################################
    def process_image(self, img_msg, intrinsic_mat, distortion, camera_frame, markers, pose_array):
        marker_candidates = {}  # {marker_id: [(tvec, rot_matrix, yaw_deg)]}
        
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Keep original detection method

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
	    #corners = list of detected marker corners. For each marker, its four corners are provided. For N detected markers, the dimensions of this array is Nx4. The order of the corners is clockwise. 

        if marker_ids is not None:

            #cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
            #cv2.imshow("Detected ArUco Markers", cv_image)
            #cv2.waitKey(1)  # Allow OpenCV window refresh
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, intrinsic_mat, distortion)
            #self.get_logger().info(f"Markers detected: {marker_ids}")

            # get the transform position from camera frame to base_link frame
            #first argument = target frame
            #second arg = source frame
            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
            except tf2_ros.LookupException as ex:
                
                #self.get_logger().warn(f"Transform lookup failed (URDF probably isnt published): {ex}")
                return


            all_markers = []  # Store all detected markers (ID, pose, yaw)
            distinct_ids = set()  # Track unique marker IDs

            # Step 1: Store all detections per ID
            for i, marker_id in enumerate(marker_ids):

                # ignore marker if too far away => measurement is imprecise
                tvec_ar = tvecs[i][0]
                if np.linalg.norm(tvec_ar) > self.max_aruco_dist:
                    self.get_logger().debug(f"Ignoring marker {marker_id[0]} (>{self.max_aruco_dist} m)")
                    continue

                marker_id = marker_id[0]  # Extract integer ID
                distinct_ids.add(marker_id)  # Track distinct marker IDs

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

                # Compute yaw angle from rotation matrix
                R_tag2cam, _ = cv2.Rodrigues(rvecs[i])
                R_cam2tag = R_tag2cam.T
                R_180_x = R.from_euler('x', 180, degrees=True).as_matrix()
                R_corrected = R_180_x @ R_cam2tag
                # R_desired = R.from_matrix(R_corrected).inv()
                R_desired = R.from_matrix(R_corrected)
                euler = R_desired.as_euler('xyz', degrees=True)

                # if str(marker_id) == '51':
                #      self.get_logger().info(f"euler angles {euler}")

                # Store all detected markers
                if marker_id not in marker_candidates:
                    marker_candidates[marker_id] = []
                marker_candidates[marker_id].append((tvecs[i][0], rot_matrix, abs(euler[1]), rvecs[i][0]))

            #self.get_logger().info(f"distinc ids set: {distinct_ids}")

            #Only keep the best aruco face for each box
            if len(distinct_ids) >= 1:
                #self.get_logger().info("Filtering markers with worst yaw angles")
                
                for marker_id in marker_candidates.keys():
                    # Sort detections by lowest yaw angle (least distortion)
                    marker_candidates[marker_id].sort(key=lambda x: x[2], reverse=False)  # Sort by yaw_deg
                    #self.get_logger().info(f"marker id: {marker_id}, angles: {marker_candidates[marker_id]} ")
                    # Keep only the best (lowest yaw) detection per ID
                    marker_candidates[marker_id] = [marker_candidates[marker_id][0]]


            # else:
            #     self.get_logger().info("Skipping yaw filtering (Exactly 2 distinct IDs)")

        else:
            # self.get_logger().info("No markers detected.")
            return

        # **Publish only the best markers (lowest yaw per ID)**
        for marker_id, marker_list in marker_candidates.items():
            for tvec, rot_matrix, best_yaw, rvec in marker_list:  # Handle multiple detections per ID
                pose = Pose()

                # Offset ArUco pose to the center of the box face
                face_to_center_offset = np.array([0, 0, -self.aruco_box_offset])
                adjusted_tvec = tvec + rot_matrix[0:3, 0:3] @ face_to_center_offset
                
                # Convert to ROS2 frame
                pose.position.x = adjusted_tvec[2]
                pose.position.y = -adjusted_tvec[0]
                pose.position.z = -adjusted_tvec[1]

                #transform = camera --> base_link
                
                try: 
                    #pose from opencv : camera-->aruco
                    pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                except  tf2_ros.LookupException as ex:
                    #self.get_logger().warn(f"TF transform failed: {ex}")
                    return
                
                # **ERC ID Mapping Logic (Preserved)**
                erc_to_index = lambda erc_id: erc_id - 51 if 51 <= erc_id <= 65 else None
                aruco_index = erc_to_index(marker_id)

                # **Only publish valid ERC markers**
                #but first verify if they are redundant tags (2x the same ID) because it should have been filtered
                if aruco_index is not None:
                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(aruco_index)
                    #self.get_logger().info(f"Publishing Marker ID {marker_id} (ERC Index {aruco_index})")

                    bearing_deg, tf_cam_box = self.calculate_aruco_box_bearing(tvec, rvec)
                    bearing_deg = normalize_angle_deg(bearing_deg)
                    #markers.ar_angles_list.append(bearing_deg)
                    #self.get_logger().info(f"bearing in camera frame for aruco {aruco_index+51} : {bearing_deg}°")
                    
                    t_base_link_to_aruco_box = self.transform_to_matrix(transform) @ tf_cam_box                    
                    R_base_box = t_base_link_to_aruco_box[0:3, 0:3]

                    r = R.from_matrix(R_base_box)
                    euler = r.as_euler('xyz', degrees=True)
                    yaw_deg = euler[2]# yaw is third angle in 'sxyz' convention
                    yaw_deg = normalize_angle_deg(yaw_deg)
                    #self.get_logger().info(f"bearing in rover frame for aruco {aruco_index+51} : {yaw_deg}°")

                    #setting the corresponding marker position in the map frame
                    ar_map_x, ar_map_y = self.landmark_poses[aruco_index]
                    markers.landmark_map_pos_x.append(ar_map_x)
                    markers.landmark_map_pos_y.append(ar_map_y)
                    markers.ar_angles_list.append(yaw_deg)



def normalize_angle_deg(deg):
    return (deg + 180) % 360 - 180

        

########################################################################

            

def main():
    rclpy.init()
    node = MultiViewArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
