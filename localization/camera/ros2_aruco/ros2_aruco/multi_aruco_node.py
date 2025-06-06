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
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from custom_msg.srv import CameraParams
from functools import partial
import math


class MultiViewArucoNode(Node):

    def __init__(self):
        super().__init__('multi_view_aruco_node')
        self.get_logger().info("Multi camera ")

        #half of the aruco box depth
        self.aruco_box_offset = 0.125

        self.cam_params_received = {"left": False, "right": False}
        self.params_initialized = False
        self.sync_started = False
        
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")

        self.declare_parameter('sim', False) 
        sim = self.get_parameter('sim').get_parameter_value().bool_value  
        
        if sim:

            self.declare_parameter("image_topic_1", '/oak_rgb/image_raw')
            self.declare_parameter("camera_info_topic_1", '/oak_rgb/camera_info')
            self.declare_parameter("camera_frame_1", "oak_camera_link")

            self.declare_parameter("image_topic_2", '/intel_rgb/image_raw')
            self.declare_parameter("camera_info_topic_2", '/intel_rgb/camera_info')
            self.declare_parameter("camera_frame_2", "intel_camera_link")

            self.declare_parameter("marker_size", .15)

        else:
            
            #NAV Realsense Cameras

            self.declare_parameter("image_topic_1", '/NAV/feed_camera_nav_1')
            self.declare_parameter("camera_frame_1", "left_realsense_camera_link")

            self.declare_parameter("image_topic_2", '/NAV/feed_camera_nav_2')
            self.declare_parameter("camera_frame_2", "right_realsense_camera_link")

            #CS Cameras
            #topic names:
            # /ROVER/feed_camera_cs_0
            # /ROVER/feed_camera_cs_1
            # /ROVER/feed_camera_cs_2
            # /ROVER/feed_camera_cs_3

            self.declare_parameter("image_topic_3", '/ROVER/feed_camera_cs_0')
            self.declare_parameter("camera_frame_3", "CS_cam_0")

            self.declare_parameter("image_topic_4", '/ROVER/feed_camera_cs_1')
            self.declare_parameter("camera_frame_4", "CS_cam_1")

            self.declare_parameter("image_topic_5", '/ROVER/feed_camera_cs_2')
            self.declare_parameter("camera_frame_5", "CS_cam_2")

            self.declare_parameter("image_topic_6", '/ROVER/feed_camera_cs_3')
            self.declare_parameter("camera_frame_6", "CS_cam_3")

            self.declare_parameter("marker_size", .144)


        self.base_frame = "base_link"
    
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value

        # Subscribers for nav and CS cameras
        self.image_sub_1 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_1").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        self.image_sub_2 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_2").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #CS cams:
        #self.image_sub_3 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_3").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_4 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_4").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_5 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_5").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)
        #self.image_sub_6 = Subscriber(self, CompressedImage, self.get_parameter("image_topic_6").get_parameter_value().string_value, qos_profile=qos_profile_sensor_data)

        # For tf transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # synchronized callback that processes images at the same time, slop = window of time for synced images
        #self.ts = ApproximateTimeSynchronizer([self.image_sub_1, self.image_sub_2, self.image_sub_3, self.image_sub_4, self.image_sub_5, self.image_sub_6], queue_size=2, slop=0.05)
        self.ts = ApproximateTimeSynchronizer([self.image_sub_1, self.image_sub_2], queue_size=2, slop=0.05)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.bridge = CvBridge()
        self.intrinsic_mat_1 = None
        self.intrinsic_mat_2 = None
        self.distortion_1 = None
        self.distortion_2 = None
        
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
            [[2.85166130e+03, 0, 1.26999005e+03], [0, 2.84698625e+03, 4.62916150e+02], [0, 0, 1]]
        )
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.__getattribute__(dictionary_id_name))
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Camera Intrinsics acquisition
        self.client_left = self.create_client(CameraParams, "/NAV/camera_info_102122061110")
        self.client_right = self.create_client(CameraParams, "/NAV/camera_info_135322062945")
        self.timer = self.create_timer(0.5, self.wait_for_cameras_to_start)


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
        self.get_logger().info(f"Received intrinsics for {camera} camera.")

        if all(self.cam_params_received.values()) and not self.sync_started:
            self.get_logger().info("Both camera intrinsics received – starting synchroniser  AHHHHHH")
            self.ts.registerCallback(self.synced_callback)
            self.params_initialized = True
            self.sync_started = True


    #def synced_callback(self, img_msg_1, img_msg_2, img_msg_3, img_msg_4, img_msg_5, img_msg_6):
    def synced_callback(self, img_msg_1, img_msg_2):

        self.get_logger().warn("sycned callback 2 cams")
        
        if self.intrinsic_mat_1 is None or self.intrinsic_mat_2 is None:
            self.get_logger().warn("No camera info has been received!")
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
        # self.process_image(img_msg_3, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_3").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_4, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_4").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_5, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_5").get_parameter_value().string_value, markers, pose_array)
        # self.process_image(img_msg_6, self.intrinsic_mat_cs, self.distortion_cs, self.get_parameter("camera_frame_6").get_parameter_value().string_value, markers, pose_array)

        #self.get_logger().info("Publishing")

        # Publish
        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(markers) 



######################################################################
    def process_image(self, img_msg, intrinsic_mat, distortion, camera_frame, markers, pose_array):
        marker_candidates = {}  # {marker_id: [(tvec, rot_matrix, yaw_deg)]}
        
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Keep original detection method

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        if marker_ids is not None:

            #cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
            #cv2.imshow("Detected ArUco Markers", cv_image)
            #cv2.waitKey(1)  # Allow OpenCV window refresh
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, intrinsic_mat, distortion)
            self.get_logger().info(f"Markers detected: {marker_ids}")

            # Transform position from camera frame to base_link frame
            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
            except tf2_ros.LookupException as ex:
                self.get_logger().warn(f"Transform lookup failed (URDF probably isnt published lol): {ex}")
                return

            all_markers = []  # Store all detected markers (ID, pose, yaw)
            distinct_ids = set()  # Track unique marker IDs

            # Step 1: Store all detections per ID
            for i, marker_id in enumerate(marker_ids):
                marker_id = marker_id[0]  # Extract integer ID
                distinct_ids.add(marker_id)  # Track distinct marker IDs

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

                # Compute yaw angle from rotation matrix
                sy = math.sqrt(rot_matrix[0, 0]**2 + rot_matrix[1, 0]**2)
                yaw = 0 if sy < 1e-6 else math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
                yaw_deg = abs(math.degrees(yaw))  # Use absolute value to compare tilt

                # Store all detected markers
                if marker_id not in marker_candidates:
                    marker_candidates[marker_id] = []
                marker_candidates[marker_id].append((tvecs[i][0], rot_matrix, yaw_deg))

            # Step 2: **Apply Filtering Except When There Are <= 2 Distinct IDs**
            if len(distinct_ids) > 2:
                self.get_logger().info("Filtering markers with worst yaw angles")
                
                for marker_id in marker_candidates.keys():
                    # Sort detections by lowest yaw angle (least distortion)
                    marker_candidates[marker_id].sort(key=lambda x: x[2], reverse=True)  # Sort by yaw_deg
                    # Keep only the best (lowest yaw) detection per ID
                    marker_candidates[marker_id] = [marker_candidates[marker_id][0]]

            else:
                self.get_logger().info("Skipping yaw filtering (Exactly 2 distinct IDs)")

        else:
            self.get_logger().info("No markers detected.")
            return

        # **Publish only the best markers (lowest yaw per ID)**
        for marker_id, marker_list in marker_candidates.items():
            for tvec, rot_matrix, best_yaw in marker_list:  # Handle multiple detections per ID
                pose = Pose()

                # Offset ArUco pose to the center of the box face
                face_to_center_offset = np.array([0, 0, -self.aruco_box_offset])
                adjusted_tvec = tvec + rot_matrix[0:3, 0:3] @ face_to_center_offset

                # Convert to ROS2 frame
                pose.position.x = adjusted_tvec[2]
                pose.position.y = -adjusted_tvec[0]
                pose.position.z = -adjusted_tvec[1]
                
                #The rotation matrix is the rotation from the camera to the aruco frame
                #However we do not know how the arcuo frame is rotated with respect to the map frame so this isnt useful for pose estimation
                
                # quat = transformations.quaternion_from_matrix(rot_matrix)
                # pose.orientation.x = quat[0]
                # pose.orientation.y = quat[1]
                # pose.orientation.z = quat[2]
                # pose.orientation.w = quat[3]

                try:
                    pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                except  tf2_ros.LookupException as ex:
                    self.get_logger().warn(f"TF transform failed: {ex}")
                    return

                # **ERC ID Mapping Logic (Preserved)**
                erc_to_index = lambda erc_id: erc_id - 51 if 51 <= erc_id <= 65 else None
                aruco_index = erc_to_index(marker_id)

                # **Only publish valid ERC markers**
                if aruco_index is not None:
                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(aruco_index)
                    self.get_logger().info(f"Publishing Marker ID {marker_id} (ERC Index {aruco_index})")
        

########################################################################

            

def main():
    rclpy.init()
    node = MultiViewArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
