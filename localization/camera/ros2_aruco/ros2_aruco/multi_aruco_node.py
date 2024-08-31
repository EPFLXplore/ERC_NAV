import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
from ros2_aruco import transformations
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class MultiViewArucoNode(Node):

    def __init__(self):
        super().__init__('multi_view_aruco_node')
        self.get_logger().info("Multi camera ")

        
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

            self.declare_parameter("marker_size", .175)

        else:

            self.declare_parameter("image_topic_1", '/oak/rgb/image_raw')
            self.declare_parameter("camera_info_topic_1", '/oak/rgb/camera_info')
            self.declare_parameter("camera_frame_1", "oak-d-base-frame")

            self.declare_parameter("image_topic_2", '/camera/camera/color/image_raw')
            self.declare_parameter("camera_info_topic_2", '/camera/camera/color/camera_info')
            self.declare_parameter("camera_frame_2", "camera_link")

            # self.declare_parameter("marker_size", .125)
            self.declare_parameter("marker_size", .17)


        self.base_frame = "base_link"
    
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value

        # Subscribers
        self.image_sub_1 = Subscriber(self, Image, self.get_parameter("image_topic_1").get_parameter_value().string_value)
        self.image_sub_2 = Subscriber(self, Image, self.get_parameter("image_topic_2").get_parameter_value().string_value)

        # Subsribers to camera info topic to get intrisic matrix and distortion
        self.info_sub_1 = self.create_subscription(CameraInfo, 
                                                   self.get_parameter("camera_info_topic_1").get_parameter_value().string_value, 
                                                   self.info_callback_1, 
                                                   qos_profile_sensor_data)
        self.info_sub_2 = self.create_subscription(CameraInfo, 
                                                   self.get_parameter("camera_info_topic_2").get_parameter_value().string_value, 
                                                   self.info_callback_2, 
                                                   qos_profile_sensor_data)

        # For tf transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # synchronized call back that processes two images at the same time
        self.ts = ApproximateTimeSynchronizer([self.image_sub_1, self.image_sub_2], queue_size=10, slop=0.3)
        self.ts.registerCallback(self.synced_callback)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.bridge = CvBridge()
        self.info_msg_1 = None
        self.info_msg_2 = None
        self.intrinsic_mat_1 = None
        self.intrinsic_mat_2 = None
        self.distortion_1 = None
        self.distortion_2 = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.__getattribute__(dictionary_id_name))
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

    def info_callback_1(self, info_msg):
        self.info_msg_1 = info_msg
        self.intrinsic_mat_1 = np.reshape(np.array(self.info_msg_1.k), (3, 3))
        self.distortion_1 = np.array(self.info_msg_1.d)
        self.destroy_subscription(self.info_sub_1)

    def info_callback_2(self, info_msg):
        self.info_msg_2 = info_msg
        self.intrinsic_mat_2 = np.reshape(np.array(self.info_msg_2.k), (3, 3))
        self.distortion_2 = np.array(self.info_msg_2.d)
        self.destroy_subscription(self.info_sub_2)

    def synced_callback(self, img_msg_1, img_msg_2):
        self.get_logger().info("Callback")
        
        if self.info_msg_1 is None or self.info_msg_2 is None:
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

        self.get_logger().info("Publishing")

        # Publish
        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(markers) 



    def process_image(self, img_msg, intrinsic_mat, distortion, camera_frame, markers, pose_array):
        
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        
        if marker_ids is not None:

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, intrinsic_mat, distortion)
            
            for i, marker_id in enumerate(marker_ids):

                pose = Pose()
                pose.position.x = tvecs[i][0][2]
                pose.position.y = tvecs[i][0][0]
                pose.position.z = tvecs[i][0][1]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                # transform position from camera frame to base_link frame
                try:
                    transform = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, rclpy.time.Time())
                    pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                except tf2_ros.LookupException as ex:
                    self.get_logger().warn(f"Transform lookup failed: {ex}")
                    return

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0]-50) # ERC IDs go from 51 to 64
        else:
            return



            

def main():
    rclpy.init()
    node = MultiViewArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
