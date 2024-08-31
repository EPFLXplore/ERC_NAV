import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
import math
import tf2_ros
import tf2_geometry_msgs



class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        self.get_logger().info("Unique camera ")


        # Declare and read parameters
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")

        self.base_frame = "base_link"
        self.declare_parameter('sim', False) 
        sim = self.get_parameter('sim').get_parameter_value().bool_value  

        if sim:
            self.declare_parameter("image_topic", '/oak_rgb/image_raw')
            self.declare_parameter("camera_info_topic", '/oak_rgb/camera_info')
            self.declare_parameter("camera_frame", "oak_camera_link")
            self.declare_parameter("marker_size", .175)
   
        else:
            self.declare_parameter("image_topic", '/oak/rgb/image_raw')
            self.declare_parameter("camera_info_topic", '/oak/rgb/camera_info')
            self.declare_parameter("camera_frame", "oak-d-base-frame")
            self.declare_parameter("marker_size", .125) #TODO measure


        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary =  cv2.aruco.Dictionary_get(cv2.aruco.__getattribute__(dictionary_id_name))
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)


    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')

        markers = ArucoMarkers() # custom msg => ID + position
        pose_array = PoseArray() # for vizualization on rviz
        
        # Publish position of base frame
        markers.header.frame_id = self.base_frame
        pose_array.header.frame_id = self.base_frame    
        
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
       

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,self.aruco_dictionary, parameters=self.aruco_parameters)
        if marker_ids is not None:

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
        
            for i, marker_id in enumerate(marker_ids):

                pose = Pose()
                pose.position.y = tvecs[i][0][0]
                pose.position.z = tvecs[i][0][1]
                pose.position.x = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                # transform position from camera frame to base_link frame
                try:
                    transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
                    pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                except tf2_ros.LookupException as ex:
                    self.get_logger().warn(f"Transform lookup failed: {ex}")
                    return

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0]-50)

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()