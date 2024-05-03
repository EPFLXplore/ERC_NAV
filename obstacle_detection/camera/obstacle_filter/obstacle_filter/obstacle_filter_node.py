import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import numpy as np
import cv2
from cv_bridge import CvBridge # might need to sudo apt install ros-foxy-vision-opencv
import time

# Important
# from .filter.ransac_filter import getPC
# from .filter.lstsq_filter import getPC_lstsq
from .filter.filter import getPC

def point_cloud(points, parent_frame): #, stamp):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame) #, stamp=stamp)

    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

class ObstacleFilterNode(Node):
    def __init__(self):
        super().__init__('NAV_obstacle_filter_node')

        # declare subscriber to topic /stereo/depth of type Image
        self.subscription_ = self.create_subscription(
            Image,
            '/stereo/depth',
            self.callback,
            qos_profile_sensor_data)
        
        self.destroy_sub = self.create_subscription(
            String,
            "/ROVER/NAV_status",
            self.callback_destroy,
            qos_profile_sensor_data)

        # declare publisher to topic /obstacles_points of type PointCloud2
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/obstacles_points',
            qos_profile_sensor_data)
        
        # declare publisher to topic /obstacles_points of type PointCloud2
        self.publisher_2 = self.create_publisher(
            PointCloud2,
            '/not_obstacles_points',
            qos_profile_sensor_data)

        # declare parameters to be given as input to the 
        self.declare_parameter('gridshape_y', 8)    # 8x8 grid by default = 64 sub images
        self.declare_parameter('gridshape_x', 8)
        self.declare_parameter('max_depth', 3000)   # 3 meters
        self.declare_parameter('min_depth', 100)    # 10 cm
        self.declare_parameter('num_iter', 50)      # 50 iterations for ransac
        self.declare_parameter('thresh_dist', 20)   # 20 mm distance treashold for ransac (inliers)
        self.declare_parameter('num_inliers', 360)  # 360 inliers for ransac for a valid plane
        self.declare_parameter('camera_angle', 17)  # 17 degrees camera angle (hardcoded for now)
        self.declare_parameter('obstacle_angle', 45)    # 45 degrees obstacle angle
        self.declare_parameter('how_to_replace', 'random')  

        self.declare_parameter('filter', 'ransac')
        self.declare_parameter('device', 'cpu')

        # there may be a need for a ground vector parameter

        # declare CvBridge object, which converts between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def callback(self, msg):
        # print('Received message')

        # t1 = time.time()

        # get parameters values
        gridshape_y = self.get_parameter('gridshape_y').get_parameter_value().integer_value
        gridshape_x = self.get_parameter('gridshape_x').get_parameter_value().integer_value
        max_depth = self.get_parameter('max_depth').get_parameter_value().integer_value
        min_depth = self.get_parameter('min_depth').get_parameter_value().integer_value
        num_iter = self.get_parameter('num_iter').get_parameter_value().integer_value
        thresh_dist = self.get_parameter('thresh_dist').get_parameter_value().integer_value
        num_inliers = self.get_parameter('num_inliers').get_parameter_value().integer_value
        camera_angle = self.get_parameter('camera_angle').get_parameter_value().integer_value
        obstacle_angle = self.get_parameter('obstacle_angle').get_parameter_value().integer_value

        how_to_replace = self.get_parameter('how_to_replace').get_parameter_value().string_value
        filter_bool = self.get_parameter('filter').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value

        # convert ROS Image message to OpenCV image to numpy array
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        depth_array = np.array(cv_img, dtype=np.float32)

        # Filter out all points that are not obstacles

        # t2 = time.time()

        # print("Time for message processing: ", t2-t1)
        
        # points, not_points = getPC_lstsq(depth_array, \
        points, not_points = getPC(depth_array, \
                    (gridshape_y, gridshape_x), max_depth, min_depth, \
                    num_iter, thresh_dist, num_inliers, how_to_replace, \
                    filter_bool, device, camera_angle, obstacle_angle)
        
        # t3 = time.time()

        # print("Time for filtering", t3-t2)
        
        points = points.numpy() / 1000.0 # convert to meters
        print(points.shape)

        not_points = not_points.numpy() / 1000

        # Filter out all points that are not obstacles
        pcd = point_cloud(points, msg.header.frame_id)
        pcd2 = point_cloud(not_points, msg.header.frame_id)
        self.publisher_.publish(pcd)
        self.publisher_2.publish(pcd2)

        # t4 = time.time()

        # print("Time for publishing: ", t4-t3)
        # print("Time for all: ", t4-t1)

    def callback_destroy(self, msg):
        if msg.data == "abort":
            self.destroy_node()
            rclpy.shutdown()
        


def main(args=None):
    print('Obstacle Filter Node started')
    rclpy.init(args=args)

    obstacle_filter_node = ObstacleFilterNode()

    rclpy.spin(obstacle_filter_node)

    obstacle_filter_node.destroy_node()
    rclpy.shutdown()