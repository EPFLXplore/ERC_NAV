import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_srvs.srv import SetBool
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import cv2
from cv_bridge import CvBridge
import threading

import time
from time import sleep

global stop_threads

class NewCameras(Node):
    def __init__(self):

        super().__init__('new_cameras_cs')

        self.camera_ids = ["/dev/video14", "/dev/video6", "/dev/video16"] # 14, 6: cs // 0, 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # publishers for the cameras
        self.cam_pubs = [self.create_publisher(CompressedImage, 'camera_' + str(i), qos_profile) for i in range(len(self.camera_ids))]
        self.bridge = CvBridge()

        global stop_threads
        stop_threads = False
        self.threads = []

        self.service = self.create_service(SetBool, '/ROVER/start_cameras', self.start_cameras_callback)
        
        self.threads = [threading.Thread(target=publish_feeds, args=(self.camera_ids[i], self.cam_pubs[i], self.bridge,)) for i in range(len(self.camera_ids))]

        self.get_logger().info("Cameras ready")

        #for thread in self.threads:
        #    thread.start()

    def start_cameras_callback(self, request, response):
        self.get_logger().info("REQUEST")
        global stop_threads
        if request.data:
            stop_threads = False

            self.threads = [threading.Thread(target=publish_feeds, args=(self.camera_ids[i], self.cam_pubs[i], self.bridge,)) for i in range(len(self.camera_ids))]

            for thread in self.threads:
                thread.start()
            response.success = True
            response.message = "Cameras started"
        else:
            stop_threads = True
            for thread in self.threads:
                thread.join()

            self.threads = []
            
            response.success = True
            response.message = "Cameras stopped"
        return response



def publish_feeds(camera_id, publisher, bridge):
    print("Starting camera " + camera_id)
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
    camera.set(cv2.CAP_PROP_FPS, 15)
    global stop_threads
    
    while True:
        print("Open " + camera_id)

        for i in range(10):
            ret, frame = camera.read()

        image_idx = 0
        while True:
            print("Capturing " + str(image_idx) + " | time: " + str(time.time()))
            ret, frame = camera.read()
            print("Captured " + str(image_idx) + " | time: " + str(time.time()))
            if (not ret) or stop_threads:
                break
            
            compressed_image = bridge.cv2_to_compressed_imgmsg(frame)
            print("Compressed " + str(image_idx) + " | time: " + str(time.time()))
            publisher.publish(compressed_image)
            print("Sent " + str(image_idx) + " | time: " + str(time.time()))
            image_idx += 1
            sleep(1/15)

        if stop_threads:
            break
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        camera.set(cv2.CAP_PROP_FPS, 15)
        sleep(1)





def main(args=None):

    print("Start cameras_publisher node...")
    
    rclpy.init(args=args)

    cameras_publisher = NewCameras()
    
    print("Cameras ready")

    rclpy.spin(cameras_publisher)

    # cameras_publisher.stop_camera()
    cameras_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

