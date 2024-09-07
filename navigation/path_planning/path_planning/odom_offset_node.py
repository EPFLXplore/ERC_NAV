#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations
from geometry_msgs.msg import Quaternion
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class OdometryOffsetNode(Node):

    def __init__(self):
        super().__init__("odom_offset_node")
    
        package_name = 'path_planning' 
        config_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'offset.yaml')
        self.offset_position, self.offset_orientation = self.load_offsets_from_config(config_file_path)

        self.odom_subscriber = self.create_subscription(
            Odometry, "/lio_sam/mapping/odometry", self.odom_callback, QoSProfile(depth=10, reliability = ReliabilityPolicy.BEST_EFFORT)
        )
        self.odom_publisher = self.create_publisher(Odometry, "/odom_with_offset", 10)

    def load_offsets_from_config(self, config_file_path):
    
        with open(config_file_path, "r") as file:
            config = yaml.safe_load(file)
        position_offset = np.array(config["position_offset"])
        orientation_offset = config["orientation_offset"]
        return position_offset, orientation_offset

    def odom_callback(self, msg):
        msg.pose.pose.position.x += self.offset_position[0]
        msg.pose.pose.position.y += self.offset_position[1]
        msg.pose.pose.position.z += self.offset_position[2]

        current_orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        offset_quaternion = self.offset_orientation

        new_orientation = tf_transformations.quaternion_multiply(
            current_orientation, offset_quaternion
        )

        msg.pose.pose.orientation = Quaternion(
            x=new_orientation[0],
            y=new_orientation[1],
            z=new_orientation[2],
            w=new_orientation[3],
        )

        self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryOffsetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
