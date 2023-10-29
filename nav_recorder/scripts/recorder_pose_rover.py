#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

mux

# from tf2_ros.transformations import euler_from_quaternion
#from tf_transformations import euler_from_quaternion
import yaml
import math

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RecorderRoverPoseNode(Node):
    def __init__(self,source='/lio_sam/current_pose',destination="initial_position.yaml"):
        super().__init__('recorder_rover_pose')



        self.sub = self.create_subscription(
            PoseStamped, '/lio_sam/current_pose', self.callback_record, 10)
        


        self.counter = 0

        if not destination.lower().endswith('.yaml'):
            destination += '.yaml'
        self.destination = destination

 

    def callback_record(self, data):
        self.get_logger().info("Recording position")
        print('yo')

        pos = data.pose.position
        ort = data.pose.orientation


        pos = [pos.x, pos.y, pos.z]
        print('ort', ort)
        # ort = [ort.x, ort.y, ort.z, ort.w]
        print('pos', pos)
        print('ort', ort)
        ort = euler_from_quaternion(ort)


        with open(self.destination, 'w') as file:
            to_print = ''
            for label, val in zip(["p" + c for c in "xyz"], pos):
                to_print += "{} : {}\n".format(label, val)
            for label, val in zip(["r" + c for c in "xyz"], ort):
                to_print += "{} : {}\n".format(label, val)
            file.write(to_print)

        self.counter += 1
        if self.counter > 10:
            self.counter = 0
            with open("_slow.".join(self.destination.split(".")), 'w') as file:
                to_print = ''
                for lbl, val in zip(["p" + c for c in "xyz"], pos):
                    to_print += "{} : {}\n".format(lbl, val)
                for lbl, val in zip(["r" + c for c in "xyz"], ort):
                    to_print += "{} : {}\n".format(lbl, val)
                file.write(to_print)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode(
        source='/lio_sam/current_pose', 
        destination='initial_position.yaml')
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()