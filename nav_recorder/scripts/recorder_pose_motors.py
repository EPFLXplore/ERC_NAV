#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped
from custom_msg.msg import Wheelstatus



# from tf2_ros.transformations import euler_from_quaternion
#from tf_transformations import euler_from_quaternion
import yaml
import math



class RecorderMotorPoseNode(Node):
    def __init__(self,topic='/lio_sam/current_pose',yaml_file="initial_position.yaml"):
        super().__init__('recorder_motors_pose')



        self.sub = self.create_subscription(
            Wheelstatus, topic, self.callback_record, 10)
        


        self.counter = 0

        if not yaml_file.lower().endswith('.yaml'):
            yaml_file += '.yaml'
        self.yaml_file = yaml_file
        
        
        self.driving_values = [None, None, None, None]
        self.steering_values = [None, None, None, None]

 

    def callback_record(self, data):
        self.get_logger().info("Recording position")
        print('yo')

        driving = data.driving
        steering = data.steering


        driving = [driving[0], driving[1], driving[2], driving[4]]
        steering = [steering[0], steering[1], steering[2], steering[4]]
 
	for i in range(len(steering)):
            if steering[i] < 2⁽¹⁷⁾:
                self.steering_values[i] = steering[i]


        self.driving_values = driving
        
        with open(self.yaml_file, 'w') as file:
            to_print = ''
            for label, val in zip(["driving" + c for c in "1234"], driving):
                to_print += "{} : {}\n".format(label, val)

            for label, val in zip(["r" + c for c in "xyz"], steering):
                to_print += "{} : {}\n".format(label, val)
            file.write(to_print)

        self.counter += 1
        if self.counter > 10:
            self.counter = 0
            with open("_slow.".join(self.yaml_file.split(".")), 'w') as file:
                to_print = ''
                for lbl, val in zip(["p" + c for c in "1234"], driving):
                    to_print += "{} : {}\n".format(lbl, val)
                for lbl, val in zip(["r" + c for c in "1234"], steering):
                    to_print += "{} : {}\n".format(lbl, val)
                file.write(to_print)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderMotorPoseNode(
        topic='', 
        yaml_file='pose_motors.yaml')
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
