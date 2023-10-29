#! /usr/bin/env python3

import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
 
# from cs_interface.cs_interface_node  import BasicNavigator, NavigationResult # Helper module
from cs_interface.robot_navigator import BasicNavigator, NavigationResult
 
'''
Navigates a robot from an initial pose to a goal pose.
'''



class SubGoal(Node):
           
    def __init__(self):
        super().__init__('new_receive_goal')

        self.sub_goal = self.create_subscription(
                PoseStamped,
                '/ROVER/NAV_goal',
                self.goal_update,
                10)
        
        self.navigator = BasicNavigator()

        # self.navigator.waitUntilNav2Active()

        print('init 2')
        self.i = 0
    

    def goal_update(self, msg):
        print('goal update')
        self.navigator.goToPose(msg)
        print('nav')
        self.feedback_goal(msg)


    def feedback_goal(self,msg):
        print('feedback')
        

        # Keep doing stuff as long as the robot is moving towards the goal
        while not self.navigator.isNavComplete():
            print('goal not complete')


        # Do something with the feedback
        self.i = self.i + 1
        feedback = self.navigator.getFeedback()

        print('feedback2', feedback)

        if feedback and self.i % 5 == 0:
            print('Distance remaining: ' + '{:.2f}'.format(
                feedback.distance_remaining) + ' meters.')


        print('feedback3')
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        print('feedback4')






def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SubGoal()

    rclpy.spin(minimal_subscriber)

 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('MAIN')
    main()