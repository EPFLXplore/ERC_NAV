#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cs_interface.cs_interface_node import BasicNavigator

class SendNavGoal(Node):




    def __init__(self):
        super().__init__('send_goal')
        
        self.declare_parameter('x', 3.0) 
        self.declare_parameter('y', 0.0) 
        self.declare_parameter('w', 1.0) 
        
        
        #arg use to cancel the current goal, need to be set to true to cancel
        self.declare_parameter('cancel_goal',False) 
        self.declare_parameter('abort',False) 
        

        self.x=self.get_parameter('x').value
        self.y=self.get_parameter('y').value 
        self.w=self.get_parameter('w').value 
        
        self.cancel = self.get_parameter("cancel_goal").value
        self.abort = self.get_parameter("abort").value
        self.publisher_cancel = self.create_publisher(String, '/ROVER/NAV_status', 10)
        
        if self.cancel:
            msg = String()
            msg.data = "cancel"
            self.publisher_cancel.publish(msg)
            print('cancel goal')
        
        else:
            self.publisher_ = self.create_publisher(PoseStamped, '/ROVER/NAV_goal', 10)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.x
            goal_pose.pose.position.y = self.y
            goal_pose.pose.orientation.w = self.w
            self.publisher_.publish(goal_pose)
            #timer_period = 0.5  # seconds
            #self.timer = self.create_timer(timer_period, self.timer_callback)
            #self.i = 0

            print('send goal', self.cancel)
        
        
        
        if self.abort:
            msg = String()
            msg.data = "abort"
            self.publisher_cancel.publish(msg)
            print('abort')
            
    
        	
        


    def timer_callback(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = self.x
        goal_pose.pose.position.y = self.y
        goal_pose.pose.orientation.w = self.w
        self.publisher_.publish(goal_pose)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    pub = SendNavGoal()

    rclpy.spin(pub)

    
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
   main()
