#!/usr/bin/env python3

"""
pkg:    magellan
node:   NAV_magellan_talking
topics: 
        publish:    /NAV/nav_nodes_state
        subscribe:  /CS/nav_nodes_state
        
description:  

"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MagellanTalking(Node):

    def __init__(self):
        super().__init__('NAV_magellan_talking')
        self.pub_intern_com = self.create_publisher(String, '/NAV/nav_nodes_state', 10)

        self.sub_cs_instruction = self.create_subscription(
            String,
            '/CS/nav_nodes_state',
            self.cs_listener_callback,
            10)



        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = String()
        
        
        if self.i==10:  
            msg.data = "NAV_HELLO_WOLRD"
        elif self.i>50:  
            msg.data = "NAV_AUTONOMOUS_START"
        else:
            msg.data = 'Hello World: %d' % self.i

        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.pub_intern_com.publish(msg)


    def cs_listener_callback(self, msg):
        
        msg_to_send = String()
        
        if msg.data == "CS_AUTONOMOUS_START":
            msg_to_send = "NAV_AUTONOMOUS_START"
 




def main(args=None):
    rclpy.init(args=args)

    shutdown_publisher = MagellanTalking()

    rclpy.spin(shutdown_publisher)

    shutdown_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()