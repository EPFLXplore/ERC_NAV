#!/usr/bin/env python3

"""
pkg:    wheels_commands
node:   cs_shutdown_publisher
topics: 
        publish:    /CS/nav_shutdown_cmds
        subscribe:  
        
description:  

"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ShutdownPublisher(Node):

    def __init__(self):
        super().__init__('cs_shutdown_publisher')
        self.publisher_ = self.create_publisher(String, 'CS/nav_shutdown_cmds', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        
        
        if self.i==50:  
            msg.data = "NAV_SHUTDOWN"
        else:
            msg.data = 'Hello World: %d' % self.i

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    shutdown_publisher = ShutdownPublisher()

    rclpy.spin(shutdown_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shutdown_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()