#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class PublisherVelocity(Node):
    def __init__(self):
        super().__init__("publisher_velocity_controller")
       
        controller_name = "velocity_controller"
        wait_sec_between_publish = 0.5
        self.goals = [100.0, -100.0, -100.0, 100.0]

        publish_topic = "/" + controller_name + "/" + "commands"

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    #ordre actuel: FL,FR,RR,RL
    
    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.goals
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pub = PublisherVelocity()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()