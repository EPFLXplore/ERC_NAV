#!/usr/bin/python3
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimOdomPublisher(Node):
    def __init__(self):
        super().__init__("sim_odom_publisher")

        self.clock = None

        self.create_subscription(
            Clock,
            "/clock",
            self.update_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.odom_publisher = self.create_publisher(Odometry, "/sim/odom", 10)
        self.create_timer(0.1, self.publish_odom)

    def publish_odom(self):
        if self.clock is None:
            return

        msg = Odometry()

        msg.header.stamp = self.clock
        msg.header.frame_id = "odom"

        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = np.zeros(36)

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        msg.twist.covariance = np.zeros(36)

        self.odom_publisher.publish(msg)

    def update_clock(self, msg):
        self.clock = msg.clock


def main():
    rclpy.init()
    node = SimOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
