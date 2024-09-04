#!/usr/bin/python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimMapPublisher(Node):
    def __init__(self):
        super().__init__("sim_map_publisher")

        self.clock = None

        self.create_subscription(Odometry, "/odom", self.publish_aruco, 10)
        self.create_subscription(
            Clock,
            "/clock",
            self.update_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.aruco_publisher = self.create_publisher(Odometry, "/sim/map", 10)

    def publish_aruco(self, msg):
        if self.clock is None:
            return

        msg.header.stamp = self.clock
        msg.header.frame_id = "map"

        msg.child_frame_id = "base_link"

        self.aruco_publisher.publish(msg)

    def update_clock(self, msg):
        self.clock = msg.clock


def main():
    rclpy.init()
    node = SimMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
