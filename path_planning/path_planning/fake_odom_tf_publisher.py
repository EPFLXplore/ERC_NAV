#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


class FakeOdomTfPublisher(Node):
    def __init__(self):
        super().__init__("fake_odom_tf_publisher")

        # Initialize the transform broadcaster
        self.odom_tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, "/odom", self.publish_odom_tf, 10)

    def publish_odom_tf(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        pose = msg.pose.pose

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        # Send the transformation
        self.odom_tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FakeOdomTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
