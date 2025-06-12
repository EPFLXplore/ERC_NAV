#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRetimestampNode(Node):
    def __init__(self):
        super().__init__('imu_retimestamp_node')
        # Declare parameters for input and output topics
        self.declare_parameter('input_imu_topic', '/olive/imu/id001/ahrs')
        self.declare_parameter('output_imu_topic', '/olive/imu/id001/ahrs/restamped')

        input_topic = self.get_parameter('input_imu_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_imu_topic').get_parameter_value().string_value

        # QoS profile: best-effort, moderate depth
        qos = rclpy.qos.QoSProfile(
            depth=200,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)

        # Subscription to the original IMU topic
        self.sub = self.create_subscription(Imu, input_topic, self.imu_callback, qos)

        # Publisher for the restamped IMU topic
        self.pub = self.create_publisher(Imu, output_topic, qos)

        self.get_logger().info(f"Subscribed to IMU on '{input_topic}', republishing on '{output_topic}' with Best-Effort QoS")

    def imu_callback(self, msg: Imu):
        # Restamp and republish
        new_msg = Imu()
        # Stamp with current ROS time
        now = self.get_clock().now().to_msg()
        new_msg.header.stamp = now
        # Preserve frame_id or override if needed
        new_msg.header.frame_id = msg.header.frame_id  # or set to a fixed known frame, e.g., "imu_link"

        # Copy the IMU data fields
        new_msg.orientation = msg.orientation
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Publish the restamped message
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRetimestampNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()