import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu


class LowPassFilterNode(Node):
    def __init__(self):
        super().__init__('low_pass_filter_node')

        # Parameters
        self.alpha = 0.1  # alpha = 0 : extreme low pass, alpha=1 : no low pass
        self.prev_accel_x = 0.0
        self.prev_accel_y = 0.0
        self.prev_accel_z = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/ouster_imu',
            self.imu_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            Imu,
            '/ouster_imu_lowpass',
            qos_profile
        )

    def imu_callback(self, msg):
        # Create a new IMU message for filtered data
        filtered_msg = Imu()
        filtered_msg.header = msg.header

        filtered_msg.linear_acceleration.x = (
            self.alpha * msg.linear_acceleration.x +
            (1 - self.alpha) * self.prev_accel_x
        )
        filtered_msg.linear_acceleration.y = (
            self.alpha * msg.linear_acceleration.y +
            (1 - self.alpha) * self.prev_accel_y
        )
        filtered_msg.linear_acceleration.z = (
            self.alpha * msg.linear_acceleration.z +
            (1 - self.alpha) * self.prev_accel_z
        )

        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.orientation = msg.orientation

        self.prev_accel_x = filtered_msg.linear_acceleration.x
        self.prev_accel_y = filtered_msg.linear_acceleration.y
        self.prev_accel_z = filtered_msg.linear_acceleration.z

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowPassFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
