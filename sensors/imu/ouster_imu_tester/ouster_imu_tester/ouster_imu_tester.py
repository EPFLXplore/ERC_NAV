import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from collections import deque


class MovingAverageFilterNode(Node):
    def __init__(self):
        super().__init__('moving_average_filter_node')

        self.window_size = 10  # Sliding window size for moving average
        #100hz --> 0.01s per value --> 0.15s of data for it to be considered valid

        self.alpha = 0.6       # alpha = 1: no filtering, alpha = 0.01: strong filtering

        self.accel_windows = {'x': deque(maxlen=self.window_size), 
                              'y': deque(maxlen=self.window_size), 
                              'z': deque(maxlen=self.window_size)}
        self.gyro_windows = {'x': deque(maxlen=self.window_size), 
                             'y': deque(maxlen=self.window_size), 
                             'z': deque(maxlen=self.window_size)}

        self.prev_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}

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

        self.update_window(self.accel_windows, msg.linear_acceleration)
        self.update_window(self.gyro_windows, msg.angular_velocity)

        accel_filtered = self.apply_filters(self.accel_windows, self.prev_accel)
        gyro_filtered = self.apply_filters(self.gyro_windows, self.prev_gyro)

        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.linear_acceleration.x = accel_filtered['x']
        filtered_msg.linear_acceleration.y = accel_filtered['y']
        filtered_msg.linear_acceleration.z = accel_filtered['z']
        filtered_msg.angular_velocity.x = gyro_filtered['x']
        filtered_msg.angular_velocity.y = gyro_filtered['y']
        filtered_msg.angular_velocity.z = gyro_filtered['z']
        self.publisher.publish(filtered_msg)

    def update_window(self, windows, data):
        windows['x'].append(data.x)
        windows['y'].append(data.y)
        windows['z'].append(data.z)

    def apply_filters(self, windows, prev_values):
        filtered = {}
        for axis in ['x', 'y', 'z']:
            moving_avg = sum(windows[axis]) / len(windows[axis])
            filtered[axis] = self.alpha * moving_avg + (1 - self.alpha) * prev_values[axis]
            prev_values[axis] = filtered[axis]
        return filtered


def main(args=None):
    rclpy.init(args=args)
    node = MovingAverageFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
