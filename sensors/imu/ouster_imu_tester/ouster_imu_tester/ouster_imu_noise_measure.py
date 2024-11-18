import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np


class IMUNoiseCalculatorNode(Node):
    def __init__(self):
        super().__init__('imu_noise_calculator')

        # QoS profile for best-effort communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscription to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/ouster_imu',
            self.imu_callback,
            qos_profile
        )

        # Variables to store IMU data
        self.acc_data = {'x': [], 'y': [], 'z': []}
        self.gyr_data = {'x': [], 'y': [], 'z': []}
        self.sample_count = 0
        self.max_samples = 600  # Number of samples to collect

        self.get_logger().info("IMU Noise Calculator Node initialized. Waiting for data...")

    def imu_callback(self, msg):
        # Collect accelerometer data
        self.acc_data['x'].append(msg.linear_acceleration.x)
        self.acc_data['y'].append(msg.linear_acceleration.y)
        self.acc_data['z'].append(msg.linear_acceleration.z)

        # Collect gyroscope data
        self.gyr_data['x'].append(msg.angular_velocity.x)
        self.gyr_data['y'].append(msg.angular_velocity.y)
        self.gyr_data['z'].append(msg.angular_velocity.z)

        self.sample_count += 1

        if self.sample_count >= self.max_samples:
            self.calculate_noise()

    def calculate_noise(self):
        # Convert data to NumPy arrays
        acc_x = np.array(self.acc_data['x'])
        acc_y = np.array(self.acc_data['y'])
        acc_z = np.array(self.acc_data['z'])
        gyr_x = np.array(self.gyr_data['x'])
        gyr_y = np.array(self.gyr_data['y'])
        gyr_z = np.array(self.gyr_data['z'])

        # Calculate standard deviations for accelerometer and gyroscope
        imu_acc_noise = np.mean([np.std(acc_x), np.std(acc_y), np.std(acc_z)])
        imu_gyr_noise = np.mean([np.std(gyr_x), np.std(gyr_y), np.std(gyr_z)])

        # Bias noise estimation (simplified based on total samples)
        imu_acc_bias_n = imu_acc_noise / self.max_samples
        imu_gyr_bias_n = imu_gyr_noise / self.max_samples

        # Calculate gravity
        acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        imu_gravity = np.mean(acc_magnitude)

        # Print calculated values
        self.get_logger().info(f"Accelerometer Noise (imuAccNoise): {imu_acc_noise:.6e} m/s^2")
        self.get_logger().info(f"Gyroscope Noise (imuGyrNoise): {imu_gyr_noise:.6e} rad/s")
        self.get_logger().info(f"Accelerometer Bias Noise (imuAccBiasN): {imu_acc_bias_n:.6e} m/s^2/s")
        self.get_logger().info(f"Gyroscope Bias Noise (imuGyrBiasN): {imu_gyr_bias_n:.6e} rad/s/s")
        self.get_logger().info(f"Gravity (imuGravity): {imu_gravity:.6f} m/s^2")

        # Stop the node after calculation
        self.get_logger().info("IMU Noise Calculation Complete. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNoiseCalculatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
