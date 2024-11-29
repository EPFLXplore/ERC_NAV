import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyRemapperNode(Node):
    def __init__(self):
        super().__init__('joy_remapper_node')

        # Subscription to the /backup_gamepad_joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/backup_gamepad_joy',
            self.joy_callback,
            10
        )

        # Publisher to the /CS/NAV_gamepad topic
        self.publisher = self.create_publisher(
            Joy,
            '/CS/NAV_gamepad',
            10
        )

        self.get_logger().info('JoyRemapperNode initialized.')

    def remap_axes(self, axes):
        axes_cpy = axes
        #print(f"axes 2: {axes[2] }")
        #print(f"axes 5: {axes[5] }")

        # Transform axes[2] and axes[5] from [-1, 1] to [0, 1]
        axes_cpy[2] = 1 -( (axes[2]+1)/2.0 )
        axes_cpy[5] = 1- ( (axes[5]+1)/2.0 )

        # Flip sign of x axis of left thumbstick
        axes_cpy[0] = (-1.0)*axes[0]

        return axes_cpy

    def joy_callback(self, msg):
        remapped_msg = Joy()
        remapped_msg.header = msg.header  # Preserve the original header
        remapped_msg.axes = self.remap_axes(msg.axes[:])
        remapped_msg.buttons = msg.buttons

        self.publisher.publish(remapped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyRemapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('JoyRemapperNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
