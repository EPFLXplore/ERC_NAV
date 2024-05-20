#!/usr/bin/python3
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from custom_msg.msg import Motorcmds


class RosControlCmdBridge(Node):
    def __init__(self):
        super().__init__("ros_control_cmd_bridge")
        wait_sec_between_publish = 0.5

        # Bridge between the wheels velocity controller and the wheels velocity commands
        self.wheels_velocity_goals = [0.0, 0.0, 0.0, 0.0]
        # Subscribe to the topic that provides the velocities
        wheels_velocity_topic = "/NAV/displacement"
        self.create_subscription(
            Motorcmds, wheels_velocity_topic, self.wheels_velocity_callback, 1
        )

        velocity_controller_name = "velocity_controller"
        velocity_controller_topic = "/" + velocity_controller_name + "/" + "commands"
        self.wheels_velocity_publisher = self.create_publisher(
            Float64MultiArray, velocity_controller_topic, 1
        )

        # Bridge the steering position controller and the steering position commands
        self.steering_position_goals = [0.0, 0.0, 0.0, 0.0]
        # Subscribe to the topic that provides the steering positions
        steering_position_topic = "/NAV/displacement"
        self.create_subscription(
            Motorcmds, steering_position_topic, self.steering_position_callback, 1
        )

        position_controller_name = "position_controller"
        position_controller_topic = "/" + position_controller_name + "/" + "commands"
        self.steering_position_publisher = self.create_publisher(
            Float64MultiArray, position_controller_topic, 1
        )

        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    def wheels_velocity_callback(self, msg):
        # wheel order: FL,FR,RR,RL
        self.wheels_velocity_goals = [
            msg.drive[0] / 200,
            msg.drive[1] / 200,
            msg.drive[2] / 200,
            msg.drive[3] / 200,
        ]

    def steering_position_callback(self, msg):
        # steering order: FL,FR,RR,RL
        self.steering_position_goals = [
            msg.steer[0] * 2 * np.pi / 65536,
            -msg.steer[1] * 2 * np.pi / 65536,
            msg.steer[2] * 2 * np.pi / 65536,
            -msg.steer[3] * 2 * np.pi / 65536,
        ]

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.wheels_velocity_goals
        self.wheels_velocity_publisher.publish(msg)

        msg = Float64MultiArray()
        msg.data = self.steering_position_goals
        self.steering_position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ros_control_cmd_bridge = RosControlCmdBridge()
    rclpy.spin(ros_control_cmd_bridge)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
