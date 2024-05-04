#!/usr/bin/env python3

import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from ament_index_python import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

METERS_PER_PIXEL = 0.2
MAX_SLOPE_ANGLE = 40  # [deg]


class StaticCostMapPublisher(Node):

    def __init__(self):
        super().__init__("static_cost_map_publisher")

        xplore_description_share_directory = get_package_share_directory(
            "xplore_description"
        )

        static_map_path = os.path.join(
            xplore_description_share_directory,
            "models",
            "marsyard2022_terrain",
            "dem",
            "marsyard2022_terrain_hm.tif",
        )
        self.static_map = cv.imread(static_map_path, cv.IMREAD_UNCHANGED)

        self.static_cost_map_publisher = self.create_publisher(
            OccupancyGrid,
            "static_cost_map",
            10,
        )

        self.static_cost_map = self.generate_static_cost_map()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_static_cost_map)

        self.get_logger().info("static_cost_map_publisher is ready.")

    def generate_static_cost_map(self):
        # Compute the height gradient
        static_map_dx = (
            np.concatenate((self.static_map[:, 1:], self.static_map[:, -1:]), axis=1)
            - self.static_map
        )
        static_map_dx /= METERS_PER_PIXEL
        static_map_dy = (
            np.concatenate((self.static_map[1:, :], self.static_map[-1:, :]), axis=0)
            - self.static_map
        )
        static_map_dy /= METERS_PER_PIXEL

        height_gradient_norm = np.sqrt(static_map_dx**2 + static_map_dy**2)

        # Compute the slope angle
        slope_angle = np.arctan(height_gradient_norm) * 180 / np.pi

        # Compute the static cost map with values between 0 and 100
        static_cost_map = np.zeros_like(slope_angle)
        static_cost_map[slope_angle >= MAX_SLOPE_ANGLE] = 100
        static_cost_map[slope_angle < MAX_SLOPE_ANGLE] = (
            100 * slope_angle[slope_angle < MAX_SLOPE_ANGLE] / MAX_SLOPE_ANGLE
        )
        static_cost_map = static_cost_map.astype(np.int8)

        static_cost_map_msg = OccupancyGrid()

        # Populate the OccupancyGrid message Header
        static_cost_map_msg.header.stamp = self.get_clock().now().to_msg()
        static_cost_map_msg.header.frame_id = "map"

        # Populate the OccupancyGrid message MapMetaData
        static_cost_map_msg.info.width = static_cost_map.shape[1]
        static_cost_map_msg.info.height = static_cost_map.shape[0]
        static_cost_map_msg.info.resolution = METERS_PER_PIXEL
        # Real world pose of the map cell (0, 0) in the map frame
        static_cost_map_msg.info.origin.position.x = -25.0
        static_cost_map_msg.info.origin.position.y = -25.0
        static_cost_map_msg.info.origin.position.z = 0.0
        static_cost_map_msg.info.origin.orientation.x = 0.0
        static_cost_map_msg.info.origin.orientation.y = 0.0
        static_cost_map_msg.info.origin.orientation.z = 0.0
        static_cost_map_msg.info.origin.orientation.w = 1.0

        # Populate the OccupancyGrid message data flattened in row-major order
        static_cost_map_msg.data = static_cost_map.flatten().tolist()

        return static_cost_map_msg

    def publish_static_cost_map(self):
        self.static_cost_map.header.stamp = self.get_clock().now().to_msg()
        self.static_cost_map_publisher.publish(self.static_cost_map)


def main(args=None):
    rclpy.init(args=args)

    static_map_publisher = StaticCostMapPublisher()

    rclpy.spin(static_map_publisher)

    static_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
