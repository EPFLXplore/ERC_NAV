#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.point_cloud_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def create_heightmap(self, points):
        # Assume points is a Nx3 numpy array
        # Convert points to a heightmap here (simplified example)
        H, W = 100, 100  # Define dimensions of the heightmap
        heightmap = np.zeros((H, W), dtype=np.float32)

        # Fill heightmap based on x, y, z values
        # This is highly simplified and might need actual transformation logic
        for x, y, z in points:
            ix, iy = int(x), int(y)
            if 0 <= ix < W and 0 <= iy < H:
                heightmap[iy, ix] = z

        return heightmap
    
    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        print(points_array.shape)
        
        if points_array.size == 0:
            return

        # Example: Create a heightmap from the point cloud
        # This is a placeholder for actual heightmap generation code
        heightmap = self.create_heightmap(points_array)
        print(heightmap.shape)
        # Display or process the heightmap
        self.display_heightmap(heightmap)

        # Compute the X, Y, Z coordinates
        X = np.arange(0, W, 1) * METERS_PER_PIXEL
        Y = np.arange(0, H, 1) * METERS_PER_PIXEL
        X, Y = np.meshgrid(X, Y)
        Z = static_map


    def display_heightmap(self, heightmap):

        # Plot the heightmap
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.plot_surface(X, Y, Z, cmap="hot")
        ax.set_title("Heightmap")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Height [m]")


def main(args=None):
    rclpy.init(args=args)
    print("hmnm")
    point_cloud_processor = PointCloudProcessor()
    rclpy.spin(point_cloud_processor)

    point_cloud_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
