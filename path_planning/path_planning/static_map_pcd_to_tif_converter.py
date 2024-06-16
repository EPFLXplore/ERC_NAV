#!/usr/bin/env python3

import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

METERS_PER_PIXEL = 0.20  # [m/pixel]
# Lidar height from the ground with respect to the initial floor height.
LIDAR_INITIAL_FLOOR_HEIGHT = 1.0  # [m]
# Maximum and minimum height values for the heightmap with respect to the initial floor height.
Z_MAX = 3.0  # [m]
Z_MIN = 0.0  # [m]


class StaticMapPcdToTifConverter:

    def __init__(self):
        # Get current file directory
        self.current_file_dir = os.path.dirname(os.path.realpath(__file__))

        static_map_pcd_path = os.path.join(
            self.current_file_dir,
            os.pardir,
            "saved_maps",
            "static_map.pcd",
        )
        self.static_map_pcd = o3d.io.read_point_cloud(static_map_pcd_path)

    def convert_static_map(self):
        # Remove duplicated and non-finite points.
        static_map_pcd = (
            self.static_map_pcd.remove_duplicated_points().remove_non_finite_points()
        )
        # Downsample the point cloud to the desired voxel resolution in METERS_PER_PIXEL by averaging all points in the same voxel.
        static_map_pcd = static_map_pcd.voxel_down_sample(METERS_PER_PIXEL)
        # Get the min and max bounds of the point cloud
        min_x, min_y = static_map_pcd.get_min_bound()[:2]
        max_x, max_y = static_map_pcd.get_max_bound()[:2]

        # Convert the point cloud to a numpy array
        static_map_np = np.asarray(static_map_pcd.points)
        # Create a 2D grid with the desired resolution in METERS_PER_PIXEL and compute the average height of the points in each cell.
        X_val = np.arange(min_x, max_x + METERS_PER_PIXEL, METERS_PER_PIXEL)
        Y_val = np.arange(min_y, max_y + METERS_PER_PIXEL, METERS_PER_PIXEL)
        X, Y = np.meshgrid(X_val, Y_val)
        Z = np.zeros_like(X)
        for i in range(len(X_val)):
            for j in range(len(Y_val)):
                idx = np.where(
                    (static_map_np[:, 0] >= X_val[i] - METERS_PER_PIXEL / 2)
                    & (static_map_np[:, 0] < X_val[i] + METERS_PER_PIXEL / 2)
                    & (static_map_np[:, 1] >= Y_val[j] - METERS_PER_PIXEL / 2)
                    & (static_map_np[:, 1] < Y_val[j] + METERS_PER_PIXEL / 2)
                )[0]
                if len(idx) > 0:
                    height = np.mean(static_map_np[idx, 2]) + LIDAR_INITIAL_FLOOR_HEIGHT
                    if height <= Z_MAX and height >= Z_MIN:
                        Z[j, i] = height

        # Save the heightmap as a .tif file
        static_map_tif_path = os.path.join(
            self.current_file_dir,
            os.pardir,
            "saved_maps",
            "static_map.tif",
        )
        cv.imwrite(static_map_tif_path, Z)

        # Plot the heightmap
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.plot_surface(X, Y, Z, cmap="hot")
        ax.set_title("Heightmap")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Height [m]")
        plt.show()

        o3d.visualization.draw_geometries([self.static_map_pcd])


if __name__ == "__main__":
    static_map_converter = StaticMapPcdToTifConverter()
    static_map_converter.convert_static_map()
