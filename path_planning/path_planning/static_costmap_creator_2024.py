#!/usr/bin/env python3

import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

MAX_SLOPE_ANGLE = 40  # [deg]
METERS_PER_PIXEL = 0.013  # [m/pixel]


class StaticCostMapCreator:

    def __init__(self):
        # Get current file directory
        self.current_file_dir = os.path.dirname(os.path.realpath(__file__))
        # Get the xplore_description package directory
        xplore_description_dir = os.path.join(
            os.path.dirname(os.path.dirname(self.current_file_dir)),
            "xplore_description",
        )

        static_map_path = os.path.join(
            self.current_file_dir,
            os.pardir,
            "saved_maps",
            "static_map_2024.tif",
        )

        self.static_map = cv.imread(static_map_path, cv.IMREAD_UNCHANGED)

        height_factor = 0.01
        self.static_map = self.static_map * height_factor

    def plot_all_maps(self):
        H, W = self.static_map.shape

        # Compute the X, Y, Z coordinates
        X = np.arange(0, W, 1) * METERS_PER_PIXEL
        Y = np.arange(0, H, 1) * METERS_PER_PIXEL
        X, Y = np.meshgrid(X, Y)
        Z = self.static_map

        # Plot the heightmap
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.plot_surface(X, Y, Z, cmap="hot")
        ax.set_title("Heightmap")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Height [m]")

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
        height_gradient_angle = np.arctan2(static_map_dy, static_map_dx) * 180 / np.pi

        # Plot the height gradient norm
        fig, ax = plt.subplots()
        ax.pcolormesh(
            X,
            Y,
            height_gradient_norm,
            vmin=height_gradient_norm.min(),
            vmax=height_gradient_norm.max(),
            cmap="hot",
        )
        ax.set_title("Height Gradient Norm [m/m]")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        # Plot the height gradient angle
        fig, ax = plt.subplots()
        ax.pcolormesh(
            X,
            Y,
            height_gradient_angle,
            vmin=height_gradient_angle.min(),
            vmax=height_gradient_angle.max(),
            cmap="hot",
        )
        ax.set_title("Height Gradient Angle [deg]")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        # Plot the slope direction arrow
        fig, ax = plt.subplots()
        ax.quiver(X, Y, static_map_dx, static_map_dy, scale_units="xy", scale=1)
        ax.set_title("Slope Direction Arrow")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        # Plot the slope angle
        slope_angle = np.arctan(height_gradient_norm) * 180 / np.pi
        fig, ax = plt.subplots()
        ax.pcolormesh(
            X,
            Y,
            slope_angle,
            vmin=slope_angle.min(),
            vmax=slope_angle.max(),
            cmap="hot",
        )
        ax.set_title("Slope Angle [deg]")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        plt.show()

    def generate_static_costmap(self):
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

        # Compute the static costmap with values between 0 (obstacle) and 255 (free space)
        static_costmap = np.zeros_like(slope_angle)
        static_costmap[slope_angle >= MAX_SLOPE_ANGLE] = 0
        static_costmap[slope_angle < MAX_SLOPE_ANGLE] = (
            255 * (MAX_SLOPE_ANGLE - slope_angle[slope_angle < MAX_SLOPE_ANGLE]) / MAX_SLOPE_ANGLE
        )
        static_costmap = static_costmap.astype(np.uint8)

        H, W = self.static_map.shape
        X = np.arange(0, W, 1) * METERS_PER_PIXEL
        Y = np.arange(0, H, 1) * METERS_PER_PIXEL
        X, Y = np.meshgrid(X, Y)
        fig, ax = plt.subplots()
        ax.pcolormesh(
            X,
            Y,
            static_costmap,
            vmin=static_costmap.min(),
            vmax=static_costmap.max(),
            cmap="hot",
        )
        ax.set_title("Static costmap")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        # Get the static_costmap directory
        static_costmap_dir = os.path.join(
            os.path.dirname(self.current_file_dir),
            "config",
        )

        static_costmap_path = os.path.join(static_costmap_dir, "static_costmap.png")
        cv.imwrite(static_costmap_path, static_costmap)
        plt.show()


if __name__ == "__main__":
    static_costmap_creator = StaticCostMapCreator()
    # static_costmap_creator.plot_all_maps()
    static_costmap_creator.generate_static_costmap()
