#!/usr/bin/env python3

import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

METERS_PER_PIXEL = 0.2
MAX_SLOPE_ANGLE = 40  # [deg]


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
            xplore_description_dir,
            "models",
            "marsyard2022_terrain",
            "dem",
            "marsyard2022_terrain_hm.tif",
        )
        self.static_map = cv.imread(static_map_path, cv.IMREAD_UNCHANGED)

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

        # Compute the static cost map with values between 0 (obstacle) and 255 (free space)
        static_cost_map = np.zeros_like(slope_angle)
        static_cost_map[slope_angle >= MAX_SLOPE_ANGLE] = 0
        static_cost_map[slope_angle < MAX_SLOPE_ANGLE] = (
            255 * (MAX_SLOPE_ANGLE - slope_angle[slope_angle < MAX_SLOPE_ANGLE]) / MAX_SLOPE_ANGLE
        )
        static_cost_map = static_cost_map.astype(np.uint8)

        H, W = self.static_map.shape
        X = np.arange(0, W, 1) * METERS_PER_PIXEL
        Y = np.arange(0, H, 1) * METERS_PER_PIXEL
        X, Y = np.meshgrid(X, Y)
        fig, ax = plt.subplots()
        ax.pcolormesh(
            X,
            Y,
            static_cost_map,
            vmin=static_cost_map.min(),
            vmax=static_cost_map.max(),
            cmap="hot",
        )
        ax.set_title("Static cost map")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        # Get the static_cost_map directory
        static_cost_map_dir = os.path.join(
            os.path.dirname(self.current_file_dir),
            "config",
        )

        static_cost_map_path = os.path.join(static_cost_map_dir, "static_cost_map.png")
        cv.imwrite(static_cost_map_path, static_cost_map)
        plt.show()


if __name__ == "__main__":
    static_cost_map_creator = StaticCostMapCreator()
    # static_cost_map_creator.plot_all_maps()
    static_cost_map_creator.generate_static_cost_map()
