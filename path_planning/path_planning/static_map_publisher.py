#!/usr/bin/env python3

import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python import get_package_share_directory

METERS_PER_PIXEL = 0.2

if __name__ == "__main__":
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
    static_map = cv.imread(static_map_path, cv.IMREAD_UNCHANGED)
    H, W = static_map.shape

    # Compute the X, Y, Z coordinates
    X = np.arange(0, W, 1) * METERS_PER_PIXEL
    Y = np.arange(0, H, 1) * METERS_PER_PIXEL
    X, Y = np.meshgrid(X, Y)
    Z = static_map

    # Plot the heightmap
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, Z, cmap="hot")
    ax.set_title("Heightmap")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Height [m]")

    # Compute the height gradient
    static_map_dx = (
        np.concatenate((static_map[:, 1:], static_map[:, -1:]), axis=1) - static_map
    )
    static_map_dx /= METERS_PER_PIXEL
    static_map_dy = (
        np.concatenate((static_map[1:, :], static_map[-1:, :]), axis=0) - static_map
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
