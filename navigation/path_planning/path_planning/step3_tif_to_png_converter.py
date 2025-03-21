import os
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

MAX_SLOPE_ANGLE = 35        # Slope threshold from which maximum cost is assigned [deg]
METERS_PER_PIXEL = 0.2      # Resolution of the heatmap [m/pixel]


def costmap_creator(tif_path, png_path):
    global MAX_SLOPE_ANGLE, METERS_PER_PIXEL
    """
    Generates and saves a static costmap based on slope angles.
    :param tif_path: Path to the input .tif file
    :param png_path: Path to save the output .png file
    """
    static_map = cv.imread(tif_path, cv.IMREAD_UNCHANGED).astype(np.float64)
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

    # Compute the gradient along the X-axis (dx) Y-axis (dy)
    static_map_dx = (np.concatenate((static_map[:, 1:], static_map[:, -1:]), axis=1) - static_map) / METERS_PER_PIXEL
    static_map_dy = (np.concatenate((static_map[1:, :], static_map[-1:, :]), axis=0) - static_map) / METERS_PER_PIXEL

    height_gradient_norm = np.sqrt(static_map_dx**2 + static_map_dy**2)

    # Compute the slope angle
    slope_angle = np.arctan(height_gradient_norm) * 180 / np.pi

    # Compute the static costmap with values between 0 (obstacle) and 255 (free space)
    static_costmap = np.zeros_like(slope_angle)
    static_costmap[slope_angle >= MAX_SLOPE_ANGLE] = 0
    static_costmap[slope_angle < MAX_SLOPE_ANGLE] = (255 * (MAX_SLOPE_ANGLE - slope_angle[slope_angle < MAX_SLOPE_ANGLE]) / MAX_SLOPE_ANGLE)
    #static_costmap[slope_angle < (MAX_SLOPE_ANGLE)] = 255
    static_costmap = static_costmap.astype(np.uint8)

    # Save costmap
    cv.imwrite(png_path, static_costmap)

    # Display costmap
    plt.figure(figsize=(6, 6))
    plt.imshow(static_costmap, cmap="gray", vmin=0, vmax=255)
    plt.colorbar(label="Cost value (0 = obstacle, 255 = free space)")
    plt.title("Static Costmap")
    plt.axis("off")
    plt.show()


def main():
    tif_path = "mars_yard_2024.tif"  # Input file
    png_path = "mars_yard_2024.png"  # Output file

    costmap_creator(tif_path, png_path)


if __name__ == "__main__":
    main()
