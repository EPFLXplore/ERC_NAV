import os
import cv2 as cv
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

METERS_PER_PIXEL = 0.4              # Resolution of the heatmap [m/pixel]
LIDAR_INITIAL_FLOOR_HEIGHT = 1.0    # Lidar height from the ground with respect to the initial floor height [m]
Z_MAX = 3.0                         # Maximum height values for the heightmap with respect to the initial floor height [m]


def heightmap_creator(pcd_file, tif_path):
    global METERS_PER_PIXEL, LIDAR_INITIAL_FLOOR_HEIGHT, Z_MAX
    """
    Generates a heightmap from a point cloud and saves it as a .tif file.
    :param pcd: Open3D point cloud object
    :param tif_path: Path to save the output .tif file
    """
    # Remove duplicated and non-finite points
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd = pcd.remove_duplicated_points().remove_non_finite_points()

    # Visualize the original point cloud
    print("Close the window to display the heightmap...")
    o3d.visualization.draw_geometries([pcd], window_name="3D Point Cloud")

    # Downsample the point cloud to reduce computation time
    pcd_down = pcd.voxel_down_sample(METERS_PER_PIXEL)
    pcd_np = np.asarray(pcd_down.points)

    min_x, min_y = pcd_down.get_min_bound()[:2]
    max_x, max_y = pcd_down.get_max_bound()[:2]

    X_val = np.arange(min_x, max_x + METERS_PER_PIXEL, METERS_PER_PIXEL)
    Y_val = np.arange(min_y, max_y + METERS_PER_PIXEL, METERS_PER_PIXEL)
    X, Y = np.meshgrid(X_val, Y_val)
    Z = np.zeros_like(X)

    for i in range(len(X_val)):
        for j in range(len(Y_val)):
            idx = np.where(
                (pcd_np[:, 0] >= X_val[i] - METERS_PER_PIXEL / 2)
                & (pcd_np[:, 0] < X_val[i] + METERS_PER_PIXEL / 2)
                & (pcd_np[:, 1] >= Y_val[j] - METERS_PER_PIXEL / 2)
                & (pcd_np[:, 1] < Y_val[j] + METERS_PER_PIXEL / 2)
            )[0]
            if len(idx) > 0:
                heights = pcd_np[idx, 2] + LIDAR_INITIAL_FLOOR_HEIGHT
                heights = heights[heights <= Z_MAX]
                if len(heights) > 0:
                    Z[j, i] = np.mean(heights)

    print("Heightmap computed!")

    # Save the heightmap as a .tif file
    cv.imwrite(tif_path, Z)
    print(f"Saved heightmap to {tif_path}")

    # Visualizes the generated heightmap using Matplotlib
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, Z, cmap="hot")
    ax.set_title("Heightmap")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Height [m]")
    plt.show(block=True)


def main():
    pcd_path = "mars_yard_2024.pcd" # input file
    tif_path = "mars_yard_2024.tif" # output file

    heightmap_creator(pcd_path, tif_path)


if __name__ == "__main__":
    main()