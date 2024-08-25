import os

import numpy as np
import open3d as o3d

LIDAR_INITIAL_FLOOR_HEIGHT = 1.0  # [m]

if __name__ == "__main__":
    box1 = o3d.geometry.TriangleMesh.create_box(depth=5)
    box2 = o3d.geometry.TriangleMesh.create_box(depth=5)
    box3 = o3d.geometry.TriangleMesh.create_box(depth=5)
    plane = o3d.geometry.TriangleMesh.create_box(width=6, height=6, depth=0.0001)
    rotated_plane = o3d.geometry.TriangleMesh.create_box(
        width=6, height=3, depth=0.0001
    )

    box1.translate((-2, 0, -LIDAR_INITIAL_FLOOR_HEIGHT))
    box2.translate((1, 0, -LIDAR_INITIAL_FLOOR_HEIGHT))
    box3.translate((-0.5, 1, -LIDAR_INITIAL_FLOOR_HEIGHT))
    plane.translate((-3, -1, -LIDAR_INITIAL_FLOOR_HEIGHT))
    rotated_plane.translate((-3, -4, -LIDAR_INITIAL_FLOOR_HEIGHT))

    rotation_matrix = plane.get_rotation_matrix_from_xyz((-20 * np.pi / 180, 0, 0))
    rotated_plane.rotate(rotation_matrix, center=(0, -1, -LIDAR_INITIAL_FLOOR_HEIGHT))

    pcd_box1 = box1.sample_points_uniformly(number_of_points=4000)
    pcd_box2 = box2.sample_points_uniformly(number_of_points=4000)
    pcd_box3 = box3.sample_points_uniformly(number_of_points=4000)
    plane_pcd = plane.sample_points_uniformly(number_of_points=10000)
    rotated_plane_pcd = rotated_plane.sample_points_uniformly(number_of_points=10000)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0]
    )

    combined_pcd = pcd_box1 + pcd_box2 + pcd_box3 + plane_pcd + rotated_plane_pcd

    current_file_dir = os.path.dirname(os.path.realpath(__file__))

    static_map_pcd_path = os.path.join(
        current_file_dir,
        os.pardir,
        "saved_maps",
        "fake_static_map.pcd",
    )

    # Save the point cloud
    o3d.io.write_point_cloud(static_map_pcd_path, combined_pcd)

    o3d.visualization.draw_geometries([combined_pcd, mesh_frame])
