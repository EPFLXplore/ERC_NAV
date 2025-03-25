import open3d as o3d
import numpy as np

NB_POINTS = 100000  # Number of points to sample from the mesh

def point_cloud_creator(obj_file, pcd_file):
    global NB_POINTS
    """
    Converts an OBJ file to a PCD file by sampling points from the mesh surface,
    visualizes both the original mesh (with shading) and the resulting point cloud.

    :param obj_file: Path to the input .obj file
    :param pcd_file: Path to the output .pcd file
    """
    # Load the mesh from the OBJ file
    mesh = o3d.io.read_triangle_mesh(obj_file)
    
    if not mesh.has_triangles():
        raise ValueError("Error: The OBJ file does not contain a valid mesh.")

    # Compute vertex normals for better shading
    mesh.compute_vertex_normals()

    # Apply grayscale color based on normals
    normals = np.asarray(mesh.vertex_normals)
    grayscale = (normals - normals.min()) / (normals.max() - normals.min())  # Normalize to [0, 1]
    mesh.vertex_colors = o3d.utility.Vector3dVector(grayscale)

    # Visualize the original mesh with shading
    print(f"Visualizing original mesh: {obj_file}")
    print("Close the window to display the point cloud...")
    o3d.visualization.draw_geometries([mesh], window_name="Original Mesh")
    
    # Sample points from the mesh surface
    point_cloud = mesh.sample_points_uniformly(number_of_points=NB_POINTS)

    # Save the point cloud as a .pcd file
    o3d.io.write_point_cloud(pcd_file, point_cloud)
    print(f"Converted {obj_file} to {pcd_file}")

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud], window_name="3D Point Cloud")

def main():
    obj_path = "mars_yard_2024.obj" # input file
    pcd_path = "mars_yard_2024.pcd" # output file

    point_cloud_creator(obj_path, pcd_path)

if __name__ == "__main__":
    main()