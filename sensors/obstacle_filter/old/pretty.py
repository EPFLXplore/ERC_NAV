import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib

from sklearn.neighbors import BallTree

from ransacObstacles import convertToPointCloud, ransac_3d_cuda

import open3d as o3d


def displayPC(depth_array, image, scatter, scatter_all):
    skip_points = 10
    array = depth_array[::skip_points, ::skip_points]
    array = array.astype(np.float32)

    img_array = image[::skip_points, ::skip_points]

    # filter points too far or too close
    max_depth = 3000
    min_depth = 100

    mask = (array < max_depth) * (array > min_depth)

    array = array * mask
    # print(array.shape)
    # print(img_array.shape)

    f_x = 798.31
    # f_y = f_x

    c_x = 655.73
    c_y = 313.12

    f_x = f_x / (skip_points)
    c_x = c_x / (skip_points)
    c_y = c_y / (skip_points)

    mask, x, y, z = convertToPointCloud(array, f_x, c_x, c_y)
    # print(mask.shape)
    # print(img_array.reshap.shape)
    # img_array = img_array.reshape((-1, 3))[mask, :]

    img_array = img_array.ravel()[mask]

    points = np.column_stack((x, y, z))

    num_iterations = 50
    threshold_distance = 100
    num_inliers = 1000

    # best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu', num_random_pts=3)
    best_plane = None
    if best_plane is None:
        print('No plane found')
    else:
        y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

        # filter points that are too far away from the plane
        threshold = 100
        points = points[np.abs(y_pred - y) > threshold]
        img_array = img_array[np.abs(y_pred - y) > threshold]

        # remove outliers
        numberpts = 15
        radius = 200
        tree = BallTree(points, leaf_size=15, metric='euclidean')

        dist, ind = tree.query(points, k=numberpts) # query the tree for the 20 nearest neighbors for each point

        # if one of the 20 nearest neighbors is more than 200mm away, remove the point
        delidx = np.ones(points.shape[0], dtype=bool)
        for i in range(points.shape[0]):
            if np.any(dist[i] > radius):
                delidx[i] = False

        points = points[delidx]
        img_array = img_array[delidx]




    # print(img_array.shape)
    # print(points.shape)

    # scatter_all.set_data(points[:, 0], points[:, 2])
    # scatter_all.set_3d_properties(points[:, 1])
    # scatter_all.set_color(img_array)
    scatter_all = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=img_array, cmap='gray', s=0.5)

    # open3d

    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points)
    print(img_array.shape)
    img_array = img_array.reshape((-1, 1))
    img_array = np.repeat(img_array, 3, axis=1)
    print(img_array)
    pointcloud.colors = o3d.utility.Vector3dVector(img_array.astype(float) / 255.0)

    visualization_window = o3d.visualization.Visualizer()
    visualization_window.create_window()
    visualization_window.add_geometry(pointcloud)

    render_options = visualization_window.get_render_option()
    render_options.background_color = np.asarray([0, 0, 0])  # Set background color to black
    #pointcloud.paint_uniform_color(np.asarray([1, 0, 0]))  # Set point cloud color to red
    #pointcloud.point_size = 1.0  # Set point size to 1.0

    view_control = visualization_window.get_view_control()
    camera_position = [1.0, -1.0, 1.0]
    view_control.set_lookat(np.asarray(camera_position))

    # Update the visualization window after modifying the settings
    visualization_window.update_renderer()

    # Display the point cloud
    visualization_window.run()

    return scatter_all

    # num_iterations = 20
    # threshold_distance = 25
    # num_inliers = 1000

    # if points.shape[0] == 0:
    #     print('No points found')
    #     return

    # best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu', num_random_pts=4)
    # if best_plane is None:
    #     print('No plane found')
    # else:
    #     y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

    #     # filter points that are too far away from the plane
    #     threshold = 100
    #     points = points[np.abs(y_pred - y) > threshold]

    #     # remove outliers
    #     numberpts = 10
    #     radius = 200
    #     tree = BallTree(points, leaf_size=10, metric='euclidean')

    #     dist, ind = tree.query(points, k=numberpts) # query the tree for the 20 nearest neighbors for each point

    #     # if one of the 20 nearest neighbors is more than 200mm away, remove the point
    #     delidx = np.ones(points.shape[0], dtype=bool)
    #     for i in range(points.shape[0]):
    #         if np.any(dist[i] > radius):
    #             delidx[i] = False

    #     points = points[delidx]
    #     scatter.set_data(points[:, 0], points[:, 2])
    #     scatter.set_3d_properties(points[:, 1])

fig = plt.figure(0, figsize=(5, 5))
ax = fig.add_subplot(211, projection='3d')
# ax.set_size_inches(10, 10, 10)

# ax.scatter(x, y, z, c='r', s=0.5)
scatter, = ax.plot(0, 0, 0, color='b', marker='.', linestyle='None', markersize=2)

scatter_all = ax.scatter(0, 0, 0, marker='.', linestyle='None', cmap='gray')
# ax.plot_surface(x, y, z, alpha=0.5)
# ax.set_xlim(-2000, 2000)
# ax.set_ylim(-2000, 2000)
ax.set_zlim(0, 3000)

# Set the axis labels and show the plot
ax.set_xlabel('X')
ax.set_zlabel('Z')
ax.set_ylabel('Y')

ax.view_init(elev=-90., azim=-90)

folder = 'plage_tranchee_tilt_less'


# ax2 = fig.add_subplot(212)
# img=ax2.imshow(np.load('images/' + folder + '/rgb/rgb_data_' + str(0) + '.npy'))



def update(i):
    global scatter_all
    scatter_all.remove()
    #matplotlib.axes.Axes.draw_idle(ax)
    print(i)
    depth_array = np.load('images/' + folder + '/depth/depth_data_' + str(i) + '.npy')
    img_array = np.load('images/' + folder + '/rgb/rgb_data_' + str(i) + '.npy')
    # imu_array = np.load('images/' + folder + '/imu/imu_data_' + str(i) + '.npy')
    # print(imu_array)
    #displayPC2(depth_array, 0, scatter)
    # img.set_data(img_array)
    scatter_all = displayPC(depth_array, img_array, scatter, scatter_all)

update(120)
# ani = FuncAnimation(fig, update, interval=100)
# ani.save('cube.mp4', writer='ffmpeg', fps=10)
plt.show()