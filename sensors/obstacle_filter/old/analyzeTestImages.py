import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

from sklearn.neighbors import BallTree

from ransacObstacles import convertToPointCloud, ransac_3d_cuda


def displayPC(depth_array, scatter, scatter_all):
    skip_points = 10
    array = depth_array[::skip_points, ::skip_points]
    array = array.astype(np.float32)

    # filter points too far or too close
    max_depth = 3000
    min_depth = 100

    array = array * (array < max_depth) * (array > min_depth)

    f_x = 798.31
    # f_y = f_x

    c_x = 655.73
    c_y = 313.12

    f_x = f_x / (skip_points)
    c_x = c_x / (skip_points)
    c_y = c_y / (skip_points)

    x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

    points = np.column_stack((x, y, z))

    scatter_all.set_data(points[:, 0], points[:, 2])
    scatter_all.set_3d_properties(points[:, 1])

    num_iterations = 20
    threshold_distance = 25
    num_inliers = 1000

    if points.shape[0] == 0:
        print('No points found')
        return

    best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu', num_random_pts=4)
    if best_plane is None:
        print('No plane found')
    else:
        y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

        # filter points that are too far away from the plane
        threshold = 100
        points = points[np.abs(y_pred - y) > threshold]

        # remove outliers
        numberpts = 10
        radius = 200
        tree = BallTree(points, leaf_size=10, metric='euclidean')

        dist, ind = tree.query(points, k=numberpts) # query the tree for the 20 nearest neighbors for each point

        # if one of the 20 nearest neighbors is more than 200mm away, remove the point
        delidx = np.ones(points.shape[0], dtype=bool)
        for i in range(points.shape[0]):
            if np.any(dist[i] > radius):
                delidx[i] = False

        points = points[delidx]
        scatter.set_data(points[:, 0], points[:, 2])
        scatter.set_3d_properties(points[:, 1])


        # fig = plt.figure(0)
        # ax = fig.add_subplot(111, projection='3d')
        # # ax.scatter(x, y, z, c='r', s=0.5)
        # ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', s=0.5)
        # # ax.plot_surface(x, y, z, alpha=0.5)

        # # Set the axis labels and show the plot
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # plt.show(block=False)


# this one we show the entire point cloud with reference frame according to gravity
def displayPC2(depth_array, imu_array, scatter):
    skip_points = 20
    array = depth_array[::skip_points, ::skip_points]
    array = array.astype(np.float32)

    # filter points too far or too close
    max_depth = 15000
    min_depth = 100

    array = array * (array < max_depth) * (array > min_depth)

    f_x = 798.31
    # f_y = f_x

    c_x = 655.73
    c_y = 313.12

    f_x = f_x / (skip_points)
    c_x = c_x / (skip_points)
    c_y = c_y / (skip_points)

    x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

    points = np.column_stack((x, y, z))

    # convert points to gravity frame
    # imu axes are different than image coordinates
    # gravity = np.array([imu_array[1], -imu_array[0], imu_array[2]])
    # #gravity = imu_array[0:3]
    # gravity = gravity / np.linalg.norm(gravity)
    # # print(gravity)

    # # We construct the new basis

    # new_z_axis = -gravity / np.linalg.norm(gravity)
    # new_y_axis = np.cross(np.array([1, 0, 0]), gravity)
    # new_y_axis = new_y_axis / np.linalg.norm(new_y_axis)
    # new_x_axis = np.cross(new_y_axis, new_z_axis)
    # new_x_axis = new_x_axis / np.linalg.norm(new_x_axis)

    # change_matrix = np.column_stack((new_x_axis, new_y_axis, new_z_axis))
    # inv_matrix = np.linalg.inv(change_matrix)

    # points = np.matmul(points, inv_matrix.T)
    scatter.set_data(points[:, 0], points[:, 1])
    scatter.set_3d_properties(points[:, 2])

    # plt.plot([0, 1000 * new_x_axis[0]], [0, 1000 * new_x_axis[1]], [0, 1000 * new_x_axis[2]], color='r')
    # plt.plot([0, 1000 * new_y_axis[0]], [0, 1000 * new_y_axis[1]], [0, 1000 * new_y_axis[2]], color='g')
    # plt.plot([0, 1000 * new_z_axis[0]], [0, 1000 * new_z_axis[1]], [0, 1000 * new_z_axis[2]], color='b')

    # new_grav = np.matmul(gravity, inv_matrix.T)

    # plt.plot([0, 1000 * new_grav[0]], [0, 1000 * new_grav[1]], [0, 1000 * new_grav[2]], color='r')



# fig = plt.figure(0, figsize=(10, 10))
# ax = fig.add_subplot(211, projection='3d')

fig = plt.figure(0, figsize=(20, 10))
ax = fig.add_subplot(211, projection='3d')
# ax.set_size_inches(10, 10, 10)

# ax.scatter(x, y, z, c='r', s=0.5)
scatter, = ax.plot(0, 0, 0, color='b', marker='.', linestyle='None', markersize=2)

scatter_all, = ax.plot(0, 0, 0, color='r', marker='.', linestyle='None', markersize=0.5)
# ax.plot_surface(x, y, z, alpha=0.5)
ax.set_xlim(-2000, 2000)
ax.set_zlim(-2000, 2000)
ax.set_ylim(0, 3000)

# Set the axis labels and show the plot
ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')

folder = 'plage_tranchee'

ax2 = fig.add_subplot(212, )
# ax2 = axs[1]

img=ax2.imshow(np.load('images/' + folder + '/rgb/rgb_data_' + str(0) + '.npy'))


# set the spacing between subplots
plt.subplots_adjust(left=0,
                    bottom=0.1,
                    right=1,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.4)


# fig.tight_layout()

def update(i):
    print(i)
    depth_array = np.load('images/' + folder + '/depth/depth_data_' + str(i) + '.npy')
    img_array = np.load('images/' + folder + '/rgb/rgb_data_' + str(i) + '.npy')
    # imu_array = np.load('images/' + folder + '/imu/imu_data_' + str(i) + '.npy')
    # print(imu_array)
    #displayPC2(depth_array, 0, scatter)
    img.set_data(img_array)
    displayPC(depth_array, scatter, scatter_all)
    


# update(70)
ani = FuncAnimation(fig, update, interval=100)
# ani.save('cube.mp4', writer='ffmpeg', fps=10)
plt.show()