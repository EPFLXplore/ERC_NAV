import numpy as np
import time
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
import numpy as np
import torch

### make this code easily interchangeable between cpu and gpu

plots = False
plane = True

fct = lambda x, z, coeff: coeff[0] + coeff[1] * x + coeff[2] * z

def convertToPointCloud(depth_image, focal_length, c_x, c_y):
    height, width = depth_image.shape

    x = np.arange(0, width)
    y = np.arange(0, height)

    # notZeros = depth_image!=0

    x, y = np.meshgrid(x, y)

    x = x - c_x
    y = y - c_y

    x = x * depth_image / focal_length
    y = y * depth_image / focal_length
    # print(x.shape)
    x = x.ravel()
    x = x[x!=0]
    y = y.ravel()
    y = y[y!=0]

    depth_image = depth_image.ravel()
    mask = depth_image!=0
    # print(x.shape)
    # print(y.shape)
    # print(mask.shape)
    depth_image = depth_image[depth_image!=0]

    return mask, x, y, depth_image

def ransac_3d(points, num_iterations, threshold_distance, num_inliers):
    """
    RANSAC algorithm for fitting a plane to 3D point cloud data.

    :param points: 3D point cloud data (array-like object of shape (N, 3))
    :param num_iterations: number of iterations for RANSAC algorithm (int)
    :param threshold_distance: threshold distance for inliers (float)
    :param num_inliers: number of inliers required to fit a plane (int)
    :return: plane parameters (array-like object of shape (4,))
    """
    max_inliers = 0
    best_plane = None

    for i in range(num_iterations):
        # Randomly select three points
        sample_indices = np.random.choice(len(points), size=10, replace=False)
        sample_points = points[sample_indices]

        # Fit a plane to the three points
        coeffs = fit_plane(sample_points)

        # Compute the distance between each point and the plane
        distances = np.abs(points.dot(coeffs[:3]) + coeffs[3]) / np.linalg.norm(coeffs[:3])

        # Count the number of inliers
        inliers = np.sum(distances < threshold_distance)

        # Update the best plane if this iteration has more inliers
        if inliers > max_inliers and inliers >= num_inliers:
            max_inliers = inliers
            best_plane = coeffs

    if best_plane is None:
        raise ValueError("Could not fit a plane to the point cloud data")

    return best_plane

def fit_plane(points):
    """
    Fit a plane to 3D points using a least-squares regression.

    :param points: 3D points (array-like object of shape (3, 3))
    :return: plane parameters (array-like object of shape (4,))
    """
    # Construct the matrix A
    A = np.array([points[:, 0]*0 + 1, points[:, 0], points[:, 2]]).T # y = c + ax + bz
    b = points[:, 1]

    print(A.shape)
    print(b.shape)
    coeff, r, rank, s = np.linalg.lstsq(A, b)

    coeffs = np.array([coeff[1], -1, coeff[2], coeff[0]])

    return coeffs


def check_plane(plane_coeffs):
    # multiple checks on the plane

    # if plane normal is more than a certain angle versus flat orientation, consider as obstacle.
    # this could be for high positive slopes in front of the rover.

    # what would a hole in front of the rover look like on the camera ?

    # what if we are going over a hole or obstacle, how to deal with that?
    pass

fct = lambda coeff, x_, z_ : coeff[:, 0:1] + coeff[:, 1:2] * x_ + coeff[:, 2:3] * z_ + coeff[:, 3:4] * x_ * z_ + coeff[:, 4:5] * x_*z_**2 + coeff[:, 5:6] * x_**2*z_ + coeff[:, 6:7] * x_**2 + coeff[:, 7:8] * z_**2  + coeff[:, 8:9] * x_**2*z_**2

poly_fct = lambda x_, z_ : torch.cat([x_ * 0 + 1, x_, z_, x_ * z_, x_ * z_**2, z_ * x_**2, x_**2, z_**2, x_**2 * z_**2], dim=2)

def ransac_polynomial(points, num_iterations, threshold_distance, num_inliers, device, num_random_pts=30):
    ### gpu code
    # load the points to the gpu
    points = torch.from_numpy(points).to(device)

    # create a random index array
    random_indices = torch.randint(0, points.shape[0], (num_iterations, num_random_pts) ).to(device)
    random_points = points[random_indices, :] # num_iter x num_random_pts x 3

    A = poly_fct(random_points[:, :, 0:1], random_points[:, :, 2:3])
    b = random_points[:, :, 1]
    print(A.shape)
    print(b.shape)

    coeff, _, _, _ = torch.linalg.lstsq(A, b) # num_iter x num_coefs

    # compute vertical distance between points and polynomial surface
    # if we wanted to do the normal surface, we simply need to compute the gradient of the function, which we can do analytically.
    # we have to multiply by vectors of size num_points
    x = points[:, 0:1]
    y = points[:, 1:2]
    z = points[:, 2:3]
    y_pred = fct(coeff, x.T, z.T) # num_iter x num_points

    # compute vertical distance
    distances = torch.linalg.norm(y.T - y_pred, keepdim=True)
    print(distances.shape)
    inliers = torch.sum(distances < threshold_distance, dim=1)

    # choose the best plane
    best_surface_id = torch.argmax(inliers, dim=0)
    # print(coeffs.shape)

    best_surface = coeff[best_surface_id]
    best_surface_eval = y_pred[best_surface_id]

    return best_surface, best_surface_eval

def ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device, num_random_pts=3):
    ### gpu code
    # load the points to the gpu
    points = torch.from_numpy(points)
    t1 = time.time()
    
    points = points.to(device, non_blocking=True)
    t2 = time.time()
    print("Time to points on device : ", t2-t1)

    # create a random index array
    random_indices = torch.randint(0, points.shape[0], (num_iterations, num_random_pts) ).to(device)
    # rand_ind_1 = torch.randint(0, num_iterations)
    # print(points.shape)
    # points = points.repeat(num_iterations, 1, 1)
    # print(points.shape)
    # print(random_indices.shape)

    random_points = points[random_indices, :] # num_iter x random_pts x 3
    # print(points.shape)

    # fit plane to random points

    # Construct the matrix A

    A = torch.cat([random_points[:, :, 0:1]*0 + 1, random_points[:, :, 0:1], random_points[:, :, 2:3]], dim=2)#.permute(0, 2, 1) # y = c + ax + bz
    b = random_points[:, :, 1]

    # print(A.shape) # 10 x 12 x 3, num_iter x num_random_pts x 3
    # print(b.shape) # 10 x 12

    coeff, _, _, _ = torch.linalg.lstsq(A, b)
    # print(coeff.shape) # 10 x 3

    coeffs = torch.stack([coeff[:, 1], -1*torch.ones_like(coeff[:, 1]), coeff[:, 2], coeff[:, 0]], dim=1)
    # print(coeffs.shape) # 10 x 4

    # Compute the distance between each point and the plane
    distances = torch.abs(points.matmul(coeffs[:, :3].permute(1, 0)) + coeffs[:, 3]) / torch.linalg.norm(coeffs[:, :3], dim=1)

    # print(distances.shape) # pc_size x 10
    # Count the number of inliers
    inliers = torch.sum(distances < threshold_distance, dim=0)
    # print(inliers.shape)

    
    # min_inliers = 500
    enough = inliers > num_inliers

    if torch.sum(enough) == 0:
        return None

    # choose the best plane
    best_plane_id = torch.argmax(inliers, dim=0)
    # print(coeffs.shape)

    best_plane = coeffs[best_plane_id]
    best_plane = best_plane.to('cpu')

    return best_plane

    

    # create a random point array
    random_points = points[random_indices]

    # fit plane to random points
    # Construct the matrix A
    A = torch.cat([points[:, 0]*0 + 1, points[:, 0], points[:, 2]], dim=1).T # y = c + ax + bz
    b = points[:, 1]
    coeff, _, _, _ = np.linalg.lstsq(A, b)

    coeffs = torch.array([coeff[1], -1, coeff[2], coeff[0]])

    # Compute the distance between each point and the plane
    distances = torch.abs(points.dot(coeffs[:3]) + coeffs[3]) / torch.linalg.norm(coeffs[:3])

    # Count the number of inliers
    inliers = torch.sum(distances < threshold_distance)

    # Update the best plane if this iteration has more inliers
    if inliers > max_inliers and inliers >= num_inliers:
        max_inliers = inliers
        best_plane = coeffs

    return best_plane




# hd = True

# start = time.time()
# array = np.load('images/slope/depth_data_100.npy')

# skip_points = 10

# # array = array[skip_points:-skip_points, skip_points:-skip_points] # now 630 by 390

# array = array[::skip_points, ::skip_points] # now 63 by 

# #print(array.shape)

# array = array.astype(np.float32)

# # for final point cloud
# max_depth = 2000
# min_depth = 100

# array = array * (array < max_depth) * (array > min_depth)

# if hd: # 720p
#     f_x = 798.31
#     # f_y = f_x

#     c_x = 655.73
#     c_y = 313.12

# else: # 400p
#     f_x = 399.15
#     # f_y = f_x

#     c_x = 327.86
#     c_y = 176.55

# f_x = f_x / (skip_points)
# c_x = c_x / (skip_points)
# c_y = c_y / (skip_points)



# x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

# points = np.column_stack((x, y, z))

# num_iterations = 10
# threshold_distance = 25
# num_inliers = 500

# time_1 = time.time()

# if plane:
#     best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu')
#     time_2 = time.time()
#     print('Other stuff took {} seconds'.format(time_1 - start))
#     print('RANSAC plane fitting took {} seconds'.format(time_2 - time_1))

#     print(best_plane)

#     y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

#     # filter points that are too far away from the plane
#     threshold = 100
#     points = points[np.abs(y_pred - y) > threshold]
# else:
#     _, y_pred = ransac_polynomial(points, 30, threshold_distance, num_inliers, 'cpu')
#     time_2 = time.time()
#     print('Other stuff took {} seconds'.format(time_1 - start))
#     print('RANSAC surface fitting took {} seconds'.format(time_2 - time_1))

#     threshold=100
#     points = points[np.abs(y_pred - y) > threshold]

# if plots:
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(x, y, z, c='r', s=0.5)
#     ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', s=0.5)
#     # ax.plot_surface(x, y, z, alpha=0.5)

#     # Set the axis labels and show the plot
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plt.show()

# time_3 = time.time()


# # remove outliers
# numberpts = 10
# radius = 200
# tree = BallTree(points, leaf_size=10, metric='euclidean')

# dist, ind = tree.query(points, k=numberpts) # query the tree for the 20 nearest neighbors for each point

# # if one of the 20 nearest neighbors is more than 200mm away, remove the point
# delidx = np.ones(points.shape[0], dtype=bool)
# for i in range(points.shape[0]):
#     if np.any(dist[i] > radius):
#         delidx[i] = False

# points = points[delidx]

# time_4 = time.time()

# print('Removing outliers took {} seconds'.format(time_4 - time_3))

# if plots:
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     # ax.scatter(x, y, z, c='r', s=0.5)
#     ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', s=0.5)
#     # ax.plot_surface(x, y, z, alpha=0.5)

#     # Set the axis labels and show the plot
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plt.show()

# print('Total time: {} seconds'.format(time_4 - start))
