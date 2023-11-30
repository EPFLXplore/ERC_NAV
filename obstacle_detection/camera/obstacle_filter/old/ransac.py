import numpy as np
import time
import matplotlib.pyplot as plt

import numpy as np
from numpy.polynomial.polynomial import polyval2d

fct = lambda coeff, x_, z_ : coeff[0] + coeff[1] * x_ + coeff[2] * z_ + coeff[3] * x_ * z_ + coeff[4] * x_**2 + coeff[5] * z_**2 + coeff[6] * x_**2*z_ + coeff[7] * x_*z_**2 + coeff[8] * x_**2*z_**2

def ransac_poly2d(points, num_iterations, threshold_distance, num_inliers):
    best_model = None
    best_score = 0
    
    for i in range(num_iterations):
        # Randomly select a subset of points
        subset = points[np.random.choice(points.shape[0], 20, replace=False)] # 9 dof so at least 9 points needed

        x = subset[:, 0]
        z = subset[:, 1]
        y = subset[:, 2]

        # 2nd order polynomial floor
        A = np.array([x*0 + 1, x, z, x*z, x**2, z**2, x**2*z, x*z**2, x**2*z**2]).T
        #print(A.shape, y.shape)

        coeff, r, rank, s = np.linalg.lstsq(A, y)
        
        # # Fit a polynomial surface of degree 2 to the subset of points
        # x, y, z = subset[:, 0], subset[:, 1], subset[:, 2]
        # coefs = np.polyfit2d(x, y, z, deg=2)

        # Compute the score of the fitted model
        # surface_z = polyval2d(points[:, 0], points[:, 1], coefs)
        surface_z = fct(coeff, points[:, 0], points[:, 1])
        errors = surface_z - points[:, 2]
        inliers = np.abs(errors) < threshold_distance
        score = np.sum(inliers)
        #print(score)
        
        # Update the best model parameters and best score
        if score > best_score and np.sum(inliers) >= num_inliers:
            best_model = coeff
            best_score = score
    
    print(best_model)
    # Fit a polynomial surface of degree 2 to all the points using the best model parameters
    if best_model is not None:
        coeff = best_model
    # else:
    #     x, y, z = points[:, 0], points[:, 1], points[:, 2]
    #     coefs = np.polyfit2d(x, y, z, deg=2)
    
    return coeff


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
        sample_indices = np.random.choice(len(points), size=3, replace=False)
        sample_points = points[sample_indices]

        # Fit a plane to the three points
        plane_params = fit_plane(sample_points)

        # Compute the distance between each point and the plane
        distances = np.abs(points.dot(plane_params[:3]) + plane_params[3]) / np.linalg.norm(plane_params[:3])

        # Count the number of inliers
        inliers = np.sum(distances < threshold_distance)

        # Update the best plane if this iteration has more inliers
        if inliers > max_inliers and inliers >= num_inliers:
            max_inliers = inliers
            best_plane = plane_params

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
    A = np.ones((3, 4))
    A[:, :3] = points

    # Compute the least-squares solution to Ax = 0
    _, _, V = np.linalg.svd(A)
    plane_params = V[-1, :]

    # Normalize the plane parameters
    norm = np.linalg.norm(plane_params[:3])
    plane_params = plane_params / norm

    return plane_params


hd = True

start = time.time()
array = np.load('images/slope/depth_data_100.npy')

skip_points = 20

# array = array[skip_points:-skip_points, skip_points:-skip_points] # now 630 by 390

array = array[::skip_points, ::skip_points] # now 63 by 

#print(array.shape)

array = array.astype(np.float32)

# for final point cloud
max_depth = 2000
min_depth = 100

array = array * (array < max_depth) * (array > min_depth)

if hd: # 720p
    f_x = 798.31
    # f_y = f_x

    c_x = 655.73
    c_y = 313.12

else: # 400p
    f_x = 399.15
    # f_y = f_x

    c_x = 327.86
    c_y = 176.55

f_x = f_x / (skip_points)
c_x = c_x / (skip_points)
c_y = c_y / (skip_points)

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

    x = x.ravel()
    x = x[x!=0]
    y = y.ravel()
    y = y[y!=0]

    depth_image = depth_image.ravel()
    depth_image = depth_image[depth_image!=0]

    return x, y, depth_image

x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

points = np.column_stack((x, y, z))

num_iterations = 100
threshold_distance = 25
num_inliers = 500

time_1 = time.time()
best_plane = ransac_3d(points, num_iterations, threshold_distance, num_inliers)
time_2 = time.time()

print('RANSAC plane fitting took {} seconds'.format(time_2 - time_1))

print(best_plane)

y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], y_pred, points[:, 2], c='r', s=0.5)
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', s=0.5)
# ax.plot_surface(x, y, z, alpha=0.5)

# Set the axis labels and show the plot
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()