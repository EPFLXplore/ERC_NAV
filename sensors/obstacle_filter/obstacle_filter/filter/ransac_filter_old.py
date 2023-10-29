import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
import numpy as np
import torch

def filter_cuda(points):
    #fct = lambda x, z, coeff: coeff[0] + coeff[1] * x + coeff[2] * z

    ### parameters

    skip_points = 50

    # ransac parameters

    num_iterations = 10
    threshold_distance = 25
    num_inliers = 500
    num_random_pts = 12

    # distance filter threshold
    threshold = 100

    # outlier removal parameters
    numberpts = 10
    radius = 200
    leaf_size = 10

    
    points = points[::skip_points]

    best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, num_random_pts, device='cuda')

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

    # filter points that are too far away from the plane
    
    points = points[np.abs(y_pred - y) > threshold]

    # remove outliers
    tree = BallTree(points, leaf_size=leaf_size, metric='euclidean')

    dist, ind = tree.query(points, k=numberpts) # query the tree for the 20 nearest neighbors for each point

    # if one of the 20 nearest neighbors is more than 200mm away, remove the point
    delidx = np.ones(points.shape[0], dtype=bool)
    for i in range(points.shape[0]):
        if np.any(dist[i] > radius):
            delidx[i] = False

    points = points[delidx]
    return points


def ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device, num_random_pts=12):
    ### gpu code
    # load the points to the gpu
    points = torch.from_numpy(points).to(device)

    # create a random index array
    random_indices = torch.randint(0, points.shape[0], (num_iterations, num_random_pts) ).to(device)
    # rand_ind_1 = torch.randint(0, num_iterations)
    # print(points.shape)
    # points = points.repeat(num_iterations, 1, 1)
    print(points.shape)
    print(random_indices.shape)

    random_points = points[random_indices, :] # num_iter x random_pts x 3
    print(points.shape)

    # fit plane to random points

    # Construct the matrix A

    A = torch.cat([random_points[:, :, 0:1]*0 + 1, random_points[:, :, 0:1], random_points[:, :, 2:3]], dim=2)#.permute(0, 2, 1) # y = c + ax + bz
    b = random_points[:, :, 1]

    print(A.shape) # 10 x 12 x 3, num_iter x num_random_pts x 3
    print(b.shape) # 10 x 12

    coeff, _, _, _ = torch.linalg.lstsq(A, b)
    print(coeff.shape) # 10 x 3

    coeffs = torch.stack([coeff[:, 1], -1*torch.ones_like(coeff[:, 1]), coeff[:, 2], coeff[:, 0]], dim=1)
    print(coeffs.shape) # 10 x 4

    # Compute the distance between each point and the plane
    distances = torch.abs(points.matmul(coeffs[:, :3].permute(1, 0)) + coeffs[:, 3]) / torch.linalg.norm(coeffs[:, :3], dim=1)

    print(distances.shape) # pc_size x 10
    # Count the number of inliers
    inliers = torch.sum(distances < threshold_distance, dim=0)
    # print(inliers.shape)

    # choose the best plane
    best_plane_id = torch.argmax(inliers, dim=0)
    # print(coeffs.shape)

    best_plane = coeffs[best_plane_id]

    return best_plane


def filter(points):
    fct = lambda x, z, coeff: coeff[0] + coeff[1] * x + coeff[2] * z

    skip_points = 50
    points = points[::skip_points]

    num_iterations = 10
    threshold_distance = 25
    num_inliers = 500

    threshold = 100

    best_plane = ransac_3d(points, num_iterations, threshold_distance, num_inliers)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

    # filter points that are too far away from the plane
    
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
    return points

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
    coeff, r, rank, s = np.linalg.lstsq(A, b)

    coeffs = np.array([coeff[1], -1, coeff[2], coeff[0]])

    return coeffs
