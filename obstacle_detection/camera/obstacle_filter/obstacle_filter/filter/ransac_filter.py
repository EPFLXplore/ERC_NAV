import torch
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def convertToPointCloud(array, f_x, c_x, c_y):
    height, width = array.shape
    x = torch.arange(0, width)
    y = torch.arange(0, height)

    x, y = torch.meshgrid(x, y, indexing='xy')

    x = x - c_x
    y = y - c_y

    x = x * array / f_x
    y = y * array / f_x

    return x, y, array

def ransac(array, num_iterations, threshold_distance, num_inliers, num_random_pts=3, device='cpu'):

    
    array = array.to(device) # 3 x grid_x x grid_y x (height * width)
    array = array.permute(1, 2, 3, 0) # grid_x x grid_y x (height * width) x 3  

    # for each subgrid element we want to select num_random_pts random 
    num_pts = array.shape[-2]

    random_indices = torch.randint(0, num_pts, \
                                   (num_iterations, num_random_pts)) # shape num_iterations, num_random_pts

    random_points = array[:, :, random_indices, :] # shape grid_x x grid_y x num_iter x num_random_pts x 3

    # A (x, y, 1)
    A = torch.cat([random_points[:, :, :, :, 0:1], \
                   random_points[:, :, :, :, 1:2], \
                   torch.ones_like(random_points[:, :, :, :, 2:3])], dim = 4) # grid_x x grid_y x num_iter x num_random_pts x 3
    # b (z)
    b = random_points[:, :, :, :, 2]

    coeff, _, _, _ = torch.linalg.lstsq(A, b) # grid_x x grid_y x num_iter x 3

    coeffs = torch.stack([coeff[:, :, :, 0], \
                          coeff[:, :, :, 1], \
                          -1 * torch.ones_like(coeff[:, :, :, 2]), coeff[:, :, :, 2]], dim=3) # grid_x x grid_y x num_iter x 4

        # (grid_x x grid_y x (height * width) x 3) x (grid_x x grid_y x 3 x num_iter) + (grid_x x grid_y x 1 x num_iter)
    distances = torch.abs(array.matmul(coeffs[:, :, :, :3].permute(0, 1, 3, 2)) + \
                          coeffs[:, :, :, 3:4].permute(0, 1, 3, 2)) / \
                          torch.linalg.norm(coeffs[:, :, :, :3], dim = 3, keepdim=True).permute(0, 1, 3, 2) #grid_x x grid_y x (height * width) x num_iter

     # Count the number of inliers
    inliers = torch.sum(distances < threshold_distance, dim=2) # grid_x x grid_y x num_iter

    valid_planes = torch.zeros((inliers.shape[0], inliers.shape[1]), dtype=bool) # grid_x x grid_y

    maxed_inliers = torch.max(inliers, dim = 2) # grid_x x grid_y
    valid_planes = maxed_inliers.values > num_inliers # grid_x x grid_y

    # choose the best plane
    best_plane_ids = maxed_inliers.indices # grid_x x grid_y

    best_planes = torch.zeros((array.shape[0], array.shape[1], 4)).to(device) # grid_x x grid_y x 4

    for i in range(array.shape[0]):
        for j in range(array.shape[1]):
            best_planes[i, j] = coeffs[i, j, best_plane_ids[i, j], :]

    # could I do this without the for loops ?

    return best_planes, valid_planes

def getPC(depth_array, gridshape = (8, 8), \
        max_depth = 3000, min_depth = 100, num_iter = 20, thresh_dist = 20, num_inliers = 200, \
        how_to_replace = 'random', filter = True, device = 'cpu', ground_vector = torch.tensor([0, np.cos(16/180*torch.pi), np.sin(16/180*torch.pi)])):

    
    # print(depth_array.shape)

    if depth_array.shape[0] == 720: # 720p
        f_x = 798.31
        # f_y = f_x

        c_x = 655.73
        c_y = 313.12

    elif depth_array.shape[0] == 400: # 400p
        f_x = 399.15
        # f_y = f_x

        c_x = 327.86
        c_y = 176.55
    
    # before 1280 720
    # after 256 144
    skip_points = 5
    depth_array = depth_array[::skip_points, ::skip_points]

    depth_array = torch.from_numpy(depth_array.astype(np.float32))

    f_x = f_x / skip_points
    c_x = c_x / skip_points
    c_y = c_y / skip_points

    x, y, z = convertToPointCloud(depth_array, f_x, c_x, c_y)
    pc = torch.stack((x, y, z), axis=0) # 3 x height x width

    # reshape the pointcloud image into a grid of subimages
    grid_height = (depth_array.shape[0] // gridshape[0])
    grid_width = (depth_array.shape[1] // gridshape[1])

    subgrid_size = (grid_height, grid_width)

    subimages = torch.split(pc, subgrid_size[0], dim=1)
    subimages = [torch.split(subimage, subgrid_size[1], dim=2) for subimage in subimages]

    array = torch.stack([torch.stack(subimage_row, dim=1) for subimage_row in subimages], dim=1)
    array = array.reshape(3, gridshape[0], gridshape[1], grid_height * grid_width)

    # filter 0 : don't do the filtering
    if filter == False:
        array = array.reshape(3, -1)
        array = array.transpose(dim0=0, dim1=1)
        return array

    # filter 1 : points too far or too close
    array = array * (array[2, :, :, :] < max_depth) * (array[2, :, :, :] > min_depth)

    # filter 2 : compute ratio of valid points to all points, 
    # if ratio is too low, then discard the subgrid
    valid_subgrids = torch.zeros((gridshape[0], gridshape[1]), dtype=bool) # grid_x x grid_y

    num_pts_per_subgrid = grid_height * grid_width
    sums = torch.zeros((gridshape[0], gridshape[1]))
    sums = torch.sum(array[2, :, :, :] > 0, dim=2)

    print(sums)

    ratio = 0.4
    for i in range(gridshape[0]):
        for j in range(gridshape[1]):
            valid_pts_mask = (array[2, i, j, :] > 0) # grid_x x grid_y x nb_pts
            # sum_ = torch.sum(valid_pts_mask)
            # sums[i, j] = sum_

            # if number of valid points is greater than ratio
            if sums[i,j] > num_pts_per_subgrid * ratio:
                # set subgrid to be valid
                valid_subgrids[i, j] = 1

                # get the valid points
                valid_pts = array[:, i, j, valid_pts_mask] # 3 x num_valid_pts

                if how_to_replace == 'gaussian': # not working, due to invalid balues of covariance matrix
                    # set the invalid points to be distributed according to normal dist defined by the valid points
                    # compute mean and cov of valid points
                    mean = torch.mean(valid_pts, axis=1) # 3
                    cov = torch.cov(valid_pts) # 3 x 3
                    distr = torch.distributions.multivariate_normal.MultivariateNormal(mean, cov)
                    array[:, i, j, ~valid_pts_mask] = distr.sample((torch.sum(~valid_pts_mask),)).T
                elif how_to_replace == 'median':
                    median, indices = torch.median(valid_pts, dim=1, keepdim=True) # 
                    array[:, i, j, ~valid_pts_mask] = median
                elif how_to_replace == 'mean':
                    # set invalid points to the mean of the valid points
                    mean = torch.mean(valid_pts, axis=1, keepdim=True)
                    array[:, i, j, ~valid_pts_mask] = mean
                elif how_to_replace == 'random':
                    # set the invalid points to be randomly one of the valid points
                    random_valid_pts = torch.randint(0, valid_pts.shape[1], (torch.sum(~valid_pts_mask),))
                    array[:, i, j, ~valid_pts_mask] = valid_pts[:, random_valid_pts]
                else:
                    raise ValueError('how_to_replace must be one of gaussian, mean, random')
                    pass
            else:
                # ratio is not satisfied so set all points to zero
                array[:, i, j, :] = 0

    # compute the planes for each subgrid
    best_planes, valid_planes = ransac(array, num_iter, thresh_dist, num_inliers, num_random_pts=3, device=device) # grid_x x grid_y x 4

    
    plane_vectors = torch.stack([best_planes[:, :, 0], best_planes[:, :, 1], -1*torch.ones_like(best_planes[:, :, 1])], dim=2)

    plane_vectors = plane_vectors / torch.linalg.norm(plane_vectors, dim=2, keepdim=True) # grid_x x grid_y x 3

    # compute scalar product between plane and gravity vector
    scalar_products = torch.sum(plane_vectors * ground_vector, dim=2, keepdim=False) # grid_x x grid_y

    # compute angle between plane and gravity vector
    angles = torch.arccos(torch.abs(scalar_products)) * 180 / torch.pi # grid_x x grid_y
    angles = angles * valid_subgrids #* valid_planes
    print('Angles: ', angles)

    valid_subgrids = (angles > 45) # grid_x x grid_y

    array = array[:, valid_subgrids, :]

    #z_new = array[ 0, :, :, :] * coeff[:, :, 0:1] + array[1, :, :, :] * coeff[:, :, 1:2] + coeff[:, :, 2:3]

    array = array.reshape(3, -1)
    array = array.transpose(dim0=0, dim1=1)

    return array

    # # plot the best plane found by ransac

    # z_new = array[0, :, :, :] * best_planes[:, :, 0:1] + array[1, :, :, :] * best_planes[:, :, 1:2] + best_planes[:, :, 3:4]
    # array[2, :, :, :] = z_new


    # array = array.reshape(3, -1)
    # array = array.transpose(dim0=0, dim1=1)

    # return array