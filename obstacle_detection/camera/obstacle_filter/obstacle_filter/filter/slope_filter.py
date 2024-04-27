import torch
import numpy as np

from .filter import ransac, lstsq


def getPC(pc, gridshape = (8, 8), \
        max_height = 2000.0, max_dist = 6000.0, num_iter = 20, thresh_dist = 20, num_inliers = 200, \
        how_to_replace = 'random', filter = 'ransac', device = 'cpu', \
        obstacle_angle = 45):

    ground_vector = torch.tensor([0, 0, 1], device=device)

    pc = torch.from_numpy(pc.astype(np.float32))

    # reshape the pointcloud image into a grid of subimages
    grid_height = (pc.shape[1] // gridshape[0])
    grid_width = (pc.shape[2] // gridshape[1])

    subgrid_size = (grid_height, grid_width)

    subimages = torch.split(pc, subgrid_size[0], dim=1)
    subimages = [torch.split(subimage, subgrid_size[1], dim=2) for subimage in subimages]

    array = torch.stack([torch.stack(subimage_row, dim=1) for subimage_row in subimages], dim=1)
    array = array.reshape(3, gridshape[0], gridshape[1], grid_height * grid_width)

    # filter 0 : don't do the filtering
    if filter == 'no_filter':
        array = array.reshape(3, -1)
        array = array.transpose(dim0=0, dim1=1)
        return array
    elif filter == 'off':
        return
    
    orig_array = array
    
    dist = torch.linalg.norm(array[:2], dim = 0).clone()

    # filter 1 : points too high or too far
    array = array * (array[2, :, :, :] < (max_height-1.0)) * (dist < max_dist) * (dist> 2000)

    # filter 2 : compute ratio of valid points to all points, 
    # if ratio is too low, then discard the subgrid
    valid_subgrids = torch.zeros((gridshape[0], gridshape[1]), dtype=bool) # grid_x x grid_y

    num_pts_per_subgrid = grid_height * grid_width
    
    sums = torch.sum(dist > 10, dim=2) # sum all of the points that are not at (0, 0, 0) invalid

    ratio = 0.4
    for i in range(gridshape[0]):
        for j in range(gridshape[1]):
            valid_pts_mask = dist[i, j, :] > 0

            # if number of valid points is greater than ratio
            if sums[i,j] > num_pts_per_subgrid * ratio:
                # set subgrid to be valid
                valid_subgrids[i, j] = 1

                # get the valid points
                valid_pts = array[:, i, j, valid_pts_mask] # 3 x num_valid_pts

                if how_to_replace == 'median':
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
                    raise ValueError('how_to_replace must be one of median, mean, random')
            else:
                # ratio is not satisfied, so set all points to be random. This is so the lstsq function works on CUDA
                random_values = torch.normal(mean = 1000, std = 100, size=(array.shape[0], array.shape[-1]))
                array[:, i, j, :] = random_values

    if filter == 'ransac':
        coeff = ransac(array, num_iter, thresh_dist, num_inliers, num_random_pts=3, device=device) # grid_x x grid_y x 4
    elif filter == 'lstsq':
        coeff = lstsq(array, device=device) # grid_x x grid_y x 3

    plane_vectors = torch.stack([coeff[:, :, 0], coeff[:, :, 1], -1*torch.ones_like(coeff[:, :, 1])], dim=2)

    plane_vectors = plane_vectors / torch.linalg.norm(plane_vectors, dim=2, keepdim=True) # grid_x x grid_y x 3

    # compute scalar product between plane and gravity vector
    scalar_products = torch.sum(plane_vectors * ground_vector, dim=2, keepdim=False) # grid_x x grid_y

    # compute angle between plane and gravity vector
    angles = torch.arccos(torch.abs(scalar_products)) * 180 / torch.pi # grid_x x grid_y

    # only consider angles which are valid
    angles = angles * valid_subgrids
    print('Angles: ', angles)

    valid_subgrids = (angles > obstacle_angle) # grid_x x grid_y

    array = array[:, valid_subgrids, :]
    not_array = orig_array[:, ~valid_subgrids, :]

    array = array.reshape(3, -1)
    array = array.transpose(dim0=0, dim1=1)

    not_array = not_array.reshape(3, -1)
    not_array = not_array.transpose(dim0=0, dim1=1)

    return array, not_array