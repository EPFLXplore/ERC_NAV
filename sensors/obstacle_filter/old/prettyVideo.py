import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

from sklearn.neighbors import BallTree

from ransacObstacles import convertToPointCloud, ransac_3d_cuda

import open3d as o3d
import time

def displayPC(ransac, depth_array, image, pointcloud, visualization_window, i):
    skip_points = 1
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

    img_array = img_array.ravel()[mask]

    points = np.column_stack((x, y, z))

    num_iterations = 50
    threshold_distance = 50
    num_inliers = 200

    if ransac:
        best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu', num_random_pts=3)
    else:
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
        for j in range(points.shape[0]):
            if np.any(dist[j] > radius):
                delidx[j] = False

        points = points[delidx]
        img_array = img_array[delidx]

    
    pointcloud.points = o3d.utility.Vector3dVector(points)
    # print(img_array.shape)
    img_array = img_array.reshape((-1, 1))
    img_array = np.repeat(img_array, 3, axis=1)
    # print(img_array)
    pointcloud.colors = o3d.utility.Vector3dVector(img_array.astype(float) / 255.0)
    
    visualization_window.add_geometry(pointcloud)
    
    # visualization_window.get_render_option().save_to_json("view_old.json")
    # visualization_window.get_render_option().load_from_json("view.json")

    # camera_position = [0, 0, 3000]
    # view_control.set_lookat(np.asarray(camera_position))
    #view_control.rotate(0, 400)
    # ctr = visualization_window.get_view_control()
    # ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
    # ctr.set_up([ -0.016448497628074307, -0.98467888393408143, -0.17359994948187993 ])
    # ctr.set_front([ -0.034899353457976671, 0.17408304371772917, -0.98411235589143309 ])
    # ctr.set_zoom(0.7)

    ctr = visualization_window.get_view_control()
    ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
    ctr.set_up([ -0.026393788080636981, -0.99417853568218162, 0.10446246761199787 ])
    ctr.set_front([ -0.051869689604595301, -0.10299608789487412, -0.99332841556993345 ])
    ctr.set_zoom(0.4)

    visualization_window.poll_events()
    visualization_window.update_renderer()
    # screen = visualization_window.capture_screen_float_buffer()
    # plt.figure(figsize=(12, 7))
    # plt.imshow(np.asarray(screen))
    # file_name = f'images/output/filter_{i:04d}.jpeg'
    # plt.savefig(file_name)
    # plt.close()
    

    
    #pointcloud.paint_uniform_color(np.asarray([1, 0, 0]))  # Set point cloud color to red
    #pointcloud.point_size = 1.0  # Set point size to 1.0



    # Update the visualization window after modifying the settings
    

visualization_window = o3d.visualization.Visualizer()
visualization_window.create_window()

render_options = visualization_window.get_render_option()
render_options.background_color = np.asarray([0, 0, 0])  # Set background color to black

# view_control = visualization_window.get_view_control()
# pinhole = view_control.convert_to_pinhole_camera_parameters()

pointcloud = o3d.geometry.PointCloud()
visualization_window.add_geometry(pointcloud)
# visualization_window.run()

ctr = visualization_window.get_view_control()
ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
ctr.set_up([ -0.026393788080636981, -0.99417853568218162, 0.10446246761199787 ])
ctr.set_front([ -0.051869689604595301, -0.10299608789487412, -0.99332841556993345 ])
ctr.set_zoom(0.58)


def update(ransac, i, pointcloud, visualization_window, folder):

    depth_array = np.load('images/' + folder + '/depth/depth_data_' + str(i) + '.npy')
    img_array = np.load('images/' + folder + '/rgb/gray_data_' + str(i) + '.npy')
    
    # plt.figure(figsize=(12, 7))
    # plt.imshow(img_array)
    # plt.savefig(f'images/output/rgb_{i:04d}.jpeg')
    # plt.close()

    displayPC(ransac, depth_array, img_array, pointcloud, visualization_window, i)
    # 

    time.sleep(0.1)

def update2(i, folder):
    img_array = np.load('images/' + folder + '/rgb/rgb_data_' + str(i) + '.npy')
    
    plt.figure(figsize=(12, 7))
    plt.imshow(img_array, cmap='gray')
    plt.savefig(f'images/output/rgb_{i:04d}.jpeg')
    plt.close()

folder = 'a'
ransac = False
i = 0

while True:
    try:
        # 
        update(ransac, i, pointcloud, visualization_window, folder)
        # update2(i, folder)
        i += 1
    except:
        print('done')
        break


# update(120, pointcloud, visualization_window, folder)
# visualization_window.run()
visualization_window.destroy_window()