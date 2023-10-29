# import numpy as np
# from matplotlib.animation import FuncAnimation
# import matplotlib.pyplot as plt

# from sklearn.neighbors import BallTree

# from ransacObstacles import convertToPointCloud, ransac_3d_cuda

# import open3d as o3d
# import time

# visualization_window = o3d.visualization.Visualizer()
# visualization_window.create_window()

# render_options = visualization_window.get_render_option()
# render_options.background_color = np.asarray([1, 1, 1])  # Set background color to black

# # view_control = visualization_window.get_view_control()
# # pinhole = view_control.convert_to_pinhole_camera_parameters()

# # pointcloud = o3d.geometry.PointCloud()
# # visualization_window.add_geometry(pointcloud)
# # visualization_window.run()

# ctr = visualization_window.get_view_control()
# ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
# ctr.set_up([ -0.026393788080636981, -0.99417853568218162, 0.10446246761199787 ])
# ctr.set_front([ -0.051869689604595301, -0.10299608789487412, -0.99332841556993345 ])
# ctr.set_zoom(0.58)

# def displaySubGridPC(depth_array, pointcloud, visualization_window, color):
#     skip_points = 5
#     array = depth_array[::skip_points, ::skip_points]
#     array = array.astype(np.float32)

#     # filter points too far or too close
#     max_depth = 3000
#     min_depth = 100

#     mask = (array < max_depth) * (array > min_depth)

#     array = array * mask
#     # print(array.shape)
#     # print(img_array.shape)

#     f_x = 798.31
#     # f_y = f_x

#     c_x = 655.73
#     c_y = 313.12

#     f_x = f_x / (skip_points)
#     c_x = c_x / (skip_points)
#     c_y = c_y / (skip_points)

#     mask, x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

#     points = np.column_stack((x, y, z))

#     num_iterations = 10
#     threshold_distance = 30
#     num_inliers = 200

#     # t1 = time.time()
#     best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu', num_random_pts=3)
#     if best_plane is None:
#         print('No plane found')
#         # pointcloud.points = o3d.utility.Vector3dVector(points)
#         # pointcloud = o3d.geometry.PointCloud()
#         pointcloud.points = o3d.utility.Vector3dVector(points)
#         pointcloud.colors = o3d.utility.Vector3dVector(np.repeat(np.asarray([[0, 0, 0]]), points.shape[0], axis=0))
#         visualization_window.add_geometry(pointcloud)
#     else:
#         y_pred = -(best_plane[0] * x + best_plane[2] * z + best_plane[3]) / best_plane[1]

#         # normalize plane vector
#         plane_vector = np.array([best_plane[0], best_plane[1], best_plane[2]])
#         plane_vector = plane_vector / np.linalg.norm(plane_vector)

#         # compute scalar product between plane and gravity vector
#         gravity_vector = np.array([0, 1, 0])
#         scalar_product = np.dot(plane_vector, gravity_vector)

#         # compute angle between plane and gravity vector
#         angle = np.arccos(np.abs(scalar_product)) * 180 / np.pi
#         if angle > 45:
#             print('obstacle detected')
#             print('Angle: ', angle)


#             # points = np.column_stack((x, y_pred, z))
#             #t2 = time.time()
#             #print('Time taken: ', t2 - t1)
#             #pointcloud = o3d.geometry.PointCloud()
            
#             pointcloud.points = o3d.utility.Vector3dVector(points)

#             # pointcloud = pointcloud.paint_uniform_color(color)
#             # pointcloud.point_size = 5
#             pointcloud.colors = o3d.utility.Vector3dVector(np.repeat(color, points.shape[0], axis=0))
#             visualization_window.add_geometry(pointcloud)
#         else:
#             print('no obstacle detected')

#             pointcloud.points = o3d.utility.Vector3dVector(points)
#             pointcloud.colors = o3d.utility.Vector3dVector(np.repeat(np.asarray([[0, 0, 0]]), points.shape[0], axis=0))
#             # visualization_window.add_geometry(pointcloud)
#             visualization_window.add_geometry(pointcloud)

#     # ctr = visualization_window.get_view_control()
#     # ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
#     # ctr.set_up([ -0.026393788080636981, -0.99417853568218162, 0.10446246761199787 ])
#     # ctr.set_front([ -0.051869689604595301, -0.10299608789487412, -0.99332841556993345 ])
#     # ctr.set_zoom(0.4)

#     # visualization_window.poll_events()
#     # visualization_window.update_renderer()
#     return pointcloud

# def displayPC(depth_array, image, pointcloud, visualization_window):
#     skip_points = 1
#     array = depth_array[::skip_points, ::skip_points]
#     array = array.astype(np.float32)

#     img_array = image[::skip_points, ::skip_points]

#     # filter points too far or too close
#     max_depth = 3000
#     min_depth = 100

#     mask = (array < max_depth) * (array > min_depth)

#     array = array * mask
#     # print(array.shape)
#     # print(img_array.shape)

#     f_x = 798.31
#     # f_y = f_x

#     c_x = 655.73
#     c_y = 313.12

#     f_x = f_x / (skip_points)
#     c_x = c_x / (skip_points)
#     c_y = c_y / (skip_points)

#     mask, x, y, z = convertToPointCloud(array, f_x, c_x, c_y)

#     img_array = img_array.ravel()[mask]

#     points = np.column_stack((x, y, z))
    
#     pointcloud.points = o3d.utility.Vector3dVector(points)
#     # print(img_array.shape)
#     img_array = img_array.reshape((-1, 1))
#     img_array = np.repeat(img_array, 3, axis=1)
#     # print(img_array)
#     pointcloud.colors = o3d.utility.Vector3dVector(img_array.astype(float) / 255.0)
    
#     visualization_window.add_geometry(pointcloud)

#     ctr = visualization_window.get_view_control()
#     ctr.set_lookat([ -149.72369756109833, -100.50982700955763, 1679.5 ])
#     ctr.set_up([ -0.026393788080636981, -0.99417853568218162, 0.10446246761199787 ])
#     ctr.set_front([ -0.051869689604595301, -0.10299608789487412, -0.99332841556993345 ])
#     ctr.set_zoom(0.4)

#     visualization_window.poll_events()
#     visualization_window.update_renderer()

# def masks(height, width, gridsize):
#     num_rows = gridsize[0]
#     num_cols = gridsize[1]

#     # get the set of indicies for each grid cell
#     rows = np.arange(height)
#     cols = np.arange(width)
#     masks = np.zeros((num_rows, num_cols, height, width), dtype=bool)

#     for i in range(num_rows):
#         for j in range(num_cols):
#             row_mask = (rows >= i * height / num_rows) * (rows < (i + 1) * height / num_rows)
#             col_mask = (cols >= j * width / num_cols) * (cols < (j + 1) * width / num_cols)

#             mask = row_mask[:, None] * col_mask[None, :]
#             masks[i, j] = mask
#             #yield mask
#     return masks

# folder = 'b'
# i = 10
# threshold = 100
# depth_array_ = np.load('images/' + folder + '/depth/depth_data_' + str(i) + '.npy')
# conf_array_ = np.load('images/' + folder + '/confidence/conf_data_' + str(i) + '.npy')
# # img_array_ = np.load('images/' + folder + '/rgb/gray_data_' + str(i) + '.npy')
# # imu_array_ = np.load('images/' + folder + '/imu/imu_data_' + str(i) + '.npy')



# masks = masks(depth_array_.shape[0], depth_array_.shape[1], (8, 8))

# pointclouds = [[o3d.geometry.PointCloud() for i in range(masks.shape[1])] for j in range(masks.shape[0])]
# for p in pointclouds:
#     for q in p:
#         visualization_window.add_geometry(q)

# colors = [[np.random.random(size=(1, 3)) for i in range(masks.shape[1])] for j in range(masks.shape[0])]
# k = 10
# while True:
#     t1 = time.time()
#     for i in range(masks.shape[0]):
#         for j in range(masks.shape[1]):
#             depth_array_ = np.load('images/' + folder + '/depth/depth_data_' + str(k) + '.npy')
#             conf_array_ = np.load('images/' + folder + '/confidence/conf_data_' + str(k) + '.npy')
#             color = colors[i][j]
#             # color = np.array([[1, 0, 0]])
#             depth_array = depth_array_ * (conf_array_ < threshold) * masks[i, j]
#             # img_array = img_array * (conf_array < threshold) * masks[i, j]
#             pointcloud = pointclouds[i][j]

#             pointcloud = displaySubGridPC(depth_array, pointcloud, visualization_window, color)
#             pointclouds[i][j] = pointcloud
#             # print(i,j)
#         # break
#     t2 = time.time()
#     print('time taken: ', t2 - t1)
#     # "front" : [ -0.022158323630945928, -0.33817347909490836, -0.94082288807761894 ],
#     # "lookat" : [ -100.24880059124916, 328.07703774223046, 1656.0 ],
#     # "up" : [ 0.0029821503629833112, -0.941072111824794, 0.33819282535978645 ],
#     # "zoom" : 0.47999999999999976

#     ctr = visualization_window.get_view_control()
#     ctr.set_lookat([ -100.24880059124916, 328.07703774223046, 1656.0 ])
#     ctr.set_up([ 0.0029821503629833112, -0.941072111824794, 0.33819282535978645 ])
#     ctr.set_front([ -0.022158323630945928, -0.33817347909490836, -0.94082288807761894 ])
#     ctr.set_zoom(0.47999999999999976)

#     visualization_window.poll_events()
#     visualization_window.update_renderer()
#     k+=1
#     break

#     time.sleep(0.1)

# # i = 5
# # j = 5
# # color = np.random.random(size=(3, 1))
# # color = np.array([[1, 0, 0]])
# # depth_array = depth_array * (conf_array < threshold) * masks[i, j]
# # # img_array = img_array * (conf_array < threshold) * masks[i, j]

# # displaySubGridPC(depth_array, pointcloud, visualization_window, color)
# # print(i,j)

# # displayPC(depth_array, img_array, pointcloud, visualization_window)
# visualization_window.run()
# visualization_window.destroy_window()

# # plt.figure()
# # plt.imshow(conf_array)
# # plt.show()


# # from scipy.ndimage.filters import gaussian_filter, convolve1d

# # def computeGradient(depth_array):
# #     grad_i = convolve1d(depth_array, np.array([1, 0, -1]), axis=1, mode='constant')
# #     grad_j = convolve1d(depth_array, np.array([1, 0, -1]), axis=0, mode='constant')

# #     grad_norm = np.sqrt(grad_i ** 2 + grad_j ** 2)
# #     return grad_norm

# # grad_norm = computeGradient(depth_array)

# # grad_norm = gaussian_filter(grad_norm, sigma = 10)

# # plt.figure()
# # plt.imshow(grad_norm)
# # plt.show()



