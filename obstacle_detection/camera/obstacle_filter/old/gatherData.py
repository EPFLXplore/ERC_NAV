# we use this file to gather data from the camera
# we want to have a infinite loop that will display the color images
# when we click on a button, the recording of the files will start
# when we click on the button again, the recording will stop

import depthai as dai
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2
import os
from sklearn.neighbors import BallTree

from ransacObstacles import convertToPointCloud, ransac_3d_cuda


def displayPC(depth_array, scatter):
    skip_points = 10
    array = depth_array[::skip_points, ::skip_points]
    array = array.astype(np.float32)

    # filter points too far or too close
    max_depth = 2000
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

    num_iterations = 10
    threshold_distance = 25
    num_inliers = 500

    if points.shape[0] == 0:
        print('No points found')
        return

    best_plane = ransac_3d_cuda(points, num_iterations, threshold_distance, num_inliers, device='cpu')
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
        scatter.set_offsets(points)
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

# Create pipeline
pipeline = dai.Pipeline()


### depth and rgb

# prop = dai.MonoCameraProperties.SensorResolution.THE_400_P
prop = dai.MonoCameraProperties.SensorResolution.THE_720_P

monoLeft = pipeline.create(dai.node.MonoCamera) # pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(prop)
monoLeft.setFps(10)
monoRight = pipeline.create(dai.node.MonoCamera) # pipeline.createMonoCamera()
monoRight.setResolution(prop)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setFps(10)
stereo = pipeline.createStereoDepth()

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

xout_conf = pipeline.createXLinkOut()
xout_conf.setStreamName("confidence")
stereo.initialConfig.setConfidenceThreshold(200)
stereo.confidenceMap.link(xout_conf.input)


# camera modes and filters

# we set LeftRightCheck to true and ExtendedDisparity to true
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)


# stereo.setSubpixel(True) # subpixel is useful for seeing far away objects
# MEDIAN_OFF
stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5)

# stereo.initialConfig.setConfidenceThreshold(100)

init_config = stereo.initialConfig.get()
init_config.postProcessing.speckleFilter.enable = True
# init_config.postProcessing.temporalFilter.enable = True
# init_config.postProcessing.temporalFilter.persistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4
stereo.initialConfig.set(init_config)

# stereo.initialConfig.PostProcessing.SpeckleFilter.enable = False

# stereo.initialConfig.PostProcessing.TemporalFilter.enable = True
# stereo.initialConfig.PostProcessing.TemporalFilter.persistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4


# rgbNode = pipeline.createColorCamera()
# #rgbNode.setPreviewSize(1280, 720)
# rgbNode.setInterleaved(False)
# rgbNode.setFps(10)
# rgbNode.setPreviewSize(1280, 720)
# rgbNode.setBoardSocket(dai.CameraBoardSocket.RGB)
# rgbNode.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)




# Define where the depth data will be sent
xoutDepth = pipeline.createXLinkOut()
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)

xoutgray = pipeline.createXLinkOut()
xoutgray.setStreamName("gray")
stereo.rectifiedRight.link(xoutgray.input)
# monoRight.out.link(xoutRGB.input)

### imu
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.createXLinkOut()
xlinkOut.setStreamName("imu")
imu.enableIMUSensor(dai.IMUSensor.GRAVITY, 100)
imu.out.link(xlinkOut.input)

### rgb

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(1280, 720)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

xoutRGB = pipeline.createXLinkOut()
xoutRGB.setStreamName("rgb")

cam.preview.link(xoutRGB.input)



# xoutRGB = pipeline.createXLinkOut()
# xoutRGB.setStreamName("rgb")
# rgbNode.preview.link(xoutRGB.input)


img_num = 0


# fig = plt.figure(0)
# ax = fig.add_subplot(111, projection='3d')
# # ax.scatter(x, y, z, c='r', s=0.5)
# scatter = ax.scatter(0, 0, 0, c='b', s=0.5)
# # ax.plot_surface(x, y, z, alpha=0.5)

# # Set the axis labels and show the plot
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show(block=False)
# plt.ion()
# Connect to the device
with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the output defined above
    q_depth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
    q_gray = device.getOutputQueue(name="gray", maxSize=1, blocking=False)
    q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    imuQueue = device.getOutputQueue(name="imu", maxSize=1, blocking=False)
    q_conf = device.getOutputQueue(name="confidence", maxSize=1, blocking=False)
    calibData = device.readCalibration()

    intrinsics = calibData.getCameraIntrinsics(monoRight.getBoardSocket(), resizeWidth=1280, resizeHeight=720)
    print(intrinsics)

    # Create a numpy array to store the depth data
    #depth_data = np.zeros((stereo.getMaxDepth()*2, stereo.getMaxDepth()), dtype=np.uint8)

    # Continuously get the depth frames and save them as numpy arrays
    path = input("Enter folder where to save data:")
    path = 'images/' + path + '/'
    if not os.path.exists(path):
        os.makedirs(path + 'depth')
        os.makedirs(path + 'rgb')
        os.makedirs(path + 'imu')
        os.makedirs(path + 'confidence')


    
    while True:
    #def update(i):
        in_depth = q_depth.get()
        in_gray = q_gray.get()
        in_rgb = q_rgb.get()
        in_conf = q_conf.get()
        imuData = imuQueue.get()
        # print(imuData.getData())
        # print(imuData.packets)

        depth_array = in_depth.getFrame()
        gray_array = in_gray.getFrame()
        rgb_array = in_rgb.getFrame()
        conf_array = in_conf.getFrame()
        
        # print(depth_array.shape, depth_array.dtype, in_depth.getType(), in_depth.getCategory())
        # print(rgb_array.shape, rgb_array.dtype, in_rgb.getType(), in_rgb.getCategory())

        gravity = np.array([0, 0, 0])

        for imuPacket in imuData.packets:
            print("IMU packet received")
            # print(imuPacket.acceleroMeter)
            acceleroValues = imuPacket.acceleroMeter
            imuF = "{:.06f}"
            print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            gravity = np.array([acceleroValues.x, acceleroValues.y, acceleroValues.z])

        np.save(path + 'depth/depth_data_' + str(img_num) + '.npy', depth_array)
        np.save(path + 'rgb/gray_data_' + str(img_num) + '.npy', gray_array)
        np.save(path + 'rgb/rgb_data_' + str(img_num) + '.npy', rgb_array)
        np.save(path + 'confidence/conf_data_' + str(img_num) + '.npy', conf_array)
        np.save(path + 'imu/imu_data_' + str(img_num) + '.npy', gravity)

        # print(rgb_array.shape)

        cv2.imshow("preview", gray_array)
        cv2.imshow("preview2", in_rgb.getCvFrame())
        #displayPC(depth_array, scatter)
        key = cv2.waitKey(1)
        if key == 27: # exit on ESC
            break

    
        # # display rgb images
        # plt.figure()
        # plt.imshow(rgb_array)

        # plt.close('all')



        img_num += 1
    
    cv2.destroyWindow("preview")
    cv2.destroyWindow("preview2")
    # plt.figure()
    # plt.imshow(depth_array)

    # plt.figure()
    # plt.imshow(rgb_array)

    # plt.show(block=True)

    #ani = FuncAnimation(fig, update, interval=100)

# plt.ioff()
plt.show()