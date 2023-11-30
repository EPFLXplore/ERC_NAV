import depthai as dai
import numpy as np
import matplotlib.pyplot as plt

# Create pipeline
pipeline = dai.Pipeline()

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

# we set LeftRightCheck to true and ExtendedDisparity to true
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)
# stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5)


# Define where the depth data will be sent
xoutDepth = pipeline.createXLinkOut()
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)

xoutRGB = pipeline.createXLinkOut()
xoutRGB.setStreamName("rgb")
stereo.rectifiedRight.link(xoutRGB.input)
# monoRight.out.link(xoutRGB.input)

img_num = 0

# Connect to the device
with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the output defined above
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    calibData = device.readCalibration()

    intrinsics = calibData.getCameraIntrinsics(monoRight.getBoardSocket(), resizeWidth=1280, resizeHeight=720)
    print(intrinsics)

    # Create a numpy array to store the depth data
    #depth_data = np.zeros((stereo.getMaxDepth()*2, stereo.getMaxDepth()), dtype=np.uint8)

    # Continuously get the depth frames and save them as numpy arrays
    while True and img_num < 100:
        in_depth = q_depth.get()
        in_rgb = q_rgb.get()

        depth_array = in_depth.getFrame()
        rgb_array = in_rgb.getFrame()
        
        print(depth_array.shape, depth_array.dtype, in_depth.getType(), in_depth.getCategory())
        print(rgb_array.shape, rgb_array.dtype, in_rgb.getType(), in_rgb.getCategory())



        img_num += 1

    plt.figure()
    plt.imshow(depth_array)

    plt.figure()
    plt.imshow(rgb_array)

    plt.show(block=True)

        #np.save('rgb_image.npy', rgb_array)

        # float_array = in_depth.getFrame().astype(float)

        # print(float_array.shape)
        # np.save('images/depth_data_' + str(img_num) + '.npy', float_array)


        # uint_array = in_depth.getFrame()
        
        
        # print(uint_array.dtype)

        # np.save('images/slope2/depth_data_' + str(img_num) + '.npy', uint_array)
        # img_num += 1

        # depth_frame = in_depth.getData()#.reshape((stereo.getMaxDepth()*2, stereo.getMaxDepth()))
        # print(len(depth_frame))
        #np.copyto(depth_data, depth_frame)
        
        # Save the depth data as a numpy array
        #np.save('depth_data.npy', depth_data)
