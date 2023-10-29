import depthai as dai
import numpy as np

# Create pipeline
pipeline = dai.Pipeline()

# Create nodes
# cam = pipeline.createColorCamera()
# cam.setPreviewSize(640, 400)
# cam.setInterleaved(False)
# cam.setCamId(0)

# prop = dai.MonoCameraProperties.SensorResolution.THE_400_P
prop = dai.MonoCameraProperties.SensorResolution.THE_720_P

monoLeft = pipeline.create(dai.node.MonoCamera) # pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(prop)
monoRight = pipeline.create(dai.node.MonoCamera) # pipeline.createMonoCamera()
monoRight.setResolution(prop)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
stereo = pipeline.createStereoDepth()
# stereo.setOutputSize(640, 400)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

# intrinsics = stereo.getCalibration()
# print(intrinsics)

# we set LeftRightCheck to true and ExtendedDisparity to true

stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)
# stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5)


# Define where the depth data will be sent
xoutDepth = pipeline.createXLinkOut()
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)



img_num = 0

# Connect to the device
with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the output defined above
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    calibData = device.readCalibration()

    intrinsics = calibData.getCameraIntrinsics(monoRight.getBoardSocket(), resizeWidth=1280, resizeHeight=720)
    print(intrinsics)

    # Create a numpy array to store the depth data
    #depth_data = np.zeros((stereo.getMaxDepth()*2, stereo.getMaxDepth()), dtype=np.uint8)

    # Continuously get the depth frames and save them as numpy arrays
    while True:
        in_depth = q_depth.get()
        
        print(in_depth.getType())
        print(in_depth.getCategory())
        print(in_depth.getFrame().dtype)

        # float_array = in_depth.getFrame().astype(float)

        # print(float_array.shape)
        # np.save('images/depth_data_' + str(img_num) + '.npy', float_array)
        uint_array = in_depth.getFrame()
        
        
        print(uint_array.dtype)

        np.save('images/slope2/depth_data_' + str(img_num) + '.npy', uint_array)
        img_num += 1

        # depth_frame = in_depth.getData()#.reshape((stereo.getMaxDepth()*2, stereo.getMaxDepth()))
        # print(len(depth_frame))
        #np.copyto(depth_data, depth_frame)
        
        # Save the depth data as a numpy array
        #np.save('depth_data.npy', depth_data)
