import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import numpy as np
import sys
from collections import namedtuple
import ctypes
import math
import struct

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

import time

from .filter.slope_filter import getPC


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    # if skip_nans:
    #     if uvs:
    #         for u, v in uvs:
    #             p = unpack_from(data, (row_step * v) + (point_step * u))
    #             has_nan = False
    #             for pv in p:
    #                 if isnan(pv):
    #                     has_nan = True
    #                     break
    #             if not has_nan:
    #                 yield p
    #     else:
    #         for v in range(height):
    #             offset = row_step * v
    #             for u in range(width):
    #                 p = unpack_from(data, offset)
    #                 has_nan = False
    #                 for pv in p:
    #                     if isnan(pv):
    #                         has_nan = True
    #                         break
    #                 if not has_nan:
    #                     yield p
    #                 offset += point_step
    # else:
    #     if uvs:
    #         for u, v in uvs:
    #             yield unpack_from(data, (row_step * v) + (point_step * u))
    #     else:
    #         for v in range(height):
    #             offset = row_step * v
    #             for u in range(width):
    #                 # print(type(unpack_from(data, offset)), unpack_from(data, offset))
    #                 yield unpack_from(data, offset)
    #                 offset += point_step

    pcd = np.zeros((3, height, width))

    for v in range(height):
        offset = row_step * v
        for u in range(width):
            data_tuple = unpack_from(data, offset)
            pcd[:, v, u] = np.array([data_tuple[0], data_tuple[1], data_tuple[2]])
            offset += point_step

    return pcd


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame)

    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

class SlopeFilterNode(Node):
    def __init__(self):
        super().__init__('slope_filter_node')
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/slope_points',
            qos_profile_sensor_data)
        self.publisher_2 = self.create_publisher(
            PointCloud2,
            '/not_slope_points',
            qos_profile_sensor_data)

        self.subscription_ = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.callback,
            qos_profile_sensor_data)

        # declare parameters to be given as input to the 
        self.declare_parameter('gridshape_y', 16)
        self.declare_parameter('gridshape_x', 32)
        self.declare_parameter('max_height', 2000)
        self.declare_parameter('max_dist', 6000)
        self.declare_parameter('num_iter', 20)
        self.declare_parameter('thresh_dist', 20)
        self.declare_parameter('num_inliers', 200)
        self.declare_parameter('obstacle_angle', 45)
        self.declare_parameter('how_to_replace', 'random')

        self.declare_parameter('filter', 'ransac')
        self.declare_parameter('device', 'cpu')



    def callback(self, msg):
        print('Received message')
        t1 = time.time()
        # get parameters values
        gridshape_y = self.get_parameter('gridshape_y').get_parameter_value().integer_value
        gridshape_x = self.get_parameter('gridshape_x').get_parameter_value().integer_value
        max_height = self.get_parameter('max_height').get_parameter_value().integer_value
        max_dist = self.get_parameter('max_dist').get_parameter_value().integer_value
        num_iter = self.get_parameter('num_iter').get_parameter_value().integer_value
        thresh_dist = self.get_parameter('thresh_dist').get_parameter_value().integer_value
        num_inliers = self.get_parameter('num_inliers').get_parameter_value().integer_value
        obstacle_angle = self.get_parameter('obstacle_angle').get_parameter_value().integer_value

        how_to_replace = self.get_parameter('how_to_replace').get_parameter_value().string_value
        filter_ = self.get_parameter('filter').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value

        points = read_points(msg, skip_nans=False) * 1000.0
        points, not_points= getPC(points, gridshape = (gridshape_y, gridshape_x), max_height = max_height, max_dist = max_dist, num_iter = num_iter, thresh_dist = thresh_dist, num_inliers = num_inliers, \
                        how_to_replace = how_to_replace, filter = filter_, device = device, \
                        obstacle_angle = obstacle_angle)

        pcd = point_cloud(points.numpy() / 1000.0, msg.header.frame_id)
        not_pcd = point_cloud(not_points.numpy() / 1000.0, msg.header.frame_id)

        self.publisher_.publish(pcd)
        self.publisher_2.publish(not_pcd)

        t4 = time.time()
        print("Time for all: ", t4-t1)

    def draw(self):

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(self.x[:512], self.y[:512], self.z[:512], s=5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show(block=True)




def main(args=None):
    print('Slope Filter Node started')
    rclpy.init(args=args)

    slope_filter_node = SlopeFilterNode()

    rclpy.spin(slope_filter_node)

    slope_filter_node.destroy_node()
    rclpy.shutdown()