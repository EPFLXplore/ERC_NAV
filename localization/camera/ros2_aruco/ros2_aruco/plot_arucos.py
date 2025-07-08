#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
import std_msgs.msg

class ArucoMarkersViz(Node):
    def __init__(self):
        super().__init__('aruco_markers_viz')
        self.declare_parameter('input_topic',  'aruco_markers')
        self.declare_parameter('output_topic', 'aruco_markers_vis')
        self.declare_parameter('marker_size',  0.144)  # meters
        
        in_t  = self.get_parameter('input_topic').value
        out_t = self.get_parameter('output_topic').value
        size  = self.get_parameter('marker_size').value
        
        self.sub = self.create_subscription(
            ArucoMarkers, in_t, self.cb_markers, 10
        )
        self.pub = self.create_publisher(MarkerArray, out_t, 10)
        self.marker_size = size
        
        self.get_logger().info(f"Listening on {in_t}, publishing MarkerArray on {out_t}")

    def cb_markers(self, msg: ArucoMarkers):
        ma = MarkerArray()
        for idx, pose in zip(msg.marker_ids, msg.poses):
            m = Marker()
            m.header = msg.header
            m.ns     = 'aruco'
            m.id     = int(idx)           # unique per marker
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.pose   = pose
            # size of the cube
            m.scale.x = self.marker_size
            m.scale.y = self.marker_size
            m.scale.z = self.marker_size
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.6

            ma.markers.append(m)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkersViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
