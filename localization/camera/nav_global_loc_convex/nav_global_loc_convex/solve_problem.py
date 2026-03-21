#!/usr/bin/env python3
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Pose
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
import numpy as np
import cvxpy as cp

class ConvexRangeBearing(Node):
    def __init__(self):
        super().__init__("convex_rb_localizer")

        self.declare_parameter("lambda_bearing", 999999.0) #99999999
        self.declare_parameter("map_xmin", -60.0)
        self.declare_parameter("map_xmax",  60.0)
        self.declare_parameter("map_ymin", -60.0)
        self.declare_parameter("map_ymax",  60.0)

        self.lambda_bearing = self.get_parameter("lambda_bearing").get_parameter_value().double_value
        
        self.map_xmin = self.get_parameter("map_xmin").value
        self.map_xmax = self.get_parameter("map_xmax").value
        self.map_ymin = self.get_parameter("map_ymin").value
        self.map_ymax = self.get_parameter("map_ymax").value

        self.relative_position = {}

        # 4 anchors for demo (anchors a absolute positions of the aruco tags)
        # self.anchor = {
        #     1: np.array([2.09, 0]),
        #     2: np.array([-1.87, -1.23]),
        #     7: np.array([-2.22, 1.17]),
        #     6: np.array([2.55, -1.0]),
        # }

        # self.anchor = {
        #     1: np.array([1.60, 0]),
        #     2: np.array([1.60, -0.65]),
        #     7: np.array([-1.70, 0.90]),
        #     6: np.array([-1.70, -0.55]),
        # }
        self.anchor = {
            1: np.array([4.11, -0.8]),
            6: np.array([4.0, 1.03]),
            2: np.array([-2.70,2.81]),
            7: np.array([-3.41, -3.0]),
        }

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.pub = self.create_publisher(Marker, "aruco_rover_pos_rviz", qos)
        self.pub_pos = self.create_publisher(Odometry, "/aruco_rover_pos", qos)

        self.sub_mes_ = self.create_subscription(
            ArucoMarkers, "/cube_markers", self.aruco_callback, qos)
        self.sub_mes_


        self.get_logger().info("Convex Range+Bearing Node started.")

    # ------------------------------------------------------------
    # Core convex solver (matches paper eq. (9)) :contentReference[oaicite:1]{index=1}
    # ------------------------------------------------------------
    def solve_cvx(self, measurements):
        M = len(measurements)

        x = cp.Variable(2)       # robot position
        w = cp.Variable((M, 2))  # anchor slack variables
        t = cp.Variable(M)       # residual radii

        constraints = []
        objective = 0

        for k, m in enumerate(measurements):
            ax, ay = m["ax"], m["ay"]
            r = m["range"]
            v = np.array([m["vx"], m["vy"]])

            # ----------- SOC for residual: || x - a_k - w_k || <= t_k -----------
            constraints.append(
                cp.SOC(t[k], x - np.array([ax, ay]) - w[k])
            )

            # ----------- SOC for range: || w_k || <= r_k -----------
            constraints.append(
                cp.SOC(r, w[k])
            )

            # ----------- Bearing objective term: -ṽᵀ w_k -----------
            vtilde = (self.lambda_bearing / r) * v
            objective += t[k] - vtilde @ w[k]

        # ----------- MAP BOUNDS (REAL CONSTRAINTS) -----------
        constraints += [
            x[0] >= self.map_xmin,
            x[0] <= self.map_xmax,
            x[1] >= self.map_ymin,
            x[1] <= self.map_ymax,
        ]

        self.get_logger().info(f"Convex Range+Bearing LAMBDA VAL : {self.lambda_bearing}")


        prob = cp.Problem(cp.Minimize(objective), constraints)

        try:
            prob.solve(solver=cp.ECOS, verbose=False, max_iters=900, reltol=1e-6, feastol=1e-6)
        except Exception as e:
            self.get_logger().error(f"CVX/ECOS failed: {e}")
            return None

        if x.value is None:
            return None

        return float(x.value[0]), float(x.value[1])

    # ------------------------------------------------------------
    # ROS2 Callback
    # ------------------------------------------------------------
    def aruco_callback(self, msg):
        # msg contains markers with poses in base_link frame
        # Process all detected markers and accumulate measurements
        pos = None

        # Process EACH marker in the message
        for i, marker_id in enumerate(msg.marker_ids):
            # Check if this marker is a known anchor
            if marker_id not in self.anchor:
                self.get_logger().warn(f"Unknown marker ID {marker_id}, skipping...")
                continue
            
            # Get relative position from base_link
            pos = msg.poses[i].position
            x_cam, y_cam, z_cam = pos.x, pos.y, pos.z
            self.get_logger().info(f"Detected marker ID {marker_id} at ({x_cam:.2f}, {y_cam:.2f})")

            # Calculate range (distance from camera to marker)
            range_measured = np.sqrt(x_cam**2 + y_cam**2)
            if range_measured < 1e-6:
                self.get_logger().warn(f"Marker {marker_id} too close, skipping...")
                continue
            
            # Calculate bearing unit vector: simple normalization
            # pose is already in base_link frame
            vx = - x_cam / range_measured
            vy = - y_cam / range_measured
            
            bearing_angle = np.arctan2(vy, vx) * 180 / np.pi
            self.get_logger().info(
                f"  Marker {marker_id}: range={range_measured:.2f}m, "
                f"bearing=({vx:.3f}, {vy:.3f}), angle={bearing_angle:.1f}°"
            )

            # Get absolute anchor position from the map
            anchor_pos = self.anchor[marker_id]
            
            # Store measurement with ABSOLUTE anchor position
            self.relative_position[marker_id] = {
                "ax": anchor_pos[0],      # Absolute x of anchor
                "ay": anchor_pos[1],      # Absolute y of anchor
                "range": range_measured,  # Relative : Range from camera to marker
                "vx": vx,                 # Relative : Bearing unit vector x-component
                "vy": vy                  # Relative : Bearing unit vector y-component   
            }
        
        if len(self.relative_position) < 2:
            self.get_logger().info(f"Collected {len(self.relative_position)} aruco markers, need at least 2")
            return
        
        # Solve for robot position
        self.get_logger().info(f"Solving with {len(self.relative_position)} markers...")
        xy = self.solve_cvx(list(self.relative_position.values()))
        
        if xy is None:
            self.get_logger().error("CVX failed to find a solution.")
            return

        pose = Odometry()
        pose.pose.pose.position.x = xy[0]
        pose.pose.pose.position.y = xy[1]
        self.pub_pos.publish(pose)

        self.get_logger().info(
            f"[RB] Robot position: x={pose.pose.pose.position.x:.3f}  y={pose.pose.pose.position.y:.3f}",
            throttle_duration_sec=1.0
        )

        if pos is not None:

            pose_aruco = Pose()
            pose_aruco.position.x, pose_aruco.position.y, pose_aruco.position.z = pos.x, pos.y, pos.z        
            m = Marker()
            m.header = msg.header
            m.ns     = 'aruco'
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.pose   = pose_aruco
            # size of the cube
            m.scale.x = 0.1 
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.6
            self.get_logger().info(f"Publishing marker at x={pose_aruco.position.x:.3f} y={pose_aruco.position.y:.3f} z={pose_aruco.position.z:.3f} id={msg.marker_ids[0]}")

            self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ConvexRangeBearing()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
