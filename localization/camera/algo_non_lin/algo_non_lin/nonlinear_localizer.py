#!/usr/bin/env python3
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker
from scipy.optimize import least_squares
import copy
import rclpy
import numpy as np


def quat_to_yaw(q):
    """Extract yaw from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


class NonlinearRangeBearing(Node):
    def __init__(self):
        super().__init__("nonlinear_rb_localizer")

        # --- parameters ---
        self.declare_parameter("map_xmin", -60.0)
        self.declare_parameter("map_xmax",  60.0)
        self.declare_parameter("map_ymin", -60.0)
        self.declare_parameter("map_ymax",  60.0)
        self.declare_parameter("range_noise_std", 0.4)
        self.declare_parameter("bearing_noise_std_deg", 3.5)

        self.map_xmin = self.get_parameter("map_xmin").value
        self.map_xmax = self.get_parameter("map_xmax").value
        self.map_ymin = self.get_parameter("map_ymin").value
        self.map_ymax = self.get_parameter("map_ymax").value
        self.range_noise_std = self.get_parameter("range_noise_std").value
        self.bearing_noise_std_deg = self.get_parameter("bearing_noise_std_deg").value

        self.relative_position = {}

        # Convex seed received from nav_global_loc_convex
        self._convex_seed = None
        # Robot yaw from EKF
        self._robot_yaw = None
        ######atention cest pas les meme id que les anchors il ya une correspondance !!!

        self.anchor = {
            1: np.array([2, 1.5]),
            0: np.array([-2.75, 1.0]),
            2: np.array([1.15, -1.65]),
            7: np.array([-2.75, -0.5]),
        }
        # id 53 ===== id2 
        # id 58 === id 7

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

        self.pub = self.create_publisher(Marker, "aruco_rover_pos_rviz", qos)
        self.pub_pos = self.create_publisher(Odometry, "/pos_aruco_non_lineaire", qos)

        # Subscribe to pose_estimator output: position seed + yaw
        self.create_subscription(
            Odometry, "/aruco_rover_pos", self._convex_seed_callback, qos)

        self._cube_merge_ttl_sec = 0.4
        self._last_cube_detect_msg = None
        self._last_cube_detect_stamp = None
        self._last_cube_phi_msg = None
        self._last_cube_phi_stamp = None

        self.create_subscription(
            ArucoMarkers, "/cube_markers", self._cube_detect_callback, qos)
        self.create_subscription(
            ArucoMarkers, "/cube_markers_phi", self._cube_phi_callback, qos)

        self.get_logger().info("Nonlinear Range+Bearing Node started (seed + yaw from /aruco_rover_pos pose_estimator).")

    # ----------------------------------------------------------------
    # Convex seed callback (from nav_global_loc_convex)
    # ----------------------------------------------------------------
    def _convex_seed_callback(self, msg: Odometry):
        self._convex_seed = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        # Extract yaw from pose_estimator orientation (set via yaw_to_quat)
        # The convex solver publishes identity quaternion (yaw=0) — skip those
        q = msg.pose.pose.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-3:
            self._robot_yaw = quat_to_yaw(q)

    # ----------------------------------------------------------------
    # Rotate bearing from robot frame to world frame
    # ----------------------------------------------------------------
    def _rotate_to_world(self, vx_robot, vy_robot, yaw):
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        vx_world = cos_y * vx_robot - sin_y * vy_robot
        vy_world = sin_y * vx_robot + cos_y * vy_robot
        return vx_world, vy_world

    # ----------------------------------------------------------------
    # Nonlinear least-squares refinement
    # ----------------------------------------------------------------
    def solve_nonlinear(self, measurements, seed):
        if seed is None:
            return None

        seed_arr = np.array(seed, dtype=float)
        seed_arr[0] = np.clip(seed_arr[0], self.map_xmin, self.map_xmax)
        seed_arr[1] = np.clip(seed_arr[1], self.map_ymin, self.map_ymax)

        # Bound the search around the seed to avoid degenerate minima at anchors
        margin = 3.0
        lb = [max(seed_arr[0] - margin, self.map_xmin),
              max(seed_arr[1] - margin, self.map_ymin)]
        ub = [min(seed_arr[0] + margin, self.map_xmax),
              min(seed_arr[1] + margin, self.map_ymax)]

        range_sigma = max(self.range_noise_std, 1e-6)
        bearing_sigma = max(np.deg2rad(self.bearing_noise_std_deg), 1e-6)

        def residuals(pos):
            res = []
            for m in measurements:
                anchor = np.array([m["ax"], m["ay"]])
                delta = pos - anchor
                distance = np.linalg.norm(delta)

                # Large penalty near anchors to prevent degenerate solutions
                if distance < 0.5:
                    res.append(50.0)
                    res.append(50.0)
                    continue

                predicted_bearing = delta / distance
                measured_bearing = np.array([m["vx"], m["vy"]])

                # range residual
                res.append((distance - m["range"]) / range_sigma)

                # bearing residual (angular error)
                cross = (measured_bearing[0] * predicted_bearing[1]
                         - measured_bearing[1] * predicted_bearing[0])
                dot = np.clip(measured_bearing @ predicted_bearing, -1.0, 1.0)
                res.append(np.arctan2(cross, dot) / bearing_sigma)

            return np.array(res)

        result = least_squares(
            residuals, seed_arr,
            bounds=(lb, ub),
            loss="soft_l1",
        )

        if not result.success:
            self.get_logger().warn(f"Nonlinear refinement failed: {result.message}")
            return None

        return float(result.x[0]), float(result.x[1])

    # ----------------------------------------------------------------
    # Merge /cube_markers + /cube_markers_phi
    # ----------------------------------------------------------------
    def _merge_cube_sources(self):
        now = self.get_clock().now()
        ttl_ns = int(self._cube_merge_ttl_sec * 1e9)
        use_det = (
            self._last_cube_detect_msg is not None
            and self._last_cube_detect_stamp is not None
            and (now - self._last_cube_detect_stamp).nanoseconds < ttl_ns)
        use_phi = (
            self._last_cube_phi_msg is not None
            and self._last_cube_phi_stamp is not None
            and (now - self._last_cube_phi_stamp).nanoseconds < ttl_ns)
        if not use_det and not use_phi:
            return None
        if use_det:
            out = copy.deepcopy(self._last_cube_detect_msg)
        else:
            out = ArucoMarkers()
        if use_phi:
            phi = self._last_cube_phi_msg
            existing = set(int(x) for x in out.marker_ids)
            for i, mid in enumerate(phi.marker_ids):
                if int(mid) in existing:
                    continue
                if i >= len(phi.poses) or i >= len(phi.ar_angles_list):
                    continue
                out.marker_ids.append(mid)
                out.poses.append(phi.poses[i])
                out.ar_angles_list.append(phi.ar_angles_list[i])
                existing.add(int(mid))
        if len(out.marker_ids) == 0:
            return None
        return out

    def _cube_detect_callback(self, msg):
        self._last_cube_detect_msg = msg
        self._last_cube_detect_stamp = self.get_clock().now()
        self._dispatch_merged_cube()

    def _cube_phi_callback(self, msg):
        self._last_cube_phi_msg = msg
        self._last_cube_phi_stamp = self.get_clock().now()
        self._dispatch_merged_cube()

    def _dispatch_merged_cube(self):
        merged = self._merge_cube_sources()
        if merged is None:
            return
        self.aruco_callback(merged)

    # ----------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------
    def aruco_callback(self, msg):
        pos = None

        # Need yaw to transform bearings to world frame
        if self._robot_yaw is None:
            self.get_logger().warn(
                "No yaw yet (waiting for /fused_nav_ekf_odom)...",
                throttle_duration_sec=2.0)
            return

        yaw = self._robot_yaw

        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id not in self.anchor:
                self.get_logger().warn(f"Unknown marker ID {marker_id}, skipping...")
                continue

            pos = msg.poses[i].position
            x_cam, y_cam = pos.x, pos.y

            range_measured = np.sqrt(x_cam**2 + y_cam**2)
            if range_measured < 1e-6:
                self.get_logger().warn(f"Marker {marker_id} too close, skipping...")
                continue

            # Bearing in robot frame (anchor -> rover direction)
            vx_robot = -x_cam / range_measured
            vy_robot = -y_cam / range_measured

            # Rotate to world frame using EKF yaw
            vx_world, vy_world = self._rotate_to_world(vx_robot, vy_robot, yaw)

            anchor_pos = self.anchor[marker_id]
            self.relative_position[marker_id] = {
                "ax": anchor_pos[0],
                "ay": anchor_pos[1],
                "range": range_measured,
                "vx": vx_world,
                "vy": vy_world,
            }

        if len(self.relative_position) < 2:
            self.get_logger().info(
                f"Collected {len(self.relative_position)} markers, need >= 2")
            return

        # Wait for convex seed from nav_global_loc_convex
        if self._convex_seed is None:
            self.get_logger().warn(
                "No convex seed yet (waiting for /aruco_rover_pos)...")
            return

        meas = list(self.relative_position.values())
        seed = self._convex_seed

        # Nonlinear refinement using convex solution as seed
        xy = self.solve_nonlinear(meas, seed)
        if xy is None:
            self.get_logger().warn("Nonlinear refinement failed, using convex seed.")
            xy = seed

        # Publish on /pos_aruco_non_lineaire
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = xy[0]
        odom.pose.pose.position.y = xy[1]
        self.pub_pos.publish(odom)

        self.get_logger().info(
            f"[NL] Robot position: x={xy[0]:.3f}  y={xy[1]:.3f} "
            f"(convex seed: {seed[0]:.3f}, {seed[1]:.3f}  yaw: {np.degrees(yaw):.1f} deg)",
            throttle_duration_sec=1.0)

        # Publish rviz marker
        if pos is not None:
            pose_aruco = Pose()
            pose_aruco.position.x = pos.x
            pose_aruco.position.y = pos.y
            pose_aruco.position.z = pos.z
            m = Marker()
            m.header = msg.header
            m.ns = 'aruco_nl'
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = pose_aruco
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 0.6
            self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NonlinearRangeBearing()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
