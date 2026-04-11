#!/usr/bin/env python3
"""
Pose estimation node using lidar-refined ArUco cube detections.

Subscribes to /cube_markers (ArucoMarkers) from the ros2_aruco_with_lidar pipeline, and to /fused_nav_ekf_odom (Odometry) for the EKF's odom->base_link state.

Initialization (n == 1 or 2): When only two markers are visible, uses the known start position erc_start_pos = [0.655, 2.515] and the two bearing angles to compute yaw via yaw = atan2(lm_y - y0, lm_x - x0) - bearing, assuming odom == base_link at init time.

Initialization (n >= 3): Uses the embedded convex SOCP solver (cvxpy + ECOS) to solve for (x, y) in the map frame, then deduces yaw via circular mean of per-marker atan2(lm_y - y, lm_x - x) - bearing. Gated to within 1.0 m of the start position during init.

Accumulation: Collects 35 init samples with MAD-based outlier rejection (_calculate_robust_tf_avg) before locking in the initial map->odom TF.

Post-init continuous updates: When 3+ markers are detected, runs the convex solver and updates map->odom TF if the solution is within max_translation_jump. Yaw is only updated every min_yaw_dt.

TF broadcasting: Publishes map->odom via TransformBroadcaster, re-broadcasting at 5 Hz via a timer to prevent TF timeout.

Odometry output: Publishes /aruco_rover_pos (Odometry) in the map frame.
"""

import math
import numpy as np
import cvxpy as cp

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as Rot
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Rot.from_euler('z', yaw).as_quat()  # x, y, z, w
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def wrap(a: float) -> float:
    return (a + np.pi) % (2 * math.pi) - math.pi


class PoseEstimatorLidarNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_lidar_node')

        # --- state ---
        self.x_estimate = 0.0
        self.y_estimate = 0.0
        self.yaw_estimate = 0.0
        self.solved_new_xy = False
        self.measured_new_yaw = False
        self.time_of_last_pose = self.get_clock().now()
        self.time_of_last_yaw_meas = self.get_clock().now()

        self.MAP_SIZE = 300.0

        # --- initialisation averaging ---
        self.nbr_init_callbacks_for_avg = 35
        self.init_callback_counter = 0
        self.initialized_map_odom_tf = False
        self.last_callback_time = self.get_clock().now()
        self.callback_period_limit = 1 / 15.0
        self.avg_initialization_tfs = []
        self.yaw_init_list = []
        self.min_yaw_dt = 1.0

        # --- outlier gating ---
        self.max_translation_jump = 0.8
        self.max_yaw_jump = math.radians(45)

        # --- convex solver params ---
        self.lambda_bearing = 999999.0
        self.map_xmin = -60.0
        self.map_xmax = 60.0
        self.map_ymin = -60.0
        self.map_ymax = 60.0

        # --- EKF odom state ---
        self.odom_pos_x = 0.0
        self.odom_pos_y = 0.0
        self.odom_yaw = 0.0

        # --- current map->base_link estimate (via map->odom + odom->base_link) ---
        self.curr_map_base_x = 0.0
        self.curr_map_base_y = 0.0
        self.curr_map_base_yaw = 0.0

        # --- ERC starting position in the ERC frame !! ---
        self.erc_start_pos = [0.0, 0.0]

        # self.landmark_poses = [
        #     (-0.585, 0.0),
        #     (2.62, 0.505),
        #     (1.46, 8.45),
        #     (-2.28, 15.81),
        #     (3.74, 19.07),
        #     (7.04, 14.67),
        #     (11.46, 19.78),
        #     (15.51, 19.33),
        #     (16.3, 14.87),
        #     (999999, 999999),
        #     (999999, 999999),
        #     (999999, 999999),
        #     (999999, 999999),
        #     (999999, 999999),
        #     (999999, 999999),
        # ]
        self.landmark_poses = [
            (0.96, 3.57), # aruco id 51
            (-1.68, 3.7), # aruco id 52
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
            (999999, 999999),
        ]

        #3.7, 1.68

        # --- callback groups ---
        self._solver_cbg = MutuallyExclusiveCallbackGroup()
        self._rt_cbg = ReentrantCallbackGroup()

        # --- subscribers ---
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cube_sub = self.create_subscription(
            ArucoMarkers, '/cube_markers',
            self.cube_callback, sensor_qos,
            callback_group=self._solver_cbg)

        ekf_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.ekf_sub = self.create_subscription(
            Odometry, '/fused_nav_ekf_odom',
            self.ekf_callback, ekf_qos,
            callback_group=self._rt_cbg)

        # --- publishers ---
        self.odom_pub = self.create_publisher(
            Odometry, '/aruco_rover_pos', ekf_qos)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_map_odom_tf = None
        self.tf_timer = self.create_timer(
            0.2, self._republish_tf, callback_group=self._rt_cbg)

    # ------------------------------------------------------------------
    # EKF odom callback
    # ------------------------------------------------------------------
    def ekf_callback(self, msg: Odometry):
        self.odom_pos_x = msg.pose.pose.position.x
        self.odom_pos_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, self.odom_yaw = Rot.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz')

    # ------------------------------------------------------------------
    # Periodic TF re-broadcast to prevent TF timeout
    # ------------------------------------------------------------------
    def _republish_tf(self):
        if self.prev_map_odom_tf is not None:
            self.prev_map_odom_tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)

    # ------------------------------------------------------------------
    # Convex SOCP solver  (from solve_problem.py)
    # ------------------------------------------------------------------
    def solve_cvx(self, measurements: list) -> tuple | None:
        M = len(measurements)
        if M < 2:
            return None

        x = cp.Variable(2)
        w = cp.Variable((M, 2))
        t = cp.Variable(M)

        constraints = []
        objective = 0

        for k, m in enumerate(measurements):
            ax, ay = m["ax"], m["ay"]
            r = m["range"]
            v = np.array([m["vx"], m["vy"]])

            constraints.append(cp.SOC(t[k], x - np.array([ax, ay]) - w[k]))
            constraints.append(cp.SOC(r, w[k]))

            vtilde = (self.lambda_bearing / r) * v
            objective += t[k] - vtilde @ w[k]

        constraints += [
            x[0] >= self.map_xmin, x[0] <= self.map_xmax,
            x[1] >= self.map_ymin, x[1] <= self.map_ymax,
        ]

        prob = cp.Problem(cp.Minimize(objective), constraints)
        try:
            prob.solve(solver=cp.ECOS, verbose=False,
                       max_iters=900, reltol=1e-6, feastol=1e-6)
        except Exception as e:
            self.get_logger().error(f"SOCP solver failed: {e}")
            return None

        if x.value is None:
            return None
        return float(x.value[0]), float(x.value[1])

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _pose_to_mat(x: float, y: float, yaw: float) -> np.ndarray:
        c, s = math.cos(yaw), math.sin(yaw)
        M = np.eye(4)
        M[0:2, 0:2] = [[c, -s], [s, c]]
        M[0, 3], M[1, 3] = x, y
        return M

    def _build_map_odom_tf(self, map_x, map_y, map_yaw) -> TransformStamped:
        T_map_base = self._pose_to_mat(map_x, map_y, map_yaw)
        T_odom_base = self._pose_to_mat(
            self.odom_pos_x, self.odom_pos_y, self.odom_yaw)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = T_map_odom[0, 3]
        tf_msg.transform.translation.y = T_map_odom[1, 3]
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quat(
            math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
        return tf_msg

    def _circular_mean_yaw(self, yaw_list: list) -> float:
        return math.atan2(
            sum(math.sin(y) for y in yaw_list),
            sum(math.cos(y) for y in yaw_list))

    def _deduce_yaw(self, est_x, est_y, valid_markers, msg) -> float:
        yaw_list = []
        for (idx, _, k) in valid_markers:
            lm = self.landmark_poses[idx]
            bearing_map = math.atan2(lm[1] - est_y, lm[0] - est_x)
            measured_phi = math.radians(msg.ar_angles_list[k])
            yaw_list.append(wrap(bearing_map - measured_phi))
        return self._circular_mean_yaw(yaw_list)

    def _publish_odom(self, x, y, yaw):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        self.odom_pub.publish(msg)

    def _update_curr_map_base(self):
        """Refresh current map->base_link estimate from TF chain."""
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'map', Time())
            T_map_odom = self._pose_to_mat(
                tf.transform.translation.x,
                tf.transform.translation.y,
                Rot.from_quat([
                    tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w
                ]).as_euler('xyz')[2])
            T_odom_base = self._pose_to_mat(
                self.odom_pos_x, self.odom_pos_y, self.odom_yaw)

            if abs(self.odom_pos_x) < 1e-4 and abs(self.odom_pos_y) < 1e-4:
                self.curr_map_base_x = self.x_estimate
                self.curr_map_base_y = self.y_estimate
                self.curr_map_base_yaw = self.yaw_estimate
            else:
                T_map_base = T_map_odom @ T_odom_base
                self.curr_map_base_x = T_map_base[0, 3]
                self.curr_map_base_y = T_map_base[1, 3]
                self.curr_map_base_yaw = math.atan2(
                    T_map_base[1, 0], T_map_base[0, 0])
                if not self.solved_new_xy:
                    self.x_estimate = self.curr_map_base_x
                    self.y_estimate = self.curr_map_base_y
                    self.yaw_estimate = self.curr_map_base_yaw
        except TransformException:
            if self.prev_map_odom_tf is None:
                self.curr_map_base_x = self.odom_pos_x
                self.curr_map_base_y = self.odom_pos_y
                self.curr_map_base_yaw = self.odom_yaw

    def _calculate_robust_tf_avg(
            self, tf_list: list, yaw_list: list) -> TransformStamped:
        t = np.array([[tf.transform.translation.x,
                       tf.transform.translation.y] for tf in tf_list])
        med_t = np.median(t, axis=0)
        mad_t = np.median(np.linalg.norm(t - med_t, axis=1))
        inliers = np.linalg.norm(t - med_t, axis=1) < max(3 * mad_t, 0.5)
        if inliers.sum() < 3:
            inliers[:] = True
        final_t = t[inliers].mean(axis=0)

        avg_yaw = self._circular_mean_yaw(yaw_list)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = float(final_t[0])
        tf_msg.transform.translation.y = float(final_t[1])
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quat(avg_yaw)

        self.get_logger().info(
            f"Robust init TF: t=({final_t[0]:.3f}, {final_t[1]:.3f}), "
            f"yaw={math.degrees(avg_yaw):.2f} deg")
        return tf_msg

    # ------------------------------------------------------------------
    # Main callback: /cube_markers
    # ------------------------------------------------------------------
    def cube_callback(self, msg: ArucoMarkers):
        now = self.get_clock().now()
        dt_since_last = (now - self.last_callback_time).nanoseconds / 1e9
        if dt_since_last < self.callback_period_limit and self.initialized_map_odom_tf:
            return
        self.last_callback_time = now

        self._update_curr_map_base()

        # --- validate markers ---
        valid_markers = []
        for k, (idx, pose) in enumerate(zip(msg.marker_ids, msg.poses)):
            if idx < 0 or idx >= len(self.landmark_poses):
                continue
            lm_x, lm_y = self.landmark_poses[idx]
            if abs(lm_x) < self.MAP_SIZE and abs(lm_y) < self.MAP_SIZE:
                valid_markers.append((idx, pose, k))

        n = len(valid_markers)
        if n < 1:
            return

        is_measurement_valid = False
        transform_msg = None

        # ==============================================================
        # INITIALIZATION PHASE
        # ==============================================================
        if not self.initialized_map_odom_tf:
            if n == 1:
                # --- yaw-only from known start position ---
                (iA, _, kA) = valid_markers[0]
                A = self.landmark_poses[iA]
                phiA = math.radians(msg.ar_angles_list[kA])
                x0, y0 = self.erc_start_pos
                yawA = wrap(math.atan2(A[1] - y0, A[0] - x0) - phiA)
                self.yaw_estimate = yawA
                self.x_estimate = x0
                self.y_estimate = y0
                self.measured_new_yaw = True
                self.time_of_last_yaw_meas = now

                self.get_logger().info(
                    f"[INIT n=1] yaw = {math.degrees(self.yaw_estimate):.2f} deg")

                T_map_base = self._pose_to_mat(x0, y0, self.yaw_estimate)
                T_odom_base = np.eye(4)
                T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
                transform_msg = TransformStamped()
                transform_msg.header.stamp = now.to_msg()
                transform_msg.header.frame_id = 'map'
                transform_msg.child_frame_id = 'odom'
                transform_msg.transform.translation.x = T_map_odom[0, 3]
                transform_msg.transform.translation.y = T_map_odom[1, 3]
                transform_msg.transform.rotation = yaw_to_quat(
                    math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
                is_measurement_valid = True

            elif n == 2:
                # --- yaw-only from known start position ---
                (iA, _, kA), (iB, _, kB) = valid_markers
                A = self.landmark_poses[iA]
                B = self.landmark_poses[iB]
                phiA = math.radians(msg.ar_angles_list[kA])
                phiB = math.radians(msg.ar_angles_list[kB])
                x0, y0 = self.erc_start_pos

                yawA = wrap(math.atan2(A[1] - y0, A[0] - x0) - phiA)
                yawB = wrap(math.atan2(B[1] - y0, B[0] - x0) - phiB)
                self.yaw_estimate = wrap(0.5 * (yawA + yawB))
                self.x_estimate = x0
                self.y_estimate = y0
                self.measured_new_yaw = True
                self.time_of_last_yaw_meas = now

                self.get_logger().info(
                    f"[INIT n=2] yaw = {math.degrees(self.yaw_estimate):.2f} deg")

                T_map_base = self._pose_to_mat(x0, y0, self.yaw_estimate)
                T_odom_base = np.eye(4)
                T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
                transform_msg = TransformStamped()
                transform_msg.header.stamp = now.to_msg()
                transform_msg.header.frame_id = 'map'
                transform_msg.child_frame_id = 'odom'
                transform_msg.transform.translation.x = T_map_odom[0, 3]
                transform_msg.transform.translation.y = T_map_odom[1, 3]
                transform_msg.transform.rotation = yaw_to_quat(
                    math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
                is_measurement_valid = True

            elif n >= 3:
                # --- convex solver for full (x, y) ---
                measurements = self._build_measurements(valid_markers, msg)
                xy = self.solve_cvx(measurements)

                if xy is not None:
                    dx = xy[0] - self.erc_start_pos[0]
                    dy = xy[1] - self.erc_start_pos[1]
                    if math.hypot(dx, dy) < 1.0:
                        self.x_estimate, self.y_estimate = xy
                        self.solved_new_xy = True
                        self.time_of_last_pose = now

                        self.yaw_estimate = self._deduce_yaw(
                            xy[0], xy[1], valid_markers, msg)
                        self.measured_new_yaw = True
                        self.time_of_last_yaw_meas = now

                        self.get_logger().info(
                            f"[INIT n>={n}] P=({xy[0]:.3f}, {xy[1]:.3f}), "
                            f"yaw={math.degrees(self.yaw_estimate):.2f} deg")

                        T_map_base = self._pose_to_mat(
                            xy[0], xy[1], self.yaw_estimate)
                        T_odom_base = np.eye(4)
                        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
                        transform_msg = TransformStamped()
                        transform_msg.header.stamp = now.to_msg()
                        transform_msg.header.frame_id = 'map'
                        transform_msg.child_frame_id = 'odom'
                        transform_msg.transform.translation.x = T_map_odom[0, 3]
                        transform_msg.transform.translation.y = T_map_odom[1, 3]
                        transform_msg.transform.rotation = yaw_to_quat(
                            math.atan2(T_map_odom[1, 0], T_map_odom[0, 0]))
                        is_measurement_valid = True
                    else:
                        self.get_logger().warn(
                            f"[INIT] Rejected: solution {xy} too far from start "
                            f"({math.hypot(dx, dy):.2f} m)")

        # ==============================================================
        # POST-INIT CONTINUOUS UPDATES
        # ==============================================================
        else:
            if n >= 3:
                measurements = self._build_measurements(valid_markers, msg)
                xy = self.solve_cvx(measurements)

                if xy is not None:
                    dx = xy[0] - self.curr_map_base_x
                    dy = xy[1] - self.curr_map_base_y
                    jump = math.hypot(dx, dy)

                    if jump <= self.max_translation_jump:
                        self.x_estimate, self.y_estimate = xy
                        self.solved_new_xy = True
                        self.time_of_last_pose = now

                        dt_yaw = (now - self.time_of_last_yaw_meas).nanoseconds * 1e-9
                        if dt_yaw >= self.min_yaw_dt:
                            self.yaw_estimate = self._deduce_yaw(
                                xy[0], xy[1], valid_markers, msg)
                            self.measured_new_yaw = True
                            self.time_of_last_yaw_meas = now
                            self.get_logger().info(
                                f"[UPDATE] yaw = "
                                f"{math.degrees(self.yaw_estimate):.2f} deg")
                    else:
                        self.get_logger().warn(
                            f"[UPDATE] Rejected jump: {jump:.2f} m")

            # Compute and store new map->odom TF
            if self.solved_new_xy:
                transform_msg = self._build_map_odom_tf(
                    self.x_estimate, self.y_estimate, self.yaw_estimate)
                self.prev_map_odom_tf = transform_msg
                is_measurement_valid = True

            # Broadcast current TF
            if self.prev_map_odom_tf is not None:
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
                if self.solved_new_xy:
                    self.get_logger().info(
                        f"[UPDATE] x={self.x_estimate:.3f}, "
                        f"y={self.y_estimate:.3f}, "
                        f"yaw={math.degrees(self.yaw_estimate):.2f} deg")

        # ==============================================================
        # INITIALIZATION ACCUMULATION
        # ==============================================================
        if (self.init_callback_counter < self.nbr_init_callbacks_for_avg
                and is_measurement_valid and transform_msg is not None):

            self.init_callback_counter += 1
            self.avg_initialization_tfs.append(transform_msg)
            self.yaw_init_list.append(self.yaw_estimate)
            self.get_logger().info(
                f"[ACCUM {self.init_callback_counter}/"
                f"{self.nbr_init_callbacks_for_avg}] "
                f"yaw={math.degrees(self.yaw_estimate):.2f} deg")

            if self.init_callback_counter == self.nbr_init_callbacks_for_avg:
                self.initialized_map_odom_tf = True
                self.prev_map_odom_tf = self._calculate_robust_tf_avg(
                    self.avg_initialization_tfs, self.yaw_init_list)
                self.tf_broadcaster.sendTransform(self.prev_map_odom_tf)
                self.get_logger().info(
                    "INITIALIZED map->odom TF. "
                    f"Rate-limiting to {1/self.callback_period_limit:.1f} Hz")

        # Publish odometry whenever we have a valid measurement
        if is_measurement_valid:
            self._publish_odom(
                self.x_estimate, self.y_estimate, self.yaw_estimate)

        # Reset per-callback flags
        self.solved_new_xy = False
        self.measured_new_yaw = False

    # ------------------------------------------------------------------
    # Build measurement list for the convex solver
    # ------------------------------------------------------------------
    def _build_measurements(self, valid_markers, msg) -> list:
        measurements = []
        for (idx, pose, k) in valid_markers:
            lm_x, lm_y = self.landmark_poses[idx]
            px, py = pose.position.x, pose.position.y
            r = math.hypot(px, py)
            if r < 1e-3:
                continue
            bearing_rad = math.radians(msg.ar_angles_list[k])
            vx = -math.cos(bearing_rad)
            vy = -math.sin(bearing_rad)
            measurements.append({
                "ax": lm_x, "ay": lm_y,
                "range": r,
                "vx": vx, "vy": vy,
            })
        return measurements


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorLidarNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
