#!/usr/bin/env python3
"""Autonomously estimate LiDAR yaw and pitch from one straight Nav2 drive."""

import math
import os
import re
import tempfile
from enum import Enum, auto
from statistics import fmean
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class State(Enum):
    COLLECT_INITIAL = auto()
    WAIT_FOR_NAV2 = auto()
    NAVIGATING = auto()
    SETTLING = auto()
    COLLECT_FINAL = auto()
    COMPLETE = auto()
    FAILED = auto()


class LidarCalibrator(Node):
    """Drive forward, average GLIM odometry, then persist pitch/yaw calibration."""

    def __init__(self) -> None:
        super().__init__('lidar_calibrator')

        self.declare_parameter('odom_topic', '/odom_glim_repub')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('travel_distance_m', 3.0)
        self.declare_parameter('sample_count', 50)
        self.declare_parameter('settle_time_sec', 2.0)
        self.declare_parameter('nav2_wait_timeout_sec', 120.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('min_forward_displacement_m', 2.4)
        self.declare_parameter(
            'calibration_file',
            '/home/xplore/dev_ws/src/sensors/lidar/config/lidar_calibration.yaml',
        )

        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_frame_id = self.get_parameter('goal_frame_id').value
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.travel_distance_m = float(self.get_parameter('travel_distance_m').value)
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.nav2_wait_timeout_sec = float(self.get_parameter('nav2_wait_timeout_sec').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.min_forward_displacement_m = float(
            self.get_parameter('min_forward_displacement_m').value)
        self.calibration_file = self.get_parameter('calibration_file').value

        if self.sample_count < 2:
            raise ValueError('sample_count must be at least 2')
        if self.travel_distance_m <= 0.0:
            raise ValueError('travel_distance_m must be positive')
        if self.min_forward_displacement_m <= 0.0:
            raise ValueError('min_forward_displacement_m must be positive')
        if not os.path.isfile(self.calibration_file):
            raise FileNotFoundError(
                f'Calibration file does not exist: {self.calibration_file}')

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_action').value)

        self.state = State.COLLECT_INITIAL
        self.initial_samples: List[Tuple[float, float, float]] = []
        self.final_samples: List[Tuple[float, float, float]] = []
        self.initial_mean: Optional[Tuple[float, float, float]] = None
        self.state_started_ns = self.get_clock().now().nanoseconds
        self.goal_started_ns: Optional[int] = None
        self.goal_handle = None
        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'Waiting for {self.sample_count} initial samples on {self.odom_topic}; '
            f'will then send Nav2 goal ({self.goal_x:.2f}, {self.goal_y:.2f}) in {self.goal_frame_id}.')

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        sample = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in sample):
            self.get_logger().warn('Ignoring odometry sample with non-finite position.')
            return

        if self.state == State.COLLECT_INITIAL:
            self.initial_samples.append(sample)
            if len(self.initial_samples) == self.sample_count:
                self.initial_mean = self._mean(self.initial_samples)
                self._set_state(State.WAIT_FOR_NAV2)
                self.get_logger().info(
                    'Initial GLIM position averaged: '
                    f'x={self.initial_mean[0]:.4f}, y={self.initial_mean[1]:.4f}, '
                    f'z={self.initial_mean[2]:.4f}. Waiting for Nav2.')
        elif self.state == State.COLLECT_FINAL:
            self.final_samples.append(sample)
            if len(self.final_samples) == self.sample_count:
                self._finish_calibration()

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if self.state == State.WAIT_FOR_NAV2:
            if self.nav_client.wait_for_server(timeout_sec=0.0):
                self._send_goal()
            elif self._elapsed_sec(now_ns) > self.nav2_wait_timeout_sec:
                self._fail('Timed out waiting for the Nav2 NavigateToPose action server.')
        elif self.state == State.NAVIGATING:
            if self.goal_started_ns is not None and (
                    now_ns - self.goal_started_ns) / 1e9 > self.goal_timeout_sec:
                self._fail('Timed out waiting for the rover to reach the calibration goal.')
        elif self.state == State.SETTLING and self._elapsed_sec(now_ns) >= self.settle_time_sec:
            self.final_samples.clear()
            self._set_state(State.COLLECT_FINAL)
            self.get_logger().info(
                f'Collecting {self.sample_count} final GLIM odometry samples.')

    def _send_goal(self) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.goal_frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.w = 1.0

        self._set_state(State.NAVIGATING)
        self.goal_started_ns = self.get_clock().now().nanoseconds
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            f'Sent calibration goal: {self.goal_frame_id} '
            f'x={self.goal_x:.2f}, y={self.goal_y:.2f}.')

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # ROS action errors carry implementation-specific types.
            self._fail(f'Nav2 rejected the calibration goal: {error}')
            return
        if not goal_handle.accepted:
            self._fail('Nav2 did not accept the calibration goal.')
            return
        if self.state != State.NAVIGATING:
            # The caller timed out while Nav2 was deciding whether to accept.
            goal_handle.cancel_goal_async()
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as error:
            self._fail(f'Failed while executing the calibration goal: {error}')
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Calibration goal did not succeed (Nav2 status {result.status}).')
            return
        self._set_state(State.SETTLING)
        self.get_logger().info(
            f'Nav2 reached the goal. Settling for {self.settle_time_sec:.1f} seconds.')

    def _finish_calibration(self) -> None:
        if self.initial_mean is None:
            self._fail('Internal error: missing initial odometry average.')
            return

        final_mean = self._mean(self.final_samples)
        dx = final_mean[0] - self.initial_mean[0]
        dy = final_mean[1] - self.initial_mean[1]
        dz = final_mean[2] - self.initial_mean[2]

        if dx < self.min_forward_displacement_m:
            self._fail(
                f'GLIM only measured {dx:.3f} m forward; expected about '
                f'{self.travel_distance_m:.3f} m. Calibration was not written.')
            return

        delta_yaw = math.atan2(dy, self.travel_distance_m)
        delta_pitch = math.atan2(dz, self.travel_distance_m)
        try:
            self._write_calibration(delta_yaw, delta_pitch)
        except Exception as error:
            self._fail(f'Unable to write calibration file: {error}')
            return
        self.get_logger().info(
            'Calibration complete from mean GLIM displacement '
            f'(x={dx:.5f}, y={dy:.5f}, z={dz:.5f}) m: '
            f'delta_yaw={delta_yaw:.8f} rad, delta_pitch={delta_pitch:.8f} rad.')

        self._set_state(State.COMPLETE)

    def _write_calibration(self, delta_yaw: float, delta_pitch: float) -> None:
        with open(self.calibration_file, 'r', encoding='utf-8') as stream:
            content = stream.read()

        updates = {
            'delta_yaw': f'{delta_yaw:.10f}',
            'delta_pitch': f'{delta_pitch:.10f}',
        }
        for key, value in updates.items():
            pattern = re.compile(rf'^(\s*{re.escape(key)}\s*:\s*)([^#\n]*)(.*)$', re.MULTILINE)
            content, replacements = pattern.subn(rf'\g<1>{value}\g<3>', content, count=1)
            if replacements != 1:
                raise RuntimeError(
                    f'Expected exactly one "{key}" entry in {self.calibration_file}.')

        directory = os.path.dirname(self.calibration_file) or '.'
        original_mode = os.stat(self.calibration_file).st_mode
        descriptor, temporary_path = tempfile.mkstemp(
            prefix='.lidar_calibration.', suffix='.yaml', dir=directory, text=True)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary_path, original_mode)
            os.replace(temporary_path, self.calibration_file)
        except Exception:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)
            raise

    @staticmethod
    def _mean(samples: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        return tuple(fmean(sample[index] for sample in samples) for index in range(3))

    def _set_state(self, state: State) -> None:
        self.state = state
        self.state_started_ns = self.get_clock().now().nanoseconds

    def _elapsed_sec(self, now_ns: int) -> float:
        return (now_ns - self.state_started_ns) / 1e9

    def _fail(self, message: str) -> None:
        if self.state in (State.COMPLETE, State.FAILED):
            return
        if self.state == State.NAVIGATING and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().warn('Cancelled the active calibration navigation goal.')
        self._set_state(State.FAILED)
        self.get_logger().error(message)


def main() -> None:
    rclpy.init()
    node = LidarCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
