# ERC_NAV — Xplore Navigation Workspace 2026

ROS 2 Humble workspace for the **EPFL Xplore** rover's autonomous navigation stack,
for the European Rover Challenge.

- **TL:** Paul Bourgois
- **SE:** Arno Laurie
- **Documentation (Notion):** https://www.notion.so/xplore-doc/Navigation-128d2c11ab574c95a9be0790afa76d48

---

## Overview

The stack takes readings from the rover's sensors (LiDAR, IMU, wheel encoders,
cameras), fuses them into a single `odom → base_link` transform, builds a
local traversability costmap, and lets Nav2 plan and follow a path that the
wheel controller turns into individual motor commands.

---

## Repository layout

| Folder | Contents |
|---|---|
| `custom_msg/` | Motor status / command ROS messages shared across the stack. |
| `interfacing_nav_cs/` | Interface node with the Command Station. |
| `jetson_stats/` | Publishes Jetson telemetry. |
| `localization/camera/` | ArUco tag stack: `ros2_aruco_cpp` (multi-view detector), `ros2_aruco_with_lidar` (LiDAR-assisted refinement), `ros2_aruco_interfaces` (messages), `nav_global_loc_convex` (global localization). |
| `localization/lidar/` | `glim_starter` (GLIM LiDAR SLAM wrapper) and `liorf` (alternative LiDAR-inertial odometry). |
| `localization/nav_ekf/local_nav_ekf/` | Custom EKF fusing wheel odometry, IMU, ArUco, LiDAR, VIO. Builds two executables: `nav_ekf_node` (2D) and `nav_ekf_3d_node` (3D, with RK4 prediction + Joseph-form updates). |
| `localization/odom_preprocessor/` | Small helpers to re-publish/align odometry streams. |
| `localization/wheels_odometry/` | Kinematic wheel odometry math kept as a pure-CMake Eigen lib. |
| `navigation/gradient_costmap/` | `traversability_filter`, `traversability_map`, and the Nav2 costmap `gradient_layer_plugin`. |
| `navigation/path_planning/` | Nav2 launch files + `config/nav2_params_real_2026_no_global_map.yaml` tuned for ERC 2026. |
| `sensors/camera/ERC_CAMERAS/` | Camera driver and per-camera launch files (nav / high-def / command-station). |
| `sensors/imu/olive_imu_restamper/` | Restamps the Olive IMU to current ROS time. Ships calibration scripts. |
| `sensors/imu/ouster_imu_tester/` | Diagnostic tool for the Ouster built-in IMU. |
| `sensors/lidar/ros2_ouster_drivers/` | Vendored Ouster driver (OS-0 / OS-1). |
| `wheels_control/` | Gamepad and CS teleop, cmd_vel manager, displacement commands, motor command bridge, steering slip controller, wheel odometry node. |
| `xplore_description/` | URDF + robot state publisher launches (real and sim). |
| `docker_humble_*/` | Reference Docker images for desktop, GLIM, and Jetson targets. |

---

## Main launch entry points

All commands assume the workspace has been sourced:

```bash
source install/setup.bash
```

### Manual driving + sensing + localization

```bash
ros2 launch wheels_control manual_stack.launch.py
```

Starts, in order:

- Command-station interface, gamepad teleop, `cmd_vel` manager, displacement commands,
  motor command bridge (`NAV_motor_cmds`), and the steering slip controller.
- `xplore_description` URDF / robot state publisher (`pub_urdf:=true` by default).
- Olive IMU restamper.
- Wheel odometry node.
- `local_nav_ekf/nav_ekf_node` with `include_lidar:=true`, `include_aruco:=false`,
  `include_vio:=false`.
- Ouster LiDAR driver (`ros2_ouster`).
- NAV cameras (`camera/camera_node_nav.launch.py`).
- 7 s later: `ros2_aruco_cpp` multi-view ArUco detector and
  `ros2_aruco_with_lidar` (`lidar_phi_filter_node` + `detect_cube`) for
  LiDAR-refined cube centroids.
- Jetson stats publisher.

Useful launch args:

- `motor_cmds` (default `true`) — skip motor bridge when bench-testing.
- `homing` (default `false`) — run the homing sequence on startup.
- `pub_urdf` (default `true`) — publish URDF + launch the Ouster driver.

### Full autonomous stack (manual stack + costmap + Nav2)

```bash
ros2 launch path_planning autonomous_stack.launch.py
```

This wraps `manual_stack.launch.py` and then:

- At `t = 30 s`: `gradient_costmap.launch.py` in `local` mode, which spawns
  `traversability_filter` (point cloud transform + height map),
  `traversability_map` (local occupancy + cost), and the
  `gradient_layer_plugin` consumed by Nav2 costmaps.
- At `t = 35 s`: the full Nav2 stack (`controller_server`, `planner_server`,
  `bt_navigator`, `behavior_server`, `waypoint_follower`,
  `velocity_smoother`, `smoother_server`, `map_server`, `lifecycle_manager`)
  configured from
  `navigation/path_planning/config/nav2_params_real_2026_no_global_map.yaml`.
  Nav2 publishes to `cmd_vel_nav`, which the wheel controller consumes.

The delays are intentional: the Ouster TF tree typically settles around
28–30 s after boot.

### Other useful launch files

- `navigation/path_planning/launch/nav2_real.launch.py` — Nav2 only (no manual stack).
- `navigation/path_planning/launch/localization_launch.py` — Nav2 localization subset.
- `navigation/path_planning/launch/liorf_testing.launch.py` — LIORF bring-up for SLAM experiments.
- `localization/lidar/glim_starter/launch/glim_starter.launch.py` — GLIM LiDAR SLAM.
- `sensors/lidar/ros2_ouster_drivers/ros2_ouster/launch/driver_launch.py` — Ouster driver alone.
- `xplore_description/launch/xplore_sim.launch.py` — URDF in simulation.

---

## Key components

### `local_nav_ekf`

Custom Extended Kalman Filter with two variants:

- `nav_ekf_node` — 2D filter, state `[x, y, yaw, vx, vy]`. The default used by
  `manual_stack.launch.py`.
- `nav_ekf_3d_node` — 3D filter, state `[x, y, z, roll, pitch, yaw, vx, vy, vz]`.
  Mean prediction uses RK4 integration of the full nonlinear kinematics;
  covariance prediction uses a RK2-discretized constant-velocity Jacobian;
  posterior covariance updates use the Joseph form with explicit
  symmetrization. Roll and pitch are IIR low-passed before the IMU update to
  reject chassis vibration. Publishes `odom → base_link` with full 3D
  translation and rotation so downstream nodes (traversability filter,
  gradient costmap) see rover pitch/roll.

Parameters (3D node): `include_lidar`, `include_aruco`, `include_vio`,
`roll_pitch_lowpass_tau`, `mahalanobis_gate_squared`. Output:
`/fused_nav_ekf_odom`.

### ArUco + LiDAR cube detection (`ros2_aruco_with_lidar`)

Two-stage pipeline used to refine camera ArUco detections with the LiDAR:

1. `lidar_phi_filter_node` — spatial pre-filter. Associates every valid
   camera detection (outside a forbidden angular sector) with a hard-coded
   map landmark and keeps only LiDAR points that fall inside vertical
   cylinders around the associated landmarks.
2. `detect_cube` — runs sequential 3D RANSAC line fits on the pre-filtered
   cloud, projects inliers to 2D, re-RANSACs, and builds candidate cube
   centroids using the known cube side length. The radial gate uses the
   expected range from `map_frame` to the sensor (not the noisy camera
   range) to stay consistent with the map.

See `localization/camera/ros2_aruco_with_lidar/docs/LIDAR_ARUCO_NODES.md` for
the detailed math and parameter reference.

### Gradient costmap (`navigation/gradient_costmap`)

- `traversability_filter` transforms the raw Ouster cloud into a fixed frame
  (using the TF tree, so it honours rover roll/pitch when those are
  published) and builds a local height map.
- `traversability_map` fits local planes to cells to produce a
  slope/roughness occupancy grid.
- `gradient_layer_plugin` exposes that grid as a Nav2 costmap layer (read
  from topic or from a PGM/YAML pair in `global` mode).

### Wheel control (`wheels_control`)

Implements the Xplore-specific kinematics: independent steering per wheel
with Ackermann, in-place rotation, and straight-drive cases. Provides the
manual/teleop path used outside of Nav2 and the motor bridge for all modes.

---

## Build

```bash
# First-time build (fetches dependencies order)
colcon build --symlink-install

# Iterating on a single package
colcon build --packages-select local_nav_ekf --symlink-install

# Source and run
source install/setup.bash
```

Docker images for the desktop, Jetson, and GLIM-enabled targets live under
`docker_humble_desktop/`, `docker_humble_jetson/`, and `docker_humble_glim/`.

---

## LiDAR (Ouster) ethernet setup

The Ouster needs a static IPv4 on whatever interface it is plugged into,
plus a small DHCP range for the sensor to pull its lease from.

### If the LiDAR is connected to `eth0`

```bash
sudo ip addr flush dev eth0
sudo ip link set eth0 down
sudo ip addr show dev eth0   # should report state DOWN
sudo ip addr add 10.5.5.1/24 dev eth0
sudo ip link set eth0 up
sudo ip addr show eth0
sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eth0 --bind-dynamic
```

### If the LiDAR is connected to `eth1`

```bash
sudo ip addr flush dev eth1
sudo ip link set eth1 down
sudo ip addr show dev eth1   # should report state DOWN
sudo ip addr add 10.5.5.1/24 dev eth1
sudo ip link set eth1 up
sudo ip addr show eth1
sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eth1 --bind-dynamic
```

### Quick sanity checks

Web UI: http://os-122140001125.local

```bash
nmcli connection show
avahi-browse -arlt
ping 10.5.5.87
ping 10.5.5.1
pinglidar            # alias for: ping os-122140001125.local
nc os-122140001125.local
```

### Manual static IPv4 (if all else fails)

```bash
sudo ip addr add 10.5.5.1/24 dev <iface> mtu 1500
```
