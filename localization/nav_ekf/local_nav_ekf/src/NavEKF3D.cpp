// 3D Extended Kalman Filter for Navigation
// State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz]  (position in world,
//               orientation as ZYX Euler angles, linear velocity in world)
//
// Prediction:
//   - Mean propagation via RK4 integration of the full nonlinear kinematics
//     (position integrates world-frame velocity, Euler angles integrate the
//     body-frame gyro rate through the Euler-rate matrix).
//   - Covariance propagation uses the RK2-discretized Jacobian of a
//     constant-velocity motion model:
//         F = I + dt * A + (dt^2 / 2) * A * A
//     where A is the continuous-time CV Jacobian coupling position to
//     velocity (A * A collapses to zero for this model, but the expression
//     is kept symbolic so future extensions retain RK2 semantics).
//
// Measurement updates fuse:
//   - IMU orientation (roll/pitch/yaw). Roll and pitch are low-pass filtered
//     before the update to attenuate high-frequency chassis vibration.
//   - Wheel odometry position (x, y) and wheel-derived world velocity
//     (vx, vy, vz), rotated from body frame using the current RPY estimate.
//   - LiDAR, ArUco and VIO 2D position measurements (with Mahalanobis
//     gating).
//
// Author: Arno Laurie

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "custom_msg/msg/motor_status.hpp"


namespace {

// ---------- State layout ----------
constexpr int STATE_DIM       = 9;
constexpr int IDX_X           = 0;
constexpr int IDX_Y           = 1;
constexpr int IDX_Z           = 2;
constexpr int IDX_ROLL        = 3;
constexpr int IDX_PITCH       = 4;
constexpr int IDX_YAW         = 5;
constexpr int IDX_VX          = 6;
constexpr int IDX_VY          = 7;
constexpr int IDX_VZ          = 8;

// nav_msgs/Odometry pose covariance row-major offsets (6x6 block)
constexpr int COV_POSE_XX     = 0;
constexpr int COV_POSE_YY     = 7;
constexpr int COV_POSE_ZZ     = 14;
constexpr int COV_POSE_ROLL   = 21;
constexpr int COV_POSE_PITCH  = 28;
constexpr int COV_POSE_YAW    = 35;

// ---------- Chassis geometry (kept from the 2D version) ----------
constexpr int    STEERING_RESOLUTION_BITS             = 14;
constexpr double CHASSIS_WIDTH_M                      = 0.75;
constexpr double CHASSIS_LENGTH_M                     = 0.87;
constexpr double WHEEL_RADIUS_M                       = 0.12;
constexpr double GEAR_RATIO                           = 1.0 / 53.0;
const     double RPM_TO_MS                            = (2.0 * M_PI * WHEEL_RADIUS_M) / 60.0;
const     double INCR_TO_RAD                          = 2.0 * M_PI / std::pow(2.0, STEERING_RESOLUTION_BITS);
const     double DEG_TO_RAD                           = M_PI / 180.0;
const     double STRAIGHT_ANGLE_THRESHOLD_RAD         = 0.8 * DEG_TO_RAD;
const     double IN_PLACE_ROTATION_ANGLE_THRESHOLD_RAD = 0.642; // max mean Ackermann angle ≈ 0.639


inline double normalize_angle(double angle) {
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// ZYX (yaw * pitch * roll) rotation matrix built from intrinsic Euler angles.
inline Eigen::Matrix3d rotation_matrix_from_rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll),  sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw),   sy = std::sin(yaw);

  Eigen::Matrix3d rotation_world_from_body;
  rotation_world_from_body <<
      cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
      sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
         -sp,                 cp * sr,                 cp * cr;
  return rotation_world_from_body;
}

// Euler-rate (kinematic) matrix E(roll, pitch) such that
//   [roll_dot; pitch_dot; yaw_dot] = E * [wx_body; wy_body; wz_body]
// Guards against the pitch = ±pi/2 gimbal-lock singularity.
inline Eigen::Matrix3d euler_rate_matrix(double roll, double pitch) {
  const double cr = std::cos(roll),  sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double safe_cp = (std::abs(cp) < 1e-6) ? ((cp >= 0.0) ? 1e-6 : -1e-6) : cp;
  const double tp = sp / safe_cp;

  Eigen::Matrix3d euler_rate_from_body;
  euler_rate_from_body <<
      1.0, sr * tp,        cr * tp,
      0.0, cr,             -sr,
      0.0, sr / safe_cp,   cr / safe_cp;
  return euler_rate_from_body;
}

// Simple first-order IIR low-pass filter (exponential smoothing)
//   y_k = alpha * u_k + (1 - alpha) * y_{k-1}
// with alpha = dt / (tau + dt).  Using dt keeps the cutoff frequency
// independent of the IMU publication rate.
inline double low_pass_step(double raw, double previous, double dt, double time_constant) {
  if (!(dt > 0.0) || !(time_constant > 0.0)) return raw;
  const double alpha = dt / (time_constant + dt);
  return alpha * raw + (1.0 - alpha) * previous;
}

}  // namespace


// =====================================================================
// ExtendedKalmanFilter3D
// =====================================================================
class ExtendedKalmanFilter3D {
public:
  using StateVector   = Eigen::Matrix<double, STATE_DIM, 1>;
  using StateMatrix   = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

  StateVector state;              // [x y z roll pitch yaw vx vy vz]
  StateMatrix covariance;
  StateMatrix process_noise;      // continuous-time Q; integrated as Q*dt

  Eigen::Matrix3d R_orientation_rpy;    // IMU RPY variance (roll, pitch, yaw)
  Eigen::Matrix2d R_position_xy_wheel;
  Eigen::Matrix2d R_position_xy_lidar;
  Eigen::Matrix2d R_position_xy_aruco;
  Eigen::Matrix2d R_position_xy_vio;
  Eigen::Matrix3d R_velocity_world_xyz;

  ExtendedKalmanFilter3D() {
    state.setZero();
    covariance = StateMatrix::Identity() * 1e-2;

    process_noise.setZero();
    process_noise(IDX_X,     IDX_X)     = 0.01;
    process_noise(IDX_Y,     IDX_Y)     = 0.01;
    process_noise(IDX_Z,     IDX_Z)     = 0.01;
    process_noise(IDX_ROLL,  IDX_ROLL)  = 0.005;
    process_noise(IDX_PITCH, IDX_PITCH) = 0.005;
    process_noise(IDX_YAW,   IDX_YAW)   = 0.01;
    process_noise(IDX_VX,    IDX_VX)    = 0.10;
    process_noise(IDX_VY,    IDX_VY)    = 0.20;
    process_noise(IDX_VZ,    IDX_VZ)    = 0.10;

    R_orientation_rpy    = Eigen::Matrix3d::Identity() * 1e-4;   // std 0.01 rad
    R_position_xy_wheel  = Eigen::Matrix2d::Identity() * 0.08;
    R_position_xy_lidar  = Eigen::Matrix2d::Identity() * 0.01;   // std 0.10 m
    R_position_xy_aruco  = Eigen::Matrix2d::Identity() * 0.0025; // std 0.05 m
    R_position_xy_vio    = Eigen::Matrix2d::Identity() * 0.0025; // std 0.05 m
    R_velocity_world_xyz = Eigen::Matrix3d::Identity() * 0.10;
  }

  void initialize(double x0, double y0, double z0,
                  double roll0, double pitch0, double yaw0) {
    state.setZero();
    state(IDX_X)     = x0;
    state(IDX_Y)     = y0;
    state(IDX_Z)     = z0;
    state(IDX_ROLL)  = roll0;
    state(IDX_PITCH) = pitch0;
    state(IDX_YAW)   = yaw0;
  }

  // -------------------------------------------------------------------
  // Continuous-time state derivative f(x, u).
  // Input u = body-frame angular rate (gyro).  World-frame linear velocity
  // lives in the state vector.
  // -------------------------------------------------------------------
  StateVector state_derivative(const StateVector& s,
                               const Eigen::Vector3d& body_angular_rate) const
  {
    StateVector deriv = StateVector::Zero();

    deriv(IDX_X) = s(IDX_VX);
    deriv(IDX_Y) = s(IDX_VY);
    deriv(IDX_Z) = s(IDX_VZ);

    const Eigen::Vector3d rpy_rate =
        euler_rate_matrix(s(IDX_ROLL), s(IDX_PITCH)) * body_angular_rate;
    deriv(IDX_ROLL)  = rpy_rate(0);
    deriv(IDX_PITCH) = rpy_rate(1);
    deriv(IDX_YAW)   = rpy_rate(2);

    // Constant-velocity model: velocity derivatives are zero.
    return deriv;
  }

  // Classical fourth-order Runge-Kutta integration of the nonlinear model.
  StateVector rk4_integrate(const StateVector& s,
                            const Eigen::Vector3d& body_angular_rate,
                            double dt) const
  {
    const StateVector k1 = state_derivative(s, body_angular_rate);
    const StateVector k2 = state_derivative(s + (0.5 * dt) * k1, body_angular_rate);
    const StateVector k3 = state_derivative(s + (0.5 * dt) * k2, body_angular_rate);
    const StateVector k4 = state_derivative(s + dt * k3,         body_angular_rate);
    return s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  // -------------------------------------------------------------------
  // Prediction step
  //   - Mean: RK4 integration (high accuracy, nonlinear).
  //   - Covariance: RK2 (midpoint) discretization of the constant-velocity
  //     Jacobian:  F = I + dt*A + (dt^2 / 2) * A*A
  // -------------------------------------------------------------------
  void predict(double dt, const Eigen::Vector3d& body_angular_rate) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;

    // --- Mean propagation (RK4) ---
    state = rk4_integrate(state, body_angular_rate, dt);
    state(IDX_ROLL)  = normalize_angle(state(IDX_ROLL));
    state(IDX_PITCH) = normalize_angle(state(IDX_PITCH));
    state(IDX_YAW)   = normalize_angle(state(IDX_YAW));

    // --- Covariance propagation (RK2 Jacobian of CV model) ---
    StateMatrix jacobian_continuous = StateMatrix::Zero();
    jacobian_continuous(IDX_X, IDX_VX) = 1.0;
    jacobian_continuous(IDX_Y, IDX_VY) = 1.0;
    jacobian_continuous(IDX_Z, IDX_VZ) = 1.0;

    const StateMatrix jacobian_squared = jacobian_continuous * jacobian_continuous;
    const StateMatrix state_transition =
        StateMatrix::Identity()
        + dt * jacobian_continuous
        + (0.5 * dt * dt) * jacobian_squared;

    covariance = state_transition * covariance * state_transition.transpose()
                 + process_noise * dt;
  }

  // -------------------------------------------------------------------
  // Joseph-form posterior covariance update
  //   P+ = (I - K H) P (I - K H)^T + K R K^T
  // guarantees symmetry and positive semi-definiteness under numerical
  // noise and even if K departs from the optimal Kalman gain.  Slightly
  // more expensive than the standard form P+ = (I - K H) P but much
  // more robust.
  // -------------------------------------------------------------------
  template <int MEAS_DIM>
  void apply_joseph_form_update(
      const Eigen::Matrix<double, MEAS_DIM, STATE_DIM>& H,
      const Eigen::Matrix<double, STATE_DIM, MEAS_DIM>& K,
      const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R_cov)
  {
    const StateMatrix IKH = StateMatrix::Identity() - K * H;
    covariance = IKH * covariance * IKH.transpose() + K * R_cov * K.transpose();
    // Enforce exact symmetry against residual round-off.
    covariance = 0.5 * (covariance + covariance.transpose());
  }

  // -------------------------------------------------------------------
  // Measurement updates
  // -------------------------------------------------------------------
  void update_orientation_rpy(const Eigen::Vector3d& measured_rpy,
                              const Eigen::Matrix3d& R_cov)
  {
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, IDX_ROLL)  = 1.0;
    H(1, IDX_PITCH) = 1.0;
    H(2, IDX_YAW)   = 1.0;

    Eigen::Vector3d innovation;
    innovation(0) = normalize_angle(measured_rpy(0) - state(IDX_ROLL));
    innovation(1) = normalize_angle(measured_rpy(1) - state(IDX_PITCH));
    innovation(2) = normalize_angle(measured_rpy(2) - state(IDX_YAW));

    const Eigen::Matrix3d S = H * covariance * H.transpose() + R_cov;
    const Eigen::Matrix<double, STATE_DIM, 3> K =
        covariance * H.transpose() * S.inverse();

    state += K * innovation;
    state(IDX_ROLL)  = normalize_angle(state(IDX_ROLL));
    state(IDX_PITCH) = normalize_angle(state(IDX_PITCH));
    state(IDX_YAW)   = normalize_angle(state(IDX_YAW));

    apply_joseph_form_update<3>(H, K, R_cov);
  }

  void update_yaw_only(double measured_yaw, double measurement_variance) {
    Eigen::Matrix<double, 1, STATE_DIM> H = Eigen::Matrix<double, 1, STATE_DIM>::Zero();
    H(0, IDX_YAW) = 1.0;

    const double innovation = normalize_angle(measured_yaw - state(IDX_YAW));
    const double S = (H * covariance * H.transpose())(0, 0) + measurement_variance;
    const Eigen::Matrix<double, STATE_DIM, 1> K =
        covariance * H.transpose() / S;

    state += K * innovation;
    state(IDX_YAW) = normalize_angle(state(IDX_YAW));

    Eigen::Matrix<double, 1, 1> R_cov;
    R_cov(0, 0) = measurement_variance;
    apply_joseph_form_update<1>(H, K, R_cov);
  }

  void update_position_xy(double measured_x, double measured_y,
                          const Eigen::Matrix2d& R_cov)
  {
    Eigen::Matrix<double, 2, STATE_DIM> H = Eigen::Matrix<double, 2, STATE_DIM>::Zero();
    H(0, IDX_X) = 1.0;
    H(1, IDX_Y) = 1.0;

    const Eigen::Vector2d innovation(
        measured_x - state(IDX_X),
        measured_y - state(IDX_Y));

    const Eigen::Matrix2d S = H * covariance * H.transpose() + R_cov;
    const Eigen::Matrix<double, STATE_DIM, 2> K =
        covariance * H.transpose() * S.inverse();

    state += K * innovation;
    apply_joseph_form_update<2>(H, K, R_cov);
  }

  void update_position_xyz(double measured_x, double measured_y, double measured_z,
                           const Eigen::Matrix3d& R_cov)
  {
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, IDX_X) = 1.0;
    H(1, IDX_Y) = 1.0;
    H(2, IDX_Z) = 1.0;

    const Eigen::Vector3d innovation(
        measured_x - state(IDX_X),
        measured_y - state(IDX_Y),
        measured_z - state(IDX_Z));

    const Eigen::Matrix3d S = H * covariance * H.transpose() + R_cov;
    const Eigen::Matrix<double, STATE_DIM, 3> K =
        covariance * H.transpose() * S.inverse();

    state += K * innovation;
    apply_joseph_form_update<3>(H, K, R_cov);
  }

  void update_velocity_world_xyz(const Eigen::Vector3d& measured_v_world) {
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, IDX_VX) = 1.0;
    H(1, IDX_VY) = 1.0;
    H(2, IDX_VZ) = 1.0;

    const Eigen::Vector3d innovation(
        measured_v_world(0) - state(IDX_VX),
        measured_v_world(1) - state(IDX_VY),
        measured_v_world(2) - state(IDX_VZ));

    const Eigen::Matrix3d S = H * covariance * H.transpose() + R_velocity_world_xyz;
    const Eigen::Matrix<double, STATE_DIM, 3> K =
        covariance * H.transpose() * S.inverse();

    state += K * innovation;
    apply_joseph_form_update<3>(H, K, R_velocity_world_xyz);
  }

  // -------------------------------------------------------------------
  // Body-frame planar wheel odometry (unchanged kinematics from the 2D EKF).
  // Returns [vx_body, vy_body, omega_z_body].  The caller is responsible for
  // rotating the body velocity into the world frame using the current RPY.
  // -------------------------------------------------------------------
  std::vector<double> compute_wheel_odom_body(
      double steer_fr, double steer_br, double steer_fl, double steer_bl,
      double vel_fr,   double vel_br,   double vel_fl,   double vel_bl) const
  {
    const double avg_abs_angle =
        (std::fabs(steer_fl) + std::fabs(steer_fr) +
         std::fabs(steer_br) + std::fabs(steer_bl)) / 4.0;
    const double avg_speed = (vel_fl + vel_fr + vel_br + vel_bl) / 4.0;

    double vx_body = 0.0, vy_body = 0.0, omega_z = 0.0;
    bool going_right = false, going_left = false;

    const bool all_angles_small   = avg_abs_angle < STRAIGHT_ANGLE_THRESHOLD_RAD;
    const bool possible_rotation  = avg_abs_angle > IN_PLACE_ROTATION_ANGLE_THRESHOLD_RAD;

    if (all_angles_small) {
      vx_body = (vel_fl + vel_fr + vel_bl + vel_br) / 4.0;
    } else if (possible_rotation) {
      const double v_rot = (vel_fr - vel_fl + vel_br - vel_bl) / 4.0;
      const double r = std::sqrt((CHASSIS_LENGTH_M * 0.5) * (CHASSIS_LENGTH_M * 0.5)
                               + (CHASSIS_WIDTH_M  * 0.5) * (CHASSIS_WIDTH_M  * 0.5));
      omega_z = v_rot / r;
    } else {
      double alpha_ext = steer_fr;
      double alpha_int = -steer_fl;
      double v_ext = vel_fr;
      double v_int = vel_fl;

      if (steer_fl >= 0 && steer_fr >= 0 && steer_fr > steer_fl &&
          steer_bl <= 0 && steer_br <= 0 && steer_br < steer_bl) {
        going_right = true;
        v_ext = 0.5 * (vel_fl + vel_bl);
        v_int = 0.5 * (vel_fr + vel_br);
        alpha_int = 0.5 * (steer_fl - steer_bl);
        alpha_ext = 0.5 * (steer_fr - steer_br);
      } else if (steer_bl >= 0 && steer_br >= 0 && steer_br < steer_bl &&
                 steer_fl <= 0 && steer_fr <= 0 && steer_fr > steer_fl) {
        going_left = true;
        v_int = 0.5 * (vel_fl + vel_bl);
        v_ext = 0.5 * (vel_fr + vel_br);
        alpha_ext = 0.5 * (steer_fl - steer_bl);
        alpha_int = 0.5 * (steer_fr - steer_br);
      }

      if (std::abs(alpha_ext) > 0 && std::abs(alpha_int) > 0) {
        const double r_ext = std::sqrt(std::pow(CHASSIS_WIDTH_M / 2.0 + CHASSIS_LENGTH_M / (2.0 * std::tan(alpha_ext)), 2)
                                     + std::pow(CHASSIS_LENGTH_M / 2.0, 2));
        const double r_int = std::sqrt(std::pow(CHASSIS_WIDTH_M / 2.0 - CHASSIS_LENGTH_M / (2.0 * std::tan(alpha_int)), 2)
                                     + std::pow(CHASSIS_LENGTH_M / 2.0, 2));

        const double R_geo = 0.25 * CHASSIS_LENGTH_M * (1.0 / std::tan(alpha_ext) + 1.0 / std::tan(alpha_int));
        const double omega_ext = v_ext / r_ext;
        const double omega_int = v_int / r_int;
        omega_z = (omega_ext + omega_int) / 2.0;

        double safe_omega = omega_z;
        if (std::abs(safe_omega) < 1e-6) safe_omega = 1e-6;
        const double R_vel = (v_ext / safe_omega + v_int / safe_omega) / 2.0;
        const double R = 0.5 * (R_geo + R_vel);

        vx_body = omega_z * R;
        vy_body = 0.0;
      }
    }

    if (going_left)                               vx_body = -vx_body;
    if (going_left  && avg_speed >= 0.0)          omega_z = -omega_z;
    if (going_left  && avg_speed <  0.0)          omega_z = -omega_z;   // keep original sign logic
    if (going_right && avg_speed >= 0.0)          omega_z = -omega_z;

    return {vx_body, vy_body, omega_z};
  }
};


// =====================================================================
// NavEKF3DNode
// =====================================================================
class NavEKF3DNode : public rclcpp::Node {
public:
  NavEKF3DNode()
  : Node("nav_ekf_3d_node")
  {
    ekf_ = std::make_shared<ExtendedKalmanFilter3D>();
    ekf_->initialize(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // ---- Parameters ----
    declare_parameter<bool>("include_lidar", false);
    declare_parameter<bool>("include_aruco", false);
    declare_parameter<bool>("include_vio",   false);

    // Chassis-vibration low-pass on IMU roll/pitch (time-constant, seconds).
    // Larger => more smoothing.  Yaw is left unfiltered.
    declare_parameter<double>("roll_pitch_lowpass_tau", 0.15);

    // Mahalanobis gate squared (2 DoF chi^2 95% ≈ 5.99; 99% ≈ 9.21).
    declare_parameter<double>("mahalanobis_gate_squared", 7.0);

    include_lidar_ = get_parameter("include_lidar").as_bool();
    include_aruco_ = get_parameter("include_aruco").as_bool();
    include_vio_   = get_parameter("include_vio").as_bool();
    roll_pitch_lowpass_tau_ = get_parameter("roll_pitch_lowpass_tau").as_double();
    mahalanobis_gate_sq_    = get_parameter("mahalanobis_gate_squared").as_double();

    RCLCPP_INFO(get_logger(), "3D EKF | LiDAR: %s  ArUco: %s  VIO: %s  tau_rp=%.3fs",
                include_lidar_ ? "on" : "off",
                include_aruco_ ? "on" : "off",
                include_vio_   ? "on" : "off",
                roll_pitch_lowpass_tau_);

    // ---- IMU calibration services (same as 2D node) ----
    bias_client_      = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setBias");
    zero_quat_client_ = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setZeroQuaternion");
    zero_pose_client_ = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setZeroPose");
    wait_for_service(bias_client_);
    wait_for_service(zero_quat_client_);
    wait_for_service(zero_pose_client_);
    call_trigger(bias_client_,      "setBias");
    call_trigger(zero_quat_client_, "setZeroQuaternion");
    call_trigger(zero_pose_client_, "setZeroPose");

    auto sensor_qos = rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort();
    auto pub_qos    = rclcpp::QoS{rclcpp::KeepLast{1}}.reliable();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/olive/imu/id001/ahrs", sensor_qos,
        std::bind(&NavEKF3DNode::imu_callback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom_glim_repub", sensor_qos,
        std::bind(&NavEKF3DNode::lidar_callback, this, std::placeholders::_1));

    aruco_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/aruco_rover_pos", sensor_qos,
        std::bind(&NavEKF3DNode::aruco_callback, this, std::placeholders::_1));

    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odom", sensor_qos,
        std::bind(&NavEKF3DNode::vio_callback, this, std::placeholders::_1));

    wheel_info_sub_ = create_subscription<custom_msg::msg::MotorStatus>(
        "/NAV/motor_nav_status", sensor_qos,
        std::bind(&NavEKF3DNode::wheel_info_callback, this, std::placeholders::_1));

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom", sensor_qos,
        std::bind(&NavEKF3DNode::wheel_odom_callback, this, std::placeholders::_1));

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused_nav_ekf_odom", pub_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&NavEKF3DNode::timer_callback, this));
  }

private:
  // ---------------- Helpers ----------------
  void wait_for_service(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client) {
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_logger(), "Service not available");
    }
  }

  void call_trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
                    const std::string &name)
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut,
                                           std::chrono::seconds(5))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      const auto res = fut.get();
      RCLCPP_INFO(get_logger(), "Service '%s': %s", name.c_str(), res->message.c_str());
    }
  }

  // Mahalanobis-gated 2D position update.
  bool mahalanobis_gate_xy(double mx, double my, const Eigen::Matrix2d& R_cov) const {
    Eigen::Vector2d innovation(mx - ekf_->state(IDX_X), my - ekf_->state(IDX_Y));
    Eigen::Matrix<double, 2, STATE_DIM> H = Eigen::Matrix<double, 2, STATE_DIM>::Zero();
    H(0, IDX_X) = 1.0; H(1, IDX_Y) = 1.0;
    const Eigen::Matrix2d S = H * ekf_->covariance * H.transpose() + R_cov;
    const double maha2 = innovation.transpose() * S.inverse() * innovation;
    return std::isfinite(maha2) && maha2 < mahalanobis_gate_sq_;
  }

  // ---------------- Sensor callbacks ----------------
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const rclcpp::Time stamp(msg->header.stamp);
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double raw_roll, raw_pitch, raw_yaw;
    tf2::Matrix3x3(q).getRPY(raw_roll, raw_pitch, raw_yaw);

    double dt_imu = (last_imu_stamp_.nanoseconds() == 0)
                        ? (1.0 / 50.0)
                        : (stamp - last_imu_stamp_).seconds();
    last_imu_stamp_ = stamp;
    if (!(dt_imu > 0.0) || !std::isfinite(dt_imu)) return;

    // --- Low-pass roll and pitch to reject chassis vibration ---
    if (!lowpass_initialized_) {
      filtered_roll_  = raw_roll;
      filtered_pitch_ = raw_pitch;
      lowpass_initialized_ = true;
    } else {
      filtered_roll_  = low_pass_step(raw_roll,  filtered_roll_,  dt_imu, roll_pitch_lowpass_tau_);
      filtered_pitch_ = low_pass_step(raw_pitch, filtered_pitch_, dt_imu, roll_pitch_lowpass_tau_);
    }

    if (!initial_frame_set_) {
      initial_yaw_ = raw_yaw;
      initial_frame_set_ = true;
    }

    // --- Cache body-frame gyro for use in prediction ---
    last_body_angular_rate_ = Eigen::Vector3d(msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              msg->angular_velocity.z);

    // --- Measurement update with filtered orientation ---
    Eigen::Matrix3d R_rpy = ekf_->R_orientation_rpy;
    const double var_roll  = msg->orientation_covariance[0];
    const double var_pitch = msg->orientation_covariance[4];
    const double var_yaw   = msg->orientation_covariance[8];
    if (var_roll  > 0.0) R_rpy(0, 0) = var_roll;
    if (var_pitch > 0.0) R_rpy(1, 1) = var_pitch;
    if (var_yaw   > 0.0) R_rpy(2, 2) = var_yaw;

    const Eigen::Vector3d measured_rpy(filtered_roll_, filtered_pitch_, raw_yaw);
    ekf_->update_orientation_rpy(measured_rpy, R_rpy);
  }

  void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ekf_->update_position_xy(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             ekf_->R_position_xy_wheel);
  }

  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!include_lidar_) return;
    const double mx = msg->pose.pose.position.x;
    const double my = msg->pose.pose.position.y;
    if (mahalanobis_gate_xy(mx, my, ekf_->R_position_xy_lidar)) {
      ekf_->update_position_xy(mx, my, ekf_->R_position_xy_lidar);
    } else {
      RCLCPP_WARN(get_logger(), "Reject LiDAR odom (Mahalanobis)");
    }
  }

  void aruco_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!include_aruco_) return;
    const double mx = msg->pose.pose.position.x;
    const double my = msg->pose.pose.position.y;
    if (mahalanobis_gate_xy(mx, my, ekf_->R_position_xy_aruco)) {
      ekf_->update_position_xy(mx, my, ekf_->R_position_xy_aruco);
    } else {
      RCLCPP_WARN(get_logger(), "Reject ArUco odom (Mahalanobis)");
    }
  }

  void vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!include_vio_) return;
    const double mx = msg->pose.pose.position.x;
    const double my = msg->pose.pose.position.y;
    if (mahalanobis_gate_xy(mx, my, ekf_->R_position_xy_vio)) {
      ekf_->update_position_xy(mx, my, ekf_->R_position_xy_vio);
    } else {
      RCLCPP_WARN(get_logger(), "Reject VIO odom (Mahalanobis)");
    }
  }

  void wheel_info_callback(const custom_msg::msg::MotorStatus::SharedPtr msg) {
    if (msg->velocity.size() != 4 || msg->position.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Invalid MotorStatus: need 4 velocities and 4 positions.");
      return;
    }

    // Drive wiring sign conventions preserved from 2D node
    std::array<double, 4> wheel_speeds_mps;
    std::array<double, 4> wheel_angles_rad;
    wheel_speeds_mps[0] = msg->velocity[0] * RPM_TO_MS * GEAR_RATIO;
    wheel_speeds_mps[1] = msg->velocity[1] * RPM_TO_MS * GEAR_RATIO * -1.0;
    wheel_speeds_mps[2] = msg->velocity[2] * RPM_TO_MS * GEAR_RATIO * -1.0;
    wheel_speeds_mps[3] = msg->velocity[3] * RPM_TO_MS * GEAR_RATIO;

    for (std::size_t i = 0; i < 4; ++i) {
      double angle = msg->position[i] * INCR_TO_RAD;
      if      (angle >  M_PI) angle -= 2.0 * M_PI;
      else if (angle < -M_PI) angle += 2.0 * M_PI;
      wheel_angles_rad[i] = angle;
    }

    steer_fl_ = wheel_angles_rad[0];
    steer_fr_ = wheel_angles_rad[1];
    steer_br_ = wheel_angles_rad[2];
    steer_bl_ = wheel_angles_rad[3];

    vel_fl_ = wheel_speeds_mps[0];
    vel_fr_ = wheel_speeds_mps[1];
    vel_br_ = wheel_speeds_mps[2];
    vel_bl_ = wheel_speeds_mps[3];
  }

  // ---------------- Main prediction / publish loop ----------------
  void timer_callback() {
    const auto current_time = this->now();
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (!(dt > 0.0) || dt > 1.0) return;

    // ---- 1. Prediction with RK4 + RK2 Jacobian ----
    ekf_->predict(dt, last_body_angular_rate_);

    // ---- 2. Wheel-odom world-velocity measurement update ----
    const std::vector<double> body_vel = ekf_->compute_wheel_odom_body(
        steer_fr_, steer_br_, steer_fl_, steer_bl_,
        vel_fr_,   vel_br_,   vel_fl_,   vel_bl_);

    const Eigen::Vector3d body_velocity(body_vel[0], body_vel[1], 0.0);
    const Eigen::Matrix3d rotation_world_from_body = rotation_matrix_from_rpy(
        ekf_->state(IDX_ROLL), ekf_->state(IDX_PITCH), ekf_->state(IDX_YAW));
    const Eigen::Vector3d world_velocity = rotation_world_from_body * body_velocity;

    ekf_->update_velocity_world_xyz(world_velocity);

    // ---- 3. Publish Odometry ----
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    odom_msg.pose.pose.position.x = ekf_->state(IDX_X);
    odom_msg.pose.pose.position.y = ekf_->state(IDX_Y);
    odom_msg.pose.pose.position.z = ekf_->state(IDX_Z);

    tf2::Quaternion q_out;
    q_out.setRPY(ekf_->state(IDX_ROLL), ekf_->state(IDX_PITCH), ekf_->state(IDX_YAW));
    q_out.normalize();
    odom_msg.pose.pose.orientation = tf2::toMsg(q_out);

    odom_msg.pose.covariance[COV_POSE_XX]    = ekf_->covariance(IDX_X,     IDX_X);
    odom_msg.pose.covariance[COV_POSE_YY]    = ekf_->covariance(IDX_Y,     IDX_Y);
    odom_msg.pose.covariance[COV_POSE_ZZ]    = ekf_->covariance(IDX_Z,     IDX_Z);
    odom_msg.pose.covariance[COV_POSE_ROLL]  = ekf_->covariance(IDX_ROLL,  IDX_ROLL);
    odom_msg.pose.covariance[COV_POSE_PITCH] = ekf_->covariance(IDX_PITCH, IDX_PITCH);
    odom_msg.pose.covariance[COV_POSE_YAW]   = ekf_->covariance(IDX_YAW,   IDX_YAW);

    odom_msg.twist.twist.linear.x  = ekf_->state(IDX_VX);
    odom_msg.twist.twist.linear.y  = ekf_->state(IDX_VY);
    odom_msg.twist.twist.linear.z  = ekf_->state(IDX_VZ);
    odom_msg.twist.twist.angular.x = last_body_angular_rate_(0);
    odom_msg.twist.twist.angular.y = last_body_angular_rate_(1);
    odom_msg.twist.twist.angular.z = last_body_angular_rate_(2);

    ekf_pub_->publish(odom_msg);

    // ---- 4. Broadcast TF: odom -> base_link with full RPY ----
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = current_time;
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id  = "base_link";
    transform_stamped.transform.translation.x = ekf_->state(IDX_X);
    transform_stamped.transform.translation.y = ekf_->state(IDX_Y);
    transform_stamped.transform.translation.z = ekf_->state(IDX_Z);
    transform_stamped.transform.rotation      = tf2::toMsg(q_out);
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  // ---------------- Members ----------------
  std::shared_ptr<ExtendedKalmanFilter3D> ekf_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr wheel_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      aruco_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      vio_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         ekf_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
  rclcpp::Time                                                  last_time_;
  rclcpp::Time                                                  last_imu_stamp_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             bias_client_, zero_quat_client_, zero_pose_client_;

  // Gyro cache (body frame) for prediction
  Eigen::Vector3d last_body_angular_rate_ = Eigen::Vector3d::Zero();

  // Low-pass filter state for roll/pitch
  bool   lowpass_initialized_    = false;
  double filtered_roll_          = 0.0;
  double filtered_pitch_         = 0.0;
  double roll_pitch_lowpass_tau_ = 0.15;

  // Fusion toggles / gates
  bool   include_lidar_          = false;
  bool   include_aruco_          = false;
  bool   include_vio_            = false;
  double mahalanobis_gate_sq_    = 7.0;

  // Initial frame capture
  bool   initial_frame_set_      = false;
  double initial_yaw_            = 0.0;

  // Wheel kinematics cache
  double steer_fr_ = 0.0, steer_br_ = 0.0, steer_fl_ = 0.0, steer_bl_ = 0.0;
  double vel_fr_   = 0.0, vel_br_   = 0.0, vel_fl_   = 0.0, vel_bl_   = 0.0;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavEKF3DNode>());
  rclcpp::shutdown();
  return 0;
}
