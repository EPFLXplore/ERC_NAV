// 3D Extended Kalman Filter for Navigation
// State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz]  (position in world,
//               orientation as ZYX Euler angles, linear velocity in body frame)
//
// Prediction:
//   - Mean propagation via RK4 integration of the full nonlinear kinematics
//     (position integrates body-frame velocity rotated into world, Euler angles integrate the
//     body-frame gyro rate through the Euler-rate matrix). The gyro's wz is
//     inverse-variance blended with the wheel-derived omega_z before use.
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
//   - Wheel odometry body-frame velocity (vx, vy, vz).
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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/path.hpp>


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

// Ring buffer storing the last CAPACITY pre-update innovations for one scalar
// channel.  autocorr() returns the normalized autocorrelation sequence
// r[k] = R(k)/R(0), k = 0..MAX_LAG.
// For a well-tuned EKF: r[0] = 1, r[k≠0] ≈ 0 (innovation whiteness).
struct InnovBuffer {
  static constexpr int CAPACITY = 200;
  static constexpr int MAX_LAG  = 50;
  std::deque<double> buf;   // buf[0] = most recent

  void push(double v) {
    buf.push_front(v);
    if (static_cast<int>(buf.size()) > CAPACITY) buf.pop_back();
  }

  std::vector<double> autocorr() const {
    const int n = static_cast<int>(buf.size());
    if (n < 4) return {};

    double mean = 0.0;
    for (double v : buf) mean += v;
    mean /= n;

    double var = 0.0;
    for (double v : buf) { double d = v - mean; var += d * d; }
    var /= n;

    const int lags = std::min(MAX_LAG + 1, n);
    std::vector<double> ac(lags, 0.0);
    if (var < 1e-15) { if (!ac.empty()) ac[0] = 1.0; return ac; }

    for (int lag = 0; lag < lags; ++lag) {
      double s = 0.0;
      for (int t = 0; t + lag < n; ++t)
        s += (buf[t] - mean) * (buf[t + lag] - mean);
      ac[lag] = (s / n) / var;
    }
    return ac;
  }
};

}  // namespace


// =====================================================================
// ExtendedKalmanFilter3D
// =====================================================================
class ExtendedKalmanFilter3D {
public:
  using StateVector   = Eigen::Matrix<double, STATE_DIM, 1>;
  using StateMatrix   = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

  StateVector state;              // [x y z roll pitch yaw vx_body vy_body vz_body]
  StateMatrix covariance;
  StateMatrix process_noise;      // continuous-time Q; integrated as Q*dt

  Eigen::Matrix3d R_orientation_rpy;    // AHRS IMU RPY variance (roll, pitch, yaw)
  Eigen::Matrix2d R_position_xy_lidar;
  Eigen::Matrix2d R_position_xy_aruco;
  Eigen::Matrix3d R_position_xyz_vio;
  Eigen::Matrix3d R_velocity_body_xyz;
  Eigen::Matrix3d R_orientation_rpy_vio;  // VIO RPY variance (roll, pitch, yaw)

  ExtendedKalmanFilter3D() {
    state.setZero();
    covariance = StateMatrix::Identity() * 1e-3;

    process_noise.setZero();
    process_noise(IDX_X,     IDX_X)     = 0.01;
    process_noise(IDX_Y,     IDX_Y)     = 0.01;
    process_noise(IDX_Z,     IDX_Z)     = 0.01;
    process_noise(IDX_ROLL,  IDX_ROLL)  = 0.01;
    process_noise(IDX_PITCH, IDX_PITCH) = 0.01;
    process_noise(IDX_YAW,   IDX_YAW)   = 0.01;
    process_noise(IDX_VX,    IDX_VX)    = 0.10;
    process_noise(IDX_VY,    IDX_VY)    = 0.10;
    process_noise(IDX_VZ,    IDX_VZ)    = 0.10;

    R_orientation_rpy    = Eigen::Matrix3d::Identity() * 1e-4;    // IMU AHRS RPY variance ≈ 0.57° std

    R_orientation_rpy(0, 0) = 1e-4;    // roll  ≈ 0.57°
    R_orientation_rpy(1, 1) = 1e-4;    // pitch ≈ 0.57°
    R_orientation_rpy(2, 2) = 3e-4;    // yaw   ≈ 1°
    
    R_orientation_rpy_vio.setZero();
    R_orientation_rpy_vio(0, 0) = 10e-3;   // roll  std ≈
    R_orientation_rpy_vio(1, 1) = 10e-3;   // pitch std ≈ 
    R_orientation_rpy_vio(2, 2) = 29e-3;   // yaw   std ≈

    R_position_xy_lidar  = Eigen::Matrix2d::Identity() * 0.015;   // std 0.12 m
    R_position_xy_aruco  = Eigen::Matrix2d::Identity() * 0.007;   // two times lower, because its a correction.
    R_position_xyz_vio   = Eigen::Matrix3d::Identity() * 0.0225; // std 0.15m (~15cm error on 30m)
    R_velocity_body_xyz  = Eigen::Matrix3d::Identity() * 0.1;  // from wheel odom meas
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
  // Input u = body-frame angular rate (gyro).  Body-frame linear velocity
  // lives in the state vector and is rotated into world for position integration.
  // -------------------------------------------------------------------
  StateVector state_derivative(const StateVector& s,
                               const Eigen::Vector3d& body_angular_rate) const
  {
    StateVector deriv = StateVector::Zero();

    const Eigen::Vector3d body_velocity(s(IDX_VX), s(IDX_VY), s(IDX_VZ));
    const Eigen::Vector3d world_velocity =
        rotation_matrix_from_rpy(s(IDX_ROLL), s(IDX_PITCH), s(IDX_YAW)) * body_velocity;

    deriv(IDX_X) = world_velocity(0);
    deriv(IDX_Y) = world_velocity(1);
    deriv(IDX_Z) = world_velocity(2);

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
  //   - Process noise: the position/velocity block is discretized analytically
  //     (Van Loan) instead of a diagonal dt*Q_continuous approximation, so the
  //     known position<->velocity noise correlation of a constant-velocity
  //     model is preserved. For d/dt[p; v] = [[0, R], [0, 0]] [p; v] + [0; I] w,
  //     with w continuous white noise of intensity Qc_v (R frozen over the
  //     step, matching the state_transition linearization below):
  //       Q_pp = (dt^3/3) * R * Qc_v * R^T   (+ dt * Qc_p for the position's
  //                                            own unmodeled-disturbance term)
  //       Q_pv = (dt^2/2) * R * Qc_v
  //       Q_vv =  dt      * Qc_v
  //     Attitude noise stays a plain diagonal dt*Q term (it models direct
  //     orientation-disturbance uncertainty, not a random-walk-driven state).
  // -------------------------------------------------------------------
  void predict(double dt, const Eigen::Vector3d& body_angular_rate) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;

    // --- Mean propagation (RK4) ---
    state = rk4_integrate(state, body_angular_rate, dt);
    state(IDX_ROLL)  = normalize_angle(state(IDX_ROLL));
    state(IDX_PITCH) = normalize_angle(state(IDX_PITCH));
    state(IDX_YAW)   = normalize_angle(state(IDX_YAW));

    // --- Covariance propagation ---
    const double roll  = state(IDX_ROLL);
    const double pitch = state(IDX_PITCH);
    const double yaw   = state(IDX_YAW);
    const Eigen::Vector3d body_v(state(IDX_VX), state(IDX_VY), state(IDX_VZ));
    const Eigen::Matrix3d R = rotation_matrix_from_rpy(roll, pitch, yaw);
    const Eigen::Matrix3d E = euler_rate_matrix(roll, pitch);

    StateMatrix jacobian_continuous = StateMatrix::Zero();

    // dp/dv_B = R
    jacobian_continuous.block<3, 3>(IDX_X, IDX_VX) = R;

    // dp/d_eta: attitude uncertainty propagates into position uncertainty
    constexpr double EPS = 1e-5;
    const Eigen::Vector3d Rv = R * body_v;
    for (int k = 0; k < 3; ++k) {
      double e[3] = {roll, pitch, yaw};
      e[k] += EPS;
      const Eigen::Matrix3d Rp = rotation_matrix_from_rpy(e[0], e[1], e[2]);
      jacobian_continuous.block<3, 1>(IDX_X, IDX_ROLL + k) = (Rp * body_v - Rv) / EPS;
    }

    // d_eta/d_eta: roll/pitch uncertainty propagates into attitude-rate uncertainty
    // (E doesn't depend on yaw, so the yaw column stays zero)
    const Eigen::Vector3d Ew = E * body_angular_rate;
    for (int k = 0; k < 2; ++k) {
      double e[2] = {roll, pitch};
      e[k] += EPS;
      const Eigen::Matrix3d Ep = euler_rate_matrix(e[0], e[1]);
      jacobian_continuous.block<3, 1>(IDX_ROLL, IDX_ROLL + k) = (Ep * body_angular_rate - Ew) / EPS;
    }

    const StateMatrix jacobian_squared = jacobian_continuous * jacobian_continuous;
    const StateMatrix state_transition =
        StateMatrix::Identity()
        + dt * jacobian_continuous
        + (0.5 * dt * dt) * jacobian_squared;

    // --- Discrete process noise (Van Loan for the position/velocity block) ---
    const Eigen::Matrix3d Qc_p = process_noise.block<3, 3>(IDX_X,  IDX_X);
    const Eigen::Matrix3d Qc_v = process_noise.block<3, 3>(IDX_VX, IDX_VX);
    const Eigen::Matrix3d RQvR  = R * Qc_v * R.transpose();
    const Eigen::Matrix3d RQv   = R * Qc_v;

    StateMatrix process_noise_discrete = StateMatrix::Zero();
    process_noise_discrete.block<3, 3>(IDX_X,  IDX_X)  = (dt * dt * dt / 3.0) * RQvR + dt * Qc_p;
    process_noise_discrete.block<3, 3>(IDX_X,  IDX_VX) = (dt * dt / 2.0) * RQv;
    process_noise_discrete.block<3, 3>(IDX_VX, IDX_X)  = process_noise_discrete.block<3, 3>(IDX_X, IDX_VX).transpose();
    process_noise_discrete.block<3, 3>(IDX_VX, IDX_VX) = dt * Qc_v;
    process_noise_discrete.block<3, 3>(IDX_ROLL, IDX_ROLL) =
        process_noise.block<3, 3>(IDX_ROLL, IDX_ROLL) * dt;

    covariance = state_transition * covariance * state_transition.transpose()
                 + process_noise_discrete;
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

  void update_velocity_body_xyz(const Eigen::Vector3d& measured_v_body) {
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, IDX_VX) = 1.0;
    H(1, IDX_VY) = 1.0;
    H(2, IDX_VZ) = 1.0;

    const Eigen::Vector3d innovation(
        measured_v_body(0) - state(IDX_VX),
        measured_v_body(1) - state(IDX_VY),
        measured_v_body(2) - state(IDX_VZ));

    const Eigen::Matrix3d S = H * covariance * H.transpose() + R_velocity_body_xyz;
    const Eigen::Matrix<double, STATE_DIM, 3> K =
        covariance * H.transpose() * S.inverse();

    state += K * innovation;
    apply_joseph_form_update<3>(H, K, R_velocity_body_xyz);
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
    // Mahalanobis gate squared (3 DoF chi^2 95% ≈ 7.815; 99% ≈ 11.345).
    declare_parameter<double>("mahalanobis_gate_xyz_squared", 10.0);

    // Wheel-derived yaw-rate (ICR omega_z) variance used when blending with the
    // gyro.  Kept high relative to typical gyro noise so it only pulls the fused
    // rate away from the gyro when the gyro is degraded or clearly disagrees.
    declare_parameter<double>("wheel_omega_z_variance", 0.5);
    // Fallback gyro yaw-rate variance, used when the IMU message reports an
    // invalid (<=0) angular_velocity_covariance[8].
    declare_parameter<double>("gyro_omega_z_variance_default", 1e-3);

    include_lidar_ = get_parameter("include_lidar").as_bool();
    include_aruco_ = get_parameter("include_aruco").as_bool();
    include_vio_   = get_parameter("include_vio").as_bool();
    roll_pitch_lowpass_tau_ = get_parameter("roll_pitch_lowpass_tau").as_double();
    mahalanobis_gate_sq_    = get_parameter("mahalanobis_gate_squared").as_double();
    mahalanobis_gate_xyz_sq_    = get_parameter("mahalanobis_gate_xyz_squared").as_double();

    wheel_omega_z_variance_ = get_parameter("wheel_omega_z_variance").as_double();
    gyro_omega_z_variance_default_ = get_parameter("gyro_omega_z_variance_default").as_double();

    RCLCPP_INFO(get_logger(), "3D EKF | LiDAR: %s  ArUco: %s  VIO: %s  tau_rp=%.3fs  wheel_wz_var=%.3f",
                include_lidar_ ? "on" : "off",
                include_aruco_ ? "on" : "off",
                include_vio_   ? "on" : "off",
                roll_pitch_lowpass_tau_,
                wheel_omega_z_variance_);

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

    vio_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/zed/zed_node/pose_with_covariance_restamped", sensor_qos,
        std::bind(&NavEKF3DNode::vio_callback, this, std::placeholders::_1));

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom", sensor_qos,
        std::bind(&NavEKF3DNode::wheel_odom_callback, this, std::placeholders::_1));

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused_nav_ekf_odom", pub_qos);
    erc_map_pub_ = create_publisher<nav_msgs::msg::Odometry>("erc_map_localization", pub_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/fused_nav_ekf_path", pub_qos);
    ros_time_pub_ = create_publisher<builtin_interfaces::msg::Time>("/NAV/ros_time", pub_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    last_time_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&NavEKF3DNode::timer_callback, this));

    // Innovation autocorrelation publishers
    using FAMsg = std_msgs::msg::Float64MultiArray;
    auto latch = rclcpp::QoS{rclcpp::KeepLast{1}}.reliable();
    pub_ac_imu_roll_  = create_publisher<FAMsg>("/nav_ekf/autocorr/imu_roll",  latch);
    pub_ac_imu_pitch_ = create_publisher<FAMsg>("/nav_ekf/autocorr/imu_pitch", latch);
    pub_ac_imu_yaw_   = create_publisher<FAMsg>("/nav_ekf/autocorr/imu_yaw",   latch);
    pub_ac_vel_x_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vel_x",     latch);
    pub_ac_vel_y_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vel_y",     latch);
    pub_ac_vel_z_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vel_z",     latch);
    pub_ac_lidar_x_   = create_publisher<FAMsg>("/nav_ekf/autocorr/lidar_x",   latch);
    pub_ac_lidar_y_   = create_publisher<FAMsg>("/nav_ekf/autocorr/lidar_y",   latch);
    pub_ac_aruco_x_   = create_publisher<FAMsg>("/nav_ekf/autocorr/aruco_x",   latch);
    pub_ac_aruco_y_   = create_publisher<FAMsg>("/nav_ekf/autocorr/aruco_y",   latch);
    pub_ac_vio_x_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_x",     latch);
    pub_ac_vio_y_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_y",     latch);
    pub_ac_vio_z_     = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_z",     latch);
    pub_ac_vio_roll_  = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_roll",  latch);
    pub_ac_vio_pitch_ = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_pitch", latch);
    pub_ac_vio_yaw_   = create_publisher<FAMsg>("/nav_ekf/autocorr/vio_yaw",   latch);
    autocorr_timer_ = create_wall_timer(std::chrono::milliseconds(400),
                                        std::bind(&NavEKF3DNode::autocorr_callback, this));
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

  bool mahalanobis_gate_xyz(double mx, double my, double mz, const Eigen::Matrix3d& R_cov) const {
    Eigen::Vector3d innovation(mx - ekf_->state(IDX_X), my - ekf_->state(IDX_Y), mz - ekf_->state(IDX_Z));
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, IDX_X) = 1.0; H(1, IDX_Y) = 1.0; H(2, IDX_Z) = 1.0;
    const Eigen::Matrix3d S = H * ekf_->covariance * H.transpose() + R_cov;
    const double maha2 = innovation.transpose() * S.inverse() * innovation;
    return std::isfinite(maha2) && maha2 < mahalanobis_gate_xyz_sq_;
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
    // Blend gyro wz with the wheel-derived omega_z (inverse-variance weighting:
    // the minimum-variance combination of two independent noisy measurements of
    // the same body yaw rate). wheel_omega_z_variance_ is set high, so this only
    // pulls the fused rate away from the gyro when the gyro disagrees strongly
    // or the wheel estimate is comparatively certain.
    double wz_fused = msg->angular_velocity.z;
    if (has_wheel_omega_z_) {
      const double var_gyro_wz = (msg->angular_velocity_covariance[8] > 0.0)
                                      ? msg->angular_velocity_covariance[8]
                                      : gyro_omega_z_variance_default_;
      const double w_gyro  = 1.0 / var_gyro_wz;
      const double w_wheel = 1.0 / wheel_omega_z_variance_;
      wz_fused = (w_gyro * msg->angular_velocity.z + w_wheel * last_wheel_omega_z_) / (w_gyro + w_wheel);
    }

    last_body_angular_rate_ = Eigen::Vector3d(msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              wz_fused);

    // --- Measurement update with filtered orientation ---
    Eigen::Matrix3d R_rpy = ekf_->R_orientation_rpy;
    const double var_roll  = msg->orientation_covariance[0];
    const double var_pitch = msg->orientation_covariance[4];
    const double var_yaw   = msg->orientation_covariance[8];
    if (var_roll  > 0.0) R_rpy(0, 0) = var_roll;
    if (var_pitch > 0.0) R_rpy(1, 1) = var_pitch;
    if (var_yaw   > 0.0) R_rpy(2, 2) = var_yaw;

    const Eigen::Vector3d measured_rpy(filtered_roll_, filtered_pitch_, raw_yaw);

    // Capture pre-update innovations for whiteness check
    buf_imu_roll_.push(normalize_angle(filtered_roll_ - ekf_->state(IDX_ROLL)));
    buf_imu_pitch_.push(normalize_angle(filtered_pitch_ - ekf_->state(IDX_PITCH)));
    buf_imu_yaw_.push(normalize_angle(raw_yaw - ekf_->state(IDX_YAW)));

    ekf_->update_orientation_rpy(measured_rpy, R_rpy);
  }

  void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { //body frame speeds
    const double vx_body = msg->twist.twist.linear.x;
    const double vy_body = msg->twist.twist.linear.y;
    const double vz_body = 0.0;  // wheel odom does not measure vertical velocity
    if (!std::isfinite(vx_body) || !std::isfinite(vy_body)) {
      return;
    }

    buf_vel_x_.push(vx_body - ekf_->state(IDX_VX));
    buf_vel_y_.push(vy_body - ekf_->state(IDX_VY));

    ekf_->update_velocity_body_xyz(Eigen::Vector3d(vx_body, vy_body, vz_body));

    // Cache wheel-derived yaw rate for blending with the gyro in imu_callback.
    const double omega_z = msg->twist.twist.angular.z;
    if (std::isfinite(omega_z)) {
      last_wheel_omega_z_ = omega_z;
      has_wheel_omega_z_  = true;
    }
  }

  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!include_lidar_) return;
    const double mx = msg->pose.pose.position.x;
    const double my = msg->pose.pose.position.y;
    buf_lidar_x_.push(mx - ekf_->state(IDX_X));
    buf_lidar_y_.push(my - ekf_->state(IDX_Y));
    if (mahalanobis_gate_xy(mx, my, ekf_->R_position_xy_lidar)) {
      ekf_->update_position_xy(mx, my, ekf_->R_position_xy_lidar);
    } else {
      RCLCPP_WARN(get_logger(), "Reject LiDAR odom (Mahalanobis)");
    }
  }

  void aruco_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!include_aruco_) return;
    RCLCPP_WARN(get_logger(), "ArUco OODDOOOOOOOOOOOOOOOOOOOOOM ");
    const double mx = msg->pose.pose.position.x;
    const double my = msg->pose.pose.position.y;
    buf_aruco_x_.push(mx - ekf_->state(IDX_X));
    buf_aruco_y_.push(my - ekf_->state(IDX_Y));
    if (mahalanobis_gate_xy(mx, my, ekf_->R_position_xy_aruco)) {
      ekf_->update_position_xy(mx, my, ekf_->R_position_xy_aruco);
    } else {
      RCLCPP_WARN(get_logger(), "Reject ArUco odom (Mahalanobis)");
    }
  }

  void vio_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!include_vio_) return;

    // ============================================================
    // VIO pose is assumed to be the pose of the OAK-d camera frame.
    //
    // EKF state is base_link, so convert:
    //
    // T_measurement = T_base -> zed_link * T_vio_message * T_zed_link -> base;
    // ============================================================

    const auto &p_msg = msg->pose.pose.position;
    const auto &q_msg = msg->pose.pose.orientation;

    if (!std::isfinite(p_msg.x) || !std::isfinite(p_msg.y) || !std::isfinite(p_msg.z) ||
        !std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
        !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w)) {
      return;
    }

    tf2::Quaternion q_odom_oakd(
        q_msg.x,
        q_msg.y,
        q_msg.z,
        q_msg.w
    );

    if (q_odom_oakd.length2() < 1e-12) {
      return;
    }

    q_odom_oakd.normalize();

    tf2::Vector3 p_odom_oakd(
        p_msg.x,
        p_msg.y,
        p_msg.z
    );

    tf2::Transform T_zedodom_oakd(q_odom_oakd, p_odom_oakd);

    // ---------------- Lookup static extrinsic base_link -> Zed_2i_v1_1 ----------------
    geometry_msgs::msg::TransformStamped tf_base_zed_msg;

    try {
      const bool zero_stamp =
          (msg->header.stamp.sec == 0u && msg->header.stamp.nanosec == 0u);

      if (zero_stamp) {
        tf_base_zed_msg = tf_buffer_->lookupTransform(
            "base_link",
            "Zed_2i_v1_1",
            tf2::TimePointZero);
      } else {
        const rclcpp::Time t(msg->header.stamp, get_clock()->get_clock_type());

        if (tf_buffer_->canTransform(
                "base_link",
                "Zed_2i_v1_1",
                t,
                rclcpp::Duration::from_seconds(0.05))) {
          tf_base_zed_msg = tf_buffer_->lookupTransform(
              "base_link",
              "Zed_2i_v1_1",
              t,
              rclcpp::Duration::from_seconds(0.1));
        } else {
          tf_base_zed_msg = tf_buffer_->lookupTransform(
              "base_link",
              "Zed_2i_v1_1",
              tf2::TimePointZero);
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "VIO correction skipped: cannot lookup TF base_link <- OAK-d: %s",
          ex.what());
      return;
    }

    const auto &tr = tf_base_zed_msg.transform.translation;
    const auto &qr = tf_base_zed_msg.transform.rotation;

    tf2::Quaternion q_base_oakd(
        qr.x,
        qr.y,
        qr.z,
        qr.w
    );

    if (q_base_oakd.length2() < 1e-12) {
      return;
    }

    q_base_oakd.normalize();

    tf2::Vector3 p_base_oakd(
        tr.x,
        tr.y,
        tr.z
    );

    const tf2::Transform T_base_oakd(q_base_oakd, p_base_oakd);
    const tf2::Transform T_oakd_base = T_base_oakd.inverse();

    // VIO message is actually:
    //   T_zed_odom -> zed_current
    //
    // EKF measurement should be:
    //   T_odom -> base_link_current
    //
    // EKF odom starts at initial base_link,
    // and zed_odom starts at initial zed/OAK-d:
    const tf2::Transform T_odom_base =
        T_base_oakd * T_zedodom_oakd * T_oakd_base;


    const tf2::Vector3 p_odom_base = T_odom_base.getOrigin();
    tf2::Quaternion q_odom_base = T_odom_base.getRotation();
    q_odom_base.normalize();

    // ---------------- Position correction using corrected base_link position ----------------
    const double mx = p_odom_base.x();
    const double my = p_odom_base.y();
    const double mz = p_odom_base.z();

    if (std::isfinite(mx) && std::isfinite(my) && std::isfinite(mz)) {
      buf_vio_x_.push(mx - ekf_->state(IDX_X));
      buf_vio_y_.push(my - ekf_->state(IDX_Y));
      buf_vio_z_.push(mz - ekf_->state(IDX_Z));

      if (mahalanobis_gate_xyz(mx, my, mz, ekf_->R_position_xyz_vio)) {
        ekf_->update_position_xyz(mx, my, mz, ekf_->R_position_xyz_vio);
      } else {
        RCLCPP_WARN(get_logger(), "Reject VIO base_link position (Mahalanobis)");
      }
    }

    // ---------------- Orientation correction using corrected base_link orientation ----------------
    double vio_roll, vio_pitch, vio_yaw;
    tf2::Matrix3x3(q_odom_base).getRPY(vio_roll, vio_pitch, vio_yaw);

    if (!std::isfinite(vio_roll) ||
        !std::isfinite(vio_pitch) ||
        !std::isfinite(vio_yaw)) {
      return;
    }

    Eigen::Vector3d measured_rpy(vio_roll, vio_pitch, vio_yaw);

    Eigen::Matrix3d R_vio_rpy = ekf_->R_orientation_rpy_vio;

    // PoseWithCovarianceStamped covariance layout:
    // [x, y, z, roll, pitch, yaw], row-major 6x6.
    //
    // NOTE:
    // This covariance is originally for the OAK-d pose.
    // We reuse its diagonal as an approximation for base_link orientation.
    const double var_roll  = msg->pose.covariance[21];
    const double var_pitch = msg->pose.covariance[28];
    const double var_yaw   = msg->pose.covariance[35];

    if (std::isfinite(var_roll)  && var_roll  > 1e-12) R_vio_rpy(0, 0) = var_roll;
    if (std::isfinite(var_pitch) && var_pitch > 1e-12) R_vio_rpy(1, 1) = var_pitch;
    if (std::isfinite(var_yaw)   && var_yaw   > 1e-12) R_vio_rpy(2, 2) = var_yaw;

    buf_vio_roll_.push(normalize_angle(vio_roll  - ekf_->state(IDX_ROLL)));
    buf_vio_pitch_.push(normalize_angle(vio_pitch - ekf_->state(IDX_PITCH)));
    buf_vio_yaw_.push(normalize_angle(vio_yaw   - ekf_->state(IDX_YAW)));

    ekf_->update_orientation_rpy(measured_rpy, R_vio_rpy);
  }

  // Publish normalized autocorrelation [r(0)=1, r(1), ..., r(MAX_LAG)] for one channel.
  void publish_autocorr(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & pub,
    const InnovBuffer & buf)
  {
    const auto ac = buf.autocorr();
    if (ac.empty()) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = ac;
    pub->publish(msg);
  }

  void autocorr_callback() {
    publish_autocorr(pub_ac_imu_roll_,  buf_imu_roll_);
    publish_autocorr(pub_ac_imu_pitch_, buf_imu_pitch_);
    publish_autocorr(pub_ac_imu_yaw_,   buf_imu_yaw_);
    publish_autocorr(pub_ac_vel_x_,     buf_vel_x_);
    publish_autocorr(pub_ac_vel_y_,     buf_vel_y_);
    publish_autocorr(pub_ac_vel_z_,     buf_vel_z_);
    if (include_lidar_) {
      publish_autocorr(pub_ac_lidar_x_, buf_lidar_x_);
      publish_autocorr(pub_ac_lidar_y_, buf_lidar_y_);
    }
    if (include_aruco_) {
      publish_autocorr(pub_ac_aruco_x_, buf_aruco_x_);
      publish_autocorr(pub_ac_aruco_y_, buf_aruco_y_);
    }
    if (include_vio_) {
      publish_autocorr(pub_ac_vio_x_,     buf_vio_x_);
      publish_autocorr(pub_ac_vio_y_,     buf_vio_y_);
      publish_autocorr(pub_ac_vio_z_,     buf_vio_z_);
      publish_autocorr(pub_ac_vio_roll_,  buf_vio_roll_);
      publish_autocorr(pub_ac_vio_pitch_, buf_vio_pitch_);
      publish_autocorr(pub_ac_vio_yaw_,   buf_vio_yaw_);
    }
  }

  // Look up map -> base_link (only present if some other node is publishing
  // map -> odom) and publish it in the ERC map coordinate convention:
  //   x_erc   =  x_map
  //   y_erc   = -y_map
  //   yaw_erc = -yaw_map
  // Publishes nothing if the map -> odom TF does not exist.
  void publish_erc_map_localization(const rclcpp::Time & stamp) {
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      // Passing a non-null error_msg suppresses tf2's internal console warning
      // for unknown frames; it reports the reason through error_msg instead.
      std::string tf_error;
      if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, &tf_error)) {
        return;
      }
      tf_map_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "erc_map_localization skipped: cannot lookup TF map <- base_link: %s",
                          ex.what());
      return;
    }

    tf2::Quaternion q_map_base(
        tf_map_base.transform.rotation.x,
        tf_map_base.transform.rotation.y,
        tf_map_base.transform.rotation.z,
        tf_map_base.transform.rotation.w);
    if (q_map_base.length2() < 1e-12) return;
    q_map_base.normalize();

    double roll_map, pitch_map, yaw_map;
    tf2::Matrix3x3(q_map_base).getRPY(roll_map, pitch_map, yaw_map);

    const auto &t = tf_map_base.transform.translation;
    const double x_erc   = t.x;
    const double y_erc   = -t.y;
    const double z_erc   = t.z;
    const double yaw_erc = normalize_angle(-yaw_map);
    const double pitch_erc = normalize_angle(-pitch_map);


    tf2::Quaternion q_erc;
    q_erc.setRPY(roll_map, pitch_erc, yaw_erc);
    q_erc.normalize();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "erc_map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_erc;
    msg.pose.pose.position.y = y_erc;
    msg.pose.pose.position.z = z_erc;
    msg.pose.pose.orientation = tf2::toMsg(q_erc);

    erc_map_pub_->publish(msg);
  }

  // ---------------- Main prediction / publish loop ----------------
  void timer_callback() {
    const auto current_time = this->now();
    builtin_interfaces::msg::Time ros_time_msg;
    ros_time_msg.sec = static_cast<int32_t>(current_time.seconds());
    ros_time_msg.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000LL);
    ros_time_pub_->publish(ros_time_msg);

    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (!(dt > 0.0) || dt > 1.0) return;

    // ---- 1. Prediction with RK4 + RK2 Jacobian ----
    ekf_->predict(dt, last_body_angular_rate_);

    // ---- 2. Publish Odometry ----
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
    odom_msg.twist.covariance[COV_POSE_XX]    = ekf_->covariance(IDX_VX, IDX_VX);
    odom_msg.twist.covariance[COV_POSE_YY]    = ekf_->covariance(IDX_VY, IDX_VY);
    odom_msg.twist.covariance[COV_POSE_ZZ]    = ekf_->covariance(IDX_VZ, IDX_VZ);
    odom_msg.twist.covariance[COV_POSE_ROLL]  = ekf_->R_orientation_rpy(0, 0);
    odom_msg.twist.covariance[COV_POSE_PITCH] = ekf_->R_orientation_rpy(1, 1);
    odom_msg.twist.covariance[COV_POSE_YAW]   = ekf_->R_orientation_rpy(2, 2);

    ekf_pub_->publish(odom_msg);

    // ---- 3. Broadcast TF: odom -> base_link with full RPY ----
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = current_time;
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id  = "base_link";
    transform_stamped.transform.translation.x = ekf_->state(IDX_X);
    transform_stamped.transform.translation.y = ekf_->state(IDX_Y);
    transform_stamped.transform.translation.z = ekf_->state(IDX_Z);
    transform_stamped.transform.rotation      = tf2::toMsg(q_out);
    tf_broadcaster_->sendTransform(transform_stamped);

    // ---- Publish ERC-frame localization from map -> base_link (if available) ----
    publish_erc_map_localization(current_time);

    // ---- Publish EKF path for RViz ----
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = odom_msg.header;
    path_pose.pose = odom_msg.pose.pose;

    bool add_pose = false;

    if (path_msg_.poses.empty()) {
      add_pose = true;
    } else {
      const auto &last_pose = path_msg_.poses.back().pose.position;
      const auto &curr_pose = path_pose.pose.position;

      const double dx = curr_pose.x - last_pose.x;
      const double dy = curr_pose.y - last_pose.y;
      const double dz = curr_pose.z - last_pose.z;

      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist > 0.02) {  // add point every 2 cm
        add_pose = true;
      }
    }

    if (add_pose) {
      path_msg_.header.stamp = current_time;
      path_msg_.header.frame_id = "odom";
      path_msg_.poses.push_back(path_pose);

      if (path_msg_.poses.size() > max_path_poses_) {
        path_msg_.poses.erase(path_msg_.poses.begin());
      }
    }

    path_pub_->publish(path_msg_);
  }

  // ---------------- Members ----------------
  std::shared_ptr<ExtendedKalmanFilter3D> ekf_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      aruco_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vio_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>                              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                   tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         ekf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         erc_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  size_t max_path_poses_ = 9000;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr    ros_time_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
  rclcpp::Time                                                  last_time_;
  rclcpp::Time                                                  last_imu_stamp_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             bias_client_, zero_quat_client_, zero_pose_client_;

  // Gyro cache (body frame) for prediction
  Eigen::Vector3d last_body_angular_rate_ = Eigen::Vector3d::Zero();

  // Wheel-derived yaw rate cache, blended into the gyro wz via inverse-variance
  // weighting (see imu_callback).
  double last_wheel_omega_z_          = 0.0;
  bool   has_wheel_omega_z_           = false;
  double wheel_omega_z_variance_      = 0.5;
  double gyro_omega_z_variance_default_ = 1e-3;

  // Low-pass filter state for roll/pitch
  bool   lowpass_initialized_    = false;
  double filtered_roll_          = 0.0;
  double filtered_pitch_         = 0.0;
  double roll_pitch_lowpass_tau_ = 0.2;

  // Fusion toggles / gates
  bool   include_lidar_          = false;
  bool   include_aruco_          = false;
  bool   include_vio_            = false;
  double mahalanobis_gate_sq_    = 7.0;
  double mahalanobis_gate_xyz_sq_=11.0;

  // Initial frame capture
  bool   initial_frame_set_      = false;
  double initial_yaw_            = 0.0;

  // Innovation ring buffers (one per scalar channel)
  InnovBuffer buf_imu_roll_, buf_imu_pitch_, buf_imu_yaw_;
  InnovBuffer buf_vel_x_, buf_vel_y_, buf_vel_z_;
  InnovBuffer buf_lidar_x_, buf_lidar_y_;
  InnovBuffer buf_aruco_x_, buf_aruco_y_;
  InnovBuffer buf_vio_x_,   buf_vio_y_, buf_vio_z_;
  InnovBuffer buf_vio_roll_, buf_vio_pitch_, buf_vio_yaw_;

  // Autocorrelation publishers
  using FAMsg = std_msgs::msg::Float64MultiArray;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_imu_roll_, pub_ac_imu_pitch_, pub_ac_imu_yaw_;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_vel_x_,    pub_ac_vel_y_,     pub_ac_vel_z_;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_lidar_x_,  pub_ac_lidar_y_;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_aruco_x_,  pub_ac_aruco_y_;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_vio_x_,    pub_ac_vio_y_,     pub_ac_vio_z_;
  rclcpp::Publisher<FAMsg>::SharedPtr pub_ac_vio_roll_, pub_ac_vio_pitch_, pub_ac_vio_yaw_;
  rclcpp::TimerBase::SharedPtr autocorr_timer_;

};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavEKF3DNode>());
  rclcpp::shutdown();
  return 0;
}
