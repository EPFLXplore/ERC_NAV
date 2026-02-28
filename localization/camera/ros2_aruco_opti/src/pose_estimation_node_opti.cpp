#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

class PoseEstimatorNode : public rclcpp::Node {
public:
  PoseEstimatorNode()
  : Node("pose_estimator_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    using std::placeholders::_1;

    const auto now = get_clock()->now();
    time_of_last_pose_ = now;
    time_of_last_yaw_meas_ = now;
    time_of_last_good_triangulation_ = now;
    last_callback_time_ = now;

    aruco_subscription_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers",
      10,
      std::bind(&PoseEstimatorNode::listenerCallback, this, _1));

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fused_nav_ekf_odom",
      10,
      std::bind(&PoseEstimatorNode::odometryCallback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&PoseEstimatorNode::republishMapOdomTransform, this));

    RCLCPP_INFO(get_logger(), "C++ pose estimation node started.");
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  struct ValidMarker {
    int id;
    geometry_msgs::msg::Pose pose;
    size_t k;
  };

  static double wrap(double a)
  {
    double out = std::fmod(a + kPi, 2.0 * kPi);
    if (out < 0.0) {
      out += 2.0 * kPi;
    }
    return out - kPi;
  }

  static double degToRad(double deg)
  {
    return deg * kPi / 180.0;
  }

  static double radToDeg(double rad)
  {
    return rad * 180.0 / kPi;
  }

  static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    return quat;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &quat)
  {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static Eigen::Matrix4d poseToMat(double x, double y, double yaw)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 0) = c;
    M(0, 1) = -s;
    M(1, 0) = s;
    M(1, 1) = c;
    M(0, 3) = x;
    M(1, 3) = y;
    return M;
  }

  static double medianOf(std::vector<double> values)
  {
    if (values.empty()) {
      return 0.0;
    }

    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    if ((n % 2U) == 1U) {
      return values[n / 2U];
    }
    return 0.5 * (values[n / 2U - 1U] + values[n / 2U]);
  }

  static bool arePointsCounterclockwise(const std::array<Eigen::Vector2d, 3> &vertices)
  {
    size_t min_y_idx = 0;
    for (size_t i = 1; i < vertices.size(); ++i) {
      const bool lower_y = vertices[i].y() < vertices[min_y_idx].y();
      const bool same_y_bigger_x =
        std::abs(vertices[i].y() - vertices[min_y_idx].y()) < 1e-9 &&
        vertices[i].x() > vertices[min_y_idx].x();
      if (lower_y || same_y_bigger_x) {
        min_y_idx = i;
      }
    }

    const Eigen::Vector2d A = vertices[min_y_idx];
    const Eigen::Vector2d B = vertices[(min_y_idx + 2U) % 3U];
    const Eigen::Vector2d C = vertices[(min_y_idx + 1U) % 3U];

    const Eigen::Vector2d AB = B - A;
    const Eigen::Vector2d AC = C - A;
    const double cross_prod = AB.x() * AC.y() - AB.y() * AC.x();
    return cross_prod < 0.0;
  }

  std::optional<Eigen::Vector2d> triangulateOpti(
    const std::array<Eigen::Vector2d, 3> &landmarks,
    const std::array<double, 3> &phi_angles_deg,
    const Eigen::Vector2d &current_pos) const
  {
    const Eigen::Vector2d OA = landmarks[0];
    const Eigen::Vector2d OB = landmarks[1];
    const Eigen::Vector2d OC = landmarks[2];

    const std::array<double, 3> phi = {
      degToRad(phi_angles_deg[0]),
      degToRad(phi_angles_deg[1]),
      degToRad(phi_angles_deg[2])};

    const Eigen::Vector2d P_est = current_pos;

    const Eigen::Vector2d AB = OB - OA;
    const Eigen::Vector2d AC = OC - OA;
    const Eigen::Vector2d BC = OC - OB;
    const Eigen::Vector2d CA = OA - OC;
    const Eigen::Vector2d CB = -BC;
    const Eigen::Vector2d BA = -AB;

    const Eigen::Vector2d AP = P_est - OA;
    const Eigen::Vector2d BP = P_est - OB;
    const Eigen::Vector2d CP = P_est - OC;

    Eigen::Matrix2d M_A;
    M_A.row(0) = AC.transpose();
    M_A.row(1) = AB.transpose();

    Eigen::Matrix2d M_B;
    M_B.row(0) = BA.transpose();
    M_B.row(1) = BC.transpose();

    Eigen::Matrix2d M_C;
    M_C.row(0) = CB.transpose();
    M_C.row(1) = CA.transpose();

    if (
      std::abs(M_A.determinant()) < 1e-7 ||
      std::abs(M_B.determinant()) < 1e-7 ||
      std::abs(M_C.determinant()) < 1e-7)
    {
      return std::nullopt;
    }

    const Eigen::Vector2d APtri = M_A.inverse().transpose() * AP;
    const Eigen::Vector2d BPtri = M_B.inverse().transpose() * BP;
    const Eigen::Vector2d CPtri = M_C.inverse().transpose() * CP;

    const double epsilon = 0.05;
    const bool inside_triangle =
      APtri[0] >= epsilon && APtri[1] >= epsilon &&
      BPtri[0] >= epsilon && BPtri[1] >= epsilon &&
      CPtri[0] >= epsilon && CPtri[1] >= epsilon;
    if (!inside_triangle) {
      return std::nullopt;
    }

    const auto residual = [&](const Eigen::Vector2d &P) -> Eigen::Vector3d {
      const Eigen::Vector2d PA = OA - P;
      const Eigen::Vector2d PB = OB - P;
      const Eigen::Vector2d PC = OC - P;

      const double nPA = PA.norm();
      const double nPB = PB.norm();
      const double nPC = PC.norm();
      if (nPA < 1e-9 || nPB < 1e-9 || nPC < 1e-9) {
        return Eigen::Vector3d::Constant(1e6);
      }

      return Eigen::Vector3d(
        PA.dot(PB) / (nPA * nPB) - std::cos(phi[0]),
        PB.dot(PC) / (nPB * nPC) - std::cos(phi[1]),
        PA.dot(PC) / (nPA * nPC) - std::cos(phi[2]));
    };

    Eigen::Vector2d P = P_est;
    for (int iter = 0; iter < 40; ++iter) {
      const Eigen::Vector3d r = residual(P);

      Eigen::Matrix<double, 3, 2> J;
      constexpr double eps = 1e-6;
      for (int c = 0; c < 2; ++c) {
        Eigen::Vector2d P_eps = P;
        P_eps[c] += eps;
        J.col(c) = (residual(P_eps) - r) / eps;
      }

      const Eigen::Matrix2d H = J.transpose() * J;
      if (std::abs(H.determinant()) < 1e-12) {
        return std::nullopt;
      }

      const Eigen::Vector2d g = J.transpose() * r;
      const Eigen::Vector2d delta = H.ldlt().solve(-g);
      P += delta;

      if (!std::isfinite(P.x()) || !std::isfinite(P.y())) {
        return std::nullopt;
      }
      if (delta.norm() < 1e-6 || r.norm() < 1e-6) {
        break;
      }
    }

    return P;
  }

  std::optional<Eigen::Vector2d> solvePositionFromTwoBearings(
    const Eigen::Vector2d &A,
    const Eigen::Vector2d &B,
    double phiA,
    double phiB,
    double psi_map) const
  {
    const double thetaA = wrap(psi_map + phiA);
    const double thetaB = wrap(psi_map + phiB);

    const Eigen::Vector2d vA(std::cos(thetaA), std::sin(thetaA));
    const Eigen::Vector2d vB(std::cos(thetaB), std::sin(thetaB));

    Eigen::Matrix2d M;
    M.col(0) = vA;
    M.col(1) = -vB;

    if (std::abs(M.determinant()) < 1e-6) {
      return std::nullopt;
    }

    const Eigen::Vector2d rhs = A - B;
    const Eigen::Vector2d sol = M.fullPivLu().solve(rhs);
    const double tA = sol[0];
    return A - tA * vA;
  }

  std::optional<Eigen::Vector2d> tryAllTriplets(
    const std::vector<ValidMarker> &valid_markers,
    const ros2_aruco_interfaces::msg::ArucoMarkers &msg) const
  {
    std::vector<Eigen::Vector2d> solutions;

    size_t used_triplets = 0;
    for (size_t a = 0; a + 2 < valid_markers.size(); ++a) {
      for (size_t b = a + 1; b + 1 < valid_markers.size(); ++b) {
        for (size_t c = b + 1; c < valid_markers.size(); ++c) {
          if (used_triplets >= static_cast<size_t>(max_nbr_triplets_)) {
            break;
          }

          const auto &mA = valid_markers[a];
          const auto &mB = valid_markers[b];
          const auto &mC = valid_markers[c];

          std::array<Eigen::Vector2d, 3> lms = {
            Eigen::Vector2d(landmark_poses_[mA.id].first, landmark_poses_[mA.id].second),
            Eigen::Vector2d(landmark_poses_[mB.id].first, landmark_poses_[mB.id].second),
            Eigen::Vector2d(landmark_poses_[mC.id].first, landmark_poses_[mC.id].second)};

          std::array<double, 3> bearings = {
            msg.ar_angles_list[mA.k],
            msg.ar_angles_list[mB.k],
            msg.ar_angles_list[mC.k]};

          if (arePointsCounterclockwise(lms)) {
            lms = {lms[2], lms[1], lms[0]};
            bearings = {bearings[2], bearings[1], bearings[0]};
          }

          const auto pair_angle = [](double u, double v) {
            return std::min(std::abs(u - v), std::abs(360.0 - std::abs(u) - std::abs(v)));
          };

          const std::array<double, 3> phis = {
            pair_angle(bearings[0], bearings[1]),
            pair_angle(bearings[1], bearings[2]),
            pair_angle(bearings[2], bearings[0])};

          const Eigen::Vector2d ref = initialized_map_odom_tf_
            ? Eigen::Vector2d(curr_map_odom_base_x_, curr_map_odom_base_y_)
            : Eigen::Vector2d(erc_start_pos_[0], erc_start_pos_[1]);

          const auto p = triangulateOpti(lms, phis, ref);
          if (p.has_value()) {
            solutions.push_back(p.value());
          }

          ++used_triplets;
        }
      }
    }

    if (solutions.empty()) {
      return std::nullopt;
    }

    Eigen::Vector2d avg(0.0, 0.0);
    for (const auto &p : solutions) {
      avg += p;
    }
    avg /= static_cast<double>(solutions.size());
    return avg;
  }

  std::optional<Eigen::Vector2d> tryFilteredLS(
    const std::vector<ValidMarker> &valid_markers,
    const ros2_aruco_interfaces::msg::ArucoMarkers &msg,
    double yaw) const
  {
    std::vector<Eigen::Vector2d> lms;
    std::vector<double> phs;
    lms.reserve(valid_markers.size());
    phs.reserve(valid_markers.size());

    for (const auto &marker : valid_markers) {
      lms.emplace_back(landmark_poses_[marker.id].first, landmark_poses_[marker.id].second);
      phs.push_back(degToRad(msg.ar_angles_list[marker.k]));
    }

    std::vector<bool> good(phs.size(), false);
    for (size_t i = 0; i < phs.size(); ++i) {
      for (size_t j = i + 1; j < phs.size(); ++j) {
        const double delta = std::abs(wrap((yaw + phs[i]) - (yaw + phs[j]) - kPi));
        if (delta >= degToRad(20.0)) {
          good[i] = true;
          good[j] = true;
        }
      }
    }

    std::vector<Eigen::Vector2d> lms_f;
    std::vector<double> phs_f;
    for (size_t i = 0; i < phs.size(); ++i) {
      if (good[i]) {
        lms_f.push_back(lms[i]);
        phs_f.push_back(phs[i]);
      }
    }

    if (phs_f.size() < 2U) {
      return std::nullopt;
    }

    Eigen::Vector2d x = initialized_map_odom_tf_
      ? Eigen::Vector2d(curr_map_odom_base_x_, curr_map_odom_base_y_)
      : Eigen::Vector2d(erc_start_pos_[0], erc_start_pos_[1]);

    for (int iter = 0; iter < 35; ++iter) {
      Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
      Eigen::Vector2d g = Eigen::Vector2d::Zero();

      for (size_t i = 0; i < lms_f.size(); ++i) {
        const double dx = lms_f[i].x() - x.x();
        const double dy = lms_f[i].y() - x.y();
        const double q = dx * dx + dy * dy;
        if (q < 1e-9) {
          continue;
        }

        const double psi = std::atan2(dy, dx);
        const double r = wrap(psi - yaw - phs_f[i]);

        const Eigen::Vector2d J(dy / q, -dx / q);
        H += J * J.transpose();
        g += J * r;
      }

      if (std::abs(H.determinant()) < 1e-12) {
        return std::nullopt;
      }

      const Eigen::Vector2d delta = H.ldlt().solve(-g);
      x += delta;

      if (!std::isfinite(x.x()) || !std::isfinite(x.y())) {
        return std::nullopt;
      }
      if (delta.norm() < 1e-6) {
        break;
      }
    }

    return x;
  }

  geometry_msgs::msg::TransformStamped calculateRobustTfAvg(
    const std::vector<geometry_msgs::msg::TransformStamped> &tf_list,
    const std::vector<double> &yaw_list) const
  {
    std::vector<Eigen::Vector2d> translations;
    translations.reserve(tf_list.size());
    for (const auto &tf : tf_list) {
      translations.emplace_back(tf.transform.translation.x, tf.transform.translation.y);
    }

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(translations.size());
    ys.reserve(translations.size());
    for (const auto &t : translations) {
      xs.push_back(t.x());
      ys.push_back(t.y());
    }

    const Eigen::Vector2d med_t(medianOf(xs), medianOf(ys));

    std::vector<double> dists;
    dists.reserve(translations.size());
    for (const auto &t : translations) {
      dists.push_back((t - med_t).norm());
    }
    const double mad_t = medianOf(dists);
    const double thr = std::max(3.0 * mad_t, 0.5);

    std::vector<Eigen::Vector2d> inliers;
    for (const auto &t : translations) {
      if ((t - med_t).norm() < thr) {
        inliers.push_back(t);
      }
    }
    if (inliers.size() < 3U) {
      inliers = translations;
    }

    Eigen::Vector2d final_t(0.0, 0.0);
    for (const auto &t : inliers) {
      final_t += t;
    }
    if (!inliers.empty()) {
      final_t /= static_cast<double>(inliers.size());
    }

    double avg_yaw = 0.0;
    if (!yaw_list.empty()) {
      for (const double y : yaw_list) {
        avg_yaw += y;
      }
      avg_yaw /= static_cast<double>(yaw_list.size());
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = final_t.x();
    tf.transform.translation.y = final_t.y();
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = yawToQuat(avg_yaw);

    return tf;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pos_x_ = msg->pose.pose.position.x;
    odom_pos_y_ = msg->pose.pose.position.y;
    odom_yaw_ = yawFromQuat(msg->pose.pose.orientation);
  }

  void republishMapOdomTransform()
  {
    if (!has_prev_map_odom_tf_) {
      return;
    }
    prev_map_odom_tf_.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(prev_map_odom_tf_);
  }

  void listenerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    if (initialized_map_odom_tf_) {
      const double dt = (now - last_callback_time_).seconds();
      if (dt < callback_period_limit_) {
        return;
      }
    }
    last_callback_time_ = now;

    try {
      const auto transform = tf_buffer_.lookupTransform("odom", "map", tf2::TimePointZero);

      const Eigen::Matrix4d T_map_odom = poseToMat(
        transform.transform.translation.x,
        transform.transform.translation.y,
        yawFromQuat(transform.transform.rotation));

      Eigen::Matrix4d T_odom_base = poseToMat(odom_pos_x_, odom_pos_y_, odom_yaw_);

      if (std::abs(odom_pos_x_) < 1e-4 && std::abs(odom_pos_y_) < 1e-4) {
        curr_map_odom_base_x_ = x_estimate_;
        curr_map_odom_base_y_ = y_estimate_;
        curr_map_odom_base_yaw_ = yaw_estimate_;
        T_odom_base = Eigen::Matrix4d::Identity();
      } else {
        const Eigen::Matrix4d T_map_base = T_map_odom * T_odom_base;
        curr_map_odom_base_x_ = T_map_base(0, 3);
        curr_map_odom_base_y_ = T_map_base(1, 3);
        curr_map_odom_base_yaw_ = std::atan2(T_map_base(1, 0), T_map_base(0, 0));

        if (!triangulated_new_xy_) {
          x_estimate_ = curr_map_odom_base_x_;
          y_estimate_ = curr_map_odom_base_y_;
          yaw_estimate_ = curr_map_odom_base_yaw_;
        }
      }
    } catch (const tf2::TransformException &) {
      if (!has_prev_map_odom_tf_) {
        curr_map_odom_base_x_ = odom_pos_x_;
        curr_map_odom_base_y_ = odom_pos_y_;
        curr_map_odom_base_yaw_ = odom_yaw_;
      }
    }

    std::vector<ValidMarker> valid_markers;
    valid_markers.reserve(msg->marker_ids.size());

    for (size_t k = 0; k < msg->marker_ids.size() && k < msg->poses.size(); ++k) {
      if (k >= msg->ar_angles_list.size()) {
        continue;
      }

      const int idx = static_cast<int>(msg->marker_ids[k]);
      if (idx < 0 || idx >= static_cast<int>(landmark_poses_.size())) {
        continue;
      }

      const auto &lm = landmark_poses_[static_cast<size_t>(idx)];
      if (std::abs(lm.first) >= map_size_ || std::abs(lm.second) >= map_size_) {
        continue;
      }

      const double ar_dist = std::hypot(msg->poses[k].position.x, msg->poses[k].position.y);
      if (ar_dist > max_aruco_dist_for_tvec_use_) {
        continue;
      }

      valid_markers.push_back(ValidMarker{idx, msg->poses[k], k});
    }

    if (valid_markers.size() < 2U) {
      return;
    }

    bool is_measurement_valid = false;
    geometry_msgs::msg::TransformStamped transform_msg;

    if (!initialized_map_odom_tf_) {
      if (valid_markers.size() == 2U) {
        const auto &mA = valid_markers[0];
        const auto &mB = valid_markers[1];
        const Eigen::Vector2d A(landmark_poses_[mA.id].first, landmark_poses_[mA.id].second);
        const Eigen::Vector2d B(landmark_poses_[mB.id].first, landmark_poses_[mB.id].second);
        const double phiA = degToRad(msg->ar_angles_list[mA.k]);
        const double phiB = degToRad(msg->ar_angles_list[mB.k]);

        const double x0 = erc_start_pos_[0];
        const double y0 = erc_start_pos_[1];
        const double yawA = wrap(std::atan2(A.y() - y0, A.x() - x0) - phiA);
        const double yawB = wrap(std::atan2(B.y() - y0, B.x() - x0) - phiB);
        yaw_estimate_ = wrap(0.5 * (yawA + yawB));
        measured_new_yaw_ = true;
        time_of_last_yaw_meas_ = now;

        const Eigen::Matrix4d T_map_base = poseToMat(erc_start_pos_[0], erc_start_pos_[1], yaw_estimate_);
        const Eigen::Matrix4d T_odom_base = Eigen::Matrix4d::Identity();
        const Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base.inverse();

        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = "odom";
        transform_msg.transform.translation.x = T_map_odom(0, 3);
        transform_msg.transform.translation.y = T_map_odom(1, 3);
        transform_msg.transform.translation.z = 0.0;
        transform_msg.transform.rotation = yawToQuat(std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
        is_measurement_valid = true;
      } else {
        const auto ptri = tryAllTriplets(valid_markers, *msg);
        if (ptri.has_value()) {
          const double dx = ptri->x() - erc_start_pos_[0];
          const double dy = ptri->y() - erc_start_pos_[1];
          if (std::sqrt(dx * dx + dy * dy) < 0.4) {
            x_estimate_ = ptri->x();
            y_estimate_ = ptri->y();
            triangulated_new_xy_ = true;
            time_of_last_pose_ = now;
            time_of_last_good_triangulation_ = now;
            measured_good_triang_ = true;

            std::vector<double> yaw_list;
            yaw_list.reserve(valid_markers.size());
            for (const auto &marker : valid_markers) {
              const auto &lm = landmark_poses_[marker.id];
              const double measured_phi = degToRad(msg->ar_angles_list[marker.k]);
              const double bearing_map = std::atan2(lm.second - y_estimate_, lm.first - x_estimate_);
              yaw_list.push_back(wrap(bearing_map - measured_phi));
            }

            double sum_sin = 0.0;
            double sum_cos = 0.0;
            for (const double y : yaw_list) {
              sum_sin += std::sin(y);
              sum_cos += std::cos(y);
            }
            yaw_estimate_ = std::atan2(sum_sin, sum_cos);
            measured_new_yaw_ = true;
            time_of_last_yaw_meas_ = now;

            const Eigen::Matrix4d T_map_base = poseToMat(x_estimate_, y_estimate_, yaw_estimate_);
            const Eigen::Matrix4d T_odom_base = Eigen::Matrix4d::Identity();
            const Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base.inverse();

            transform_msg.header.stamp = now;
            transform_msg.header.frame_id = "map";
            transform_msg.child_frame_id = "odom";
            transform_msg.transform.translation.x = T_map_odom(0, 3);
            transform_msg.transform.translation.y = T_map_odom(1, 3);
            transform_msg.transform.translation.z = 0.0;
            transform_msg.transform.rotation =
              yawToQuat(std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
            is_measurement_valid = true;
          }
        } else {
          const auto pls = tryFilteredLS(valid_markers, *msg, yaw_estimate_);
          if (pls.has_value()) {
            const double dx = pls->x() - erc_start_pos_[0];
            const double dy = pls->y() - erc_start_pos_[1];
            if (std::sqrt(dx * dx + dy * dy) < 0.4) {
              x_estimate_ = pls->x();
              y_estimate_ = pls->y();
              triangulated_new_xy_ = true;
              time_of_last_pose_ = now;

              std::vector<double> yaw_list;
              yaw_list.reserve(valid_markers.size());
              for (const auto &marker : valid_markers) {
                const auto &lm = landmark_poses_[marker.id];
                const double measured_phi = degToRad(msg->ar_angles_list[marker.k]);
                const double bearing_map = std::atan2(lm.second - y_estimate_, lm.first - x_estimate_);
                yaw_list.push_back(wrap(bearing_map - measured_phi));
              }

              double sum_sin = 0.0;
              double sum_cos = 0.0;
              for (const double y : yaw_list) {
                sum_sin += std::sin(y);
                sum_cos += std::cos(y);
              }
              yaw_estimate_ = std::atan2(sum_sin, sum_cos);
              measured_new_yaw_ = true;
              time_of_last_yaw_meas_ = now;

              const Eigen::Matrix4d T_map_base = poseToMat(x_estimate_, y_estimate_, yaw_estimate_);
              const Eigen::Matrix4d T_odom_base = Eigen::Matrix4d::Identity();
              const Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base.inverse();

              transform_msg.header.stamp = now;
              transform_msg.header.frame_id = "map";
              transform_msg.child_frame_id = "odom";
              transform_msg.transform.translation.x = T_map_odom(0, 3);
              transform_msg.transform.translation.y = T_map_odom(1, 3);
              transform_msg.transform.translation.z = 0.0;
              transform_msg.transform.rotation =
                yawToQuat(std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
              is_measurement_valid = true;
            }
          }
        }
      }
    } else {
      if (valid_markers.size() == 2U) {
        const auto &mA = valid_markers[0];
        const auto &mB = valid_markers[1];
        const Eigen::Vector2d A(landmark_poses_[mA.id].first, landmark_poses_[mA.id].second);
        const Eigen::Vector2d B(landmark_poses_[mB.id].first, landmark_poses_[mB.id].second);

        const auto p2 = solvePositionFromTwoBearings(
          A,
          B,
          degToRad(msg->ar_angles_list[mA.k]),
          degToRad(msg->ar_angles_list[mB.k]),
          yaw_estimate_);

        if (p2.has_value()) {
          const double dx = p2->x() - curr_map_odom_base_x_;
          const double dy = p2->y() - curr_map_odom_base_y_;
          if (std::sqrt(dx * dx + dy * dy) < 2.0) {
            const Eigen::Vector2d prev(curr_map_odom_base_x_, curr_map_odom_base_y_);
            const Eigen::Vector2d filtered =
              (1.0 - lpf_coeff_) * prev + lpf_coeff_ * p2.value();
            x_estimate_ = filtered.x();
            y_estimate_ = filtered.y();
            triangulated_new_xy_ = true;
            time_of_last_pose_ = now;
          }
        }
      } else {
        const auto ptri = tryAllTriplets(valid_markers, *msg);
        const auto pls = tryFilteredLS(valid_markers, *msg, yaw_estimate_);
        const double dt_since_last_triang = (now - time_of_last_good_triangulation_).seconds();

        if (ptri.has_value()) {
          const double dx = ptri->x() - curr_map_odom_base_x_;
          const double dy = ptri->y() - curr_map_odom_base_y_;
          if (std::sqrt(dx * dx + dy * dy) < 2.0) {
            x_estimate_ = ptri->x();
            y_estimate_ = ptri->y();
            triangulated_new_xy_ = true;
            measured_good_triang_ = true;
            time_of_last_good_triangulation_ = now;
            time_of_last_pose_ = now;
          }
        } else if (pls.has_value() && dt_since_last_triang >= min_least_squares_dt_from_triang_) {
          const double dx = pls->x() - curr_map_odom_base_x_;
          const double dy = pls->y() - curr_map_odom_base_y_;
          if (std::sqrt(dx * dx + dy * dy) < 1.0) {
            x_estimate_ = pls->x();
            y_estimate_ = pls->y();
            triangulated_new_xy_ = true;
            time_of_last_pose_ = now;
          }
        }

        if (triangulated_new_xy_) {
          std::vector<double> yaw_list;
          yaw_list.reserve(valid_markers.size());
          for (const auto &marker : valid_markers) {
            const auto &lm = landmark_poses_[marker.id];
            const double measured_phi = degToRad(msg->ar_angles_list[marker.k]);
            const double bearing_map = std::atan2(lm.second - y_estimate_, lm.first - x_estimate_);
            yaw_list.push_back(wrap(bearing_map - measured_phi));
          }

          const double yaw_dt = (now - time_of_last_yaw_meas_).seconds();
          if (yaw_dt >= min_yaw_dt_) {
            double sum_sin = 0.0;
            double sum_cos = 0.0;
            for (const double y : yaw_list) {
              sum_sin += std::sin(y);
              sum_cos += std::cos(y);
            }
            yaw_estimate_ = std::atan2(sum_sin, sum_cos);
            measured_new_yaw_ = true;
            time_of_last_yaw_meas_ = now;
          }
        }
      }
    }

    if (triangulated_new_xy_ && initialized_map_odom_tf_) {
      const double dx = x_estimate_ - curr_map_odom_base_x_;
      const double dy = y_estimate_ - curr_map_odom_base_y_;
      const double jump = std::hypot(dx, dy);
      if (jump > max_translation_jump_) {
        triangulated_new_xy_ = false;
      } else {
        is_measurement_valid = true;

        const Eigen::Matrix4d T_map_base = poseToMat(x_estimate_, y_estimate_, yaw_estimate_);
        const Eigen::Matrix4d T_odom_base = poseToMat(odom_pos_x_, odom_pos_y_, odom_yaw_);
        const Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base.inverse();

        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = "odom";
        transform_msg.transform.translation.x = T_map_odom(0, 3);
        transform_msg.transform.translation.y = T_map_odom(1, 3);
        transform_msg.transform.translation.z = 0.0;
        transform_msg.transform.rotation = yawToQuat(std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));

        prev_map_odom_tf_ = transform_msg;
        has_prev_map_odom_tf_ = true;
      }
    }

    if (initialized_map_odom_tf_ && has_prev_map_odom_tf_) {
      tf_broadcaster_->sendTransform(prev_map_odom_tf_);
      if (triangulated_new_xy_) {
        RCLCPP_INFO(get_logger(), "Estimated yaw(deg): %.2f", radToDeg(yaw_estimate_));
        RCLCPP_INFO(get_logger(), "Estimated X map: %.3f", x_estimate_);
        RCLCPP_INFO(get_logger(), "Estimated Y map: %.3f", y_estimate_);
      }
    }

    if (init_callback_counter_ < nbr_init_callbacks_for_avg_ && is_measurement_valid) {
      ++init_callback_counter_;
      avg_initialization_tfs_.push_back(transform_msg);
      yaw_init_list_.push_back(yaw_estimate_);

      if (init_callback_counter_ == nbr_init_callbacks_for_avg_) {
        initialized_map_odom_tf_ = true;
        prev_map_odom_tf_ = calculateRobustTfAvg(avg_initialization_tfs_, yaw_init_list_);
        has_prev_map_odom_tf_ = true;
        tf_broadcaster_->sendTransform(prev_map_odom_tf_);
        RCLCPP_INFO(get_logger(), "Initialized MAP->ODOM transform.");
      }
    }

    triangulated_new_xy_ = false;
    measured_new_yaw_ = false;
    measured_good_triang_ = false;
  }

  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_estimate_{0.0};
  double y_estimate_{0.0};
  double yaw_estimate_{0.0};

  bool triangulated_new_xy_{false};
  bool measured_new_yaw_{false};
  bool measured_good_triang_{false};

  rclcpp::Time time_of_last_pose_{0, 0, RCL_ROS_TIME};
  rclcpp::Time time_of_last_yaw_meas_{0, 0, RCL_ROS_TIME};
  rclcpp::Time time_of_last_good_triangulation_{0, 0, RCL_ROS_TIME};

  const double map_size_{300.0};
  const int nbr_init_callbacks_for_avg_{35};
  int init_callback_counter_{0};
  bool initialized_map_odom_tf_{false};
  rclcpp::Time last_callback_time_{0, 0, RCL_ROS_TIME};
  const double callback_period_limit_{1.0 / 15.0};
  std::vector<geometry_msgs::msg::TransformStamped> avg_initialization_tfs_;
  std::vector<double> yaw_init_list_;

  const double min_yaw_dt_{30.0};
  const double max_translation_jump_{0.5};
  const double max_aruco_dist_for_tvec_use_{4.0};
  const double min_least_squares_dt_from_triang_{15.0};
  const int max_nbr_triplets_{5};

  double odom_pos_x_{0.0};
  double odom_pos_y_{0.0};
  double odom_yaw_{0.0};

  double curr_map_odom_base_x_{0.0};
  double curr_map_odom_base_y_{0.0};
  double curr_map_odom_base_yaw_{0.0};

  const double lpf_coeff_{0.7};

  geometry_msgs::msg::TransformStamped prev_map_odom_tf_;
  bool has_prev_map_odom_tf_{false};

  const std::array<double, 2> erc_start_pos_{0.655, 2.515};

  const std::array<std::pair<double, double>, 15> landmark_poses_{
    std::make_pair(-0.585, 0.0),
    std::make_pair(2.62, 0.505),
    std::make_pair(1.46, 8.45),
    std::make_pair(-2.28, 15.81),
    std::make_pair(3.74, 19.07),
    std::make_pair(7.04, 14.67),
    std::make_pair(11.46, 19.78),
    std::make_pair(15.51, 19.33),
    std::make_pair(16.3, 14.87),
    std::make_pair(999999.0, 999999.0),
    std::make_pair(999999.0, 999999.0),
    std::make_pair(999999.0, 999999.0),
    std::make_pair(999999.0, 999999.0),
    std::make_pair(999999.0, 999999.0),
    std::make_pair(999999.0, 999999.0)};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseEstimatorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
