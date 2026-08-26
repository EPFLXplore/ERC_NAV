// Constant-position 2D KF estimating the map->odom transform from the ArUco
// range-bearing localization solver (/aruco_rover_pos).
//
// Author: Arno Laurie 21 August 2026
//
// This node owns the map->odom TF.  The local EKF (nav_ekf_3d_node) owns
// odom->base_link and never sees a map-frame measurement, so global
// corrections cannot introduce discontinuities in the odom frame.
//
// Handover: pose_estimator_lidar_node broadcasts map->odom during its two init
// phases, then publishes the final transform once on the latched
// /map_odom_init topic and stops broadcasting.  This node stays silent until
// that seed arrives, then takes over.
//
// state vector: X = [x, y, yaw]^T of T_map_odom
//
// state equation:        x(t+1) = x(t) + w,  w ~ N(0, Q*dt)
// propagate covariance:  P(t+1) = P(t) + Q*dt
// measurement equation:  z(t)   = x(t) + v,  v ~ N(0, R)      (H = I)
// compute Kalman gain:   K = P*H^T*(H*P*H^T + R)^-1 = P*(P + R)^-1
// update covariance:     P = (I - K)*P*(I - K)^T + K*R*K^T    (Joseph form)
//
// R is taken from each /aruco_rover_pos message: pose_estimator_lidar_node
// publishes the inverse of its solver's information matrix, so R is anisotropic
// (markers clustered in bearing leave one direction weakly constrained) and
// carries x-yaw / y-yaw cross-terms.  The full 3x3 block is used.  The
// meas_sigma_* parameters are only the fallback for messages that carry no
// usable covariance.
//
// /aruco_rover_pos measures T_map_base, not the state, so each measurement is
// converted with the live odom->base_link TF:
//
//     z = T_map_odom = T_map_base * inverse(T_odom_base)
//
// and its covariance is converted with the Jacobian of that same expression,
// which couples the measured yaw into map->odom xy through the lever arm
// t_odom_base (see measurement_noise_map_odom).
//
// The constant-position model is only valid on that quantity: map->odom is the
// slowly drifting correction, whereas map->base_link moves with the rover.
//
// Q is constant: process noise grows with wall time only, independently of
// whether the rover is moving.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

namespace {

/** Wrap an angle into (-pi, pi]. */
double wrap_angle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

}  // namespace

/* ------------------------------------------------------------------ */
/*  Filter                                                            */
/* ------------------------------------------------------------------ */
class KF2D
{
public:
    KF2D(const Mat3 &Q, const Mat3 &R) : Q_(Q), R_(R)
    {
        x_.setZero();
        P_ = R_;
    }

    /** Constant-position propagation: only the covariance grows. */
    void predict(double dt)
    {
        if (dt > 0.0) P_ += Q_ * dt;
    }

    /** Innovation z - x, with the yaw component wrapped. */
    Vec3 innovation(const Vec3 &z) const
    {
        Vec3 y = z - x_;
        y(2) = wrap_angle(y(2));
        return y;
    }

    /** Squared Mahalanobis distance of an innovation (3 DOF). */
    double chi2(const Vec3 &y, const Mat3 &R) const
    {
        const Mat3 S = P_ + R;
        return y.dot(S.ldlt().solve(y));
    }

    void update(const Vec3 &y, const Mat3 &R)
    {
        const Mat3 S = P_ + R;
        const Mat3 K = P_ * S.inverse();

        x_ += K * y;
        x_(2) = wrap_angle(x_(2));

        // Joseph form: stays symmetric positive semi-definite even with a
        // suboptimal gain.
        const Mat3 I = Mat3::Identity();
        P_ = (I - K) * P_ * (I - K).transpose() + K * R * K.transpose();
    }

    /** Widen the covariance so a mis-seeded filter can be pulled back in. */
    void inflate(double factor) { P_ *= factor; }

    const Vec3 &state() const { return x_; }
    const Mat3 &covariance() const { return P_; }

    void reset(const Vec3 &x)
    {
        x_ = x;
        x_(2) = wrap_angle(x_(2));
        P_ = R_;
    }

private:
    Vec3 x_;  // state [x, y, yaw] of T_map_odom
    Mat3 P_;  // state covariance
    Mat3 Q_;  // process noise density (per second, at full scale)
    Mat3 R_;  // nominal measurement noise, seeds P_ on construction and reset
};

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */
class GlobalNavKF2DNode : public rclcpp::Node
{
public:
    GlobalNavKF2DNode() : Node("global_nav_kf_2d")
    {
        broadcast_rate_hz_ = declare_parameter<double>("broadcast_rate_hz", 5.0);
        const double meas_sigma_xy_m =
            declare_parameter<double>("meas_sigma_xy_m", 0.45);
        const double meas_sigma_yaw_deg =
            declare_parameter<double>("meas_sigma_yaw_deg", 3.0);
        const double process_sigma_xy_m_per_s =
            declare_parameter<double>("process_sigma_xy_m_per_s", 0.25);
        const double process_sigma_yaw_deg_per_s =
            declare_parameter<double>("process_sigma_yaw_deg_per_s", 0.3);
        gate_chi2_ = declare_parameter<double>("mahalanobis_gate_chi2", 16.27);
        max_consecutive_rejects_ =
            declare_parameter<int>("max_consecutive_rejects", 10);
        inflate_only_in_camera_recovery_ =
            declare_parameter<bool>("inflate_only_in_camera_recovery", true);

        if (!(broadcast_rate_hz_ > 0.0) ||
            !(meas_sigma_xy_m > 0.0) || !(meas_sigma_yaw_deg > 0.0) ||
            !(process_sigma_xy_m_per_s > 0.0) ||
            !(process_sigma_yaw_deg_per_s > 0.0)) {
            RCLCPP_ERROR(get_logger(),
                "broadcast_rate_hz and all sigmas must be positive");
            throw std::runtime_error("invalid global_nav_kf parameters");
        }

        const double meas_sigma_yaw_rad = meas_sigma_yaw_deg * M_PI / 180.0;
        const double process_sigma_yaw_rad_per_s =
            process_sigma_yaw_deg_per_s * M_PI / 180.0;

        R_nominal_ = Mat3::Zero();
        R_nominal_(0, 0) = meas_sigma_xy_m * meas_sigma_xy_m;
        R_nominal_(1, 1) = meas_sigma_xy_m * meas_sigma_xy_m;
        R_nominal_(2, 2) = meas_sigma_yaw_rad * meas_sigma_yaw_rad;

        Mat3 Q = Mat3::Zero();
        Q(0, 0) = process_sigma_xy_m_per_s * process_sigma_xy_m_per_s;
        Q(1, 1) = process_sigma_xy_m_per_s * process_sigma_xy_m_per_s;
        Q(2, 2) = process_sigma_yaw_rad_per_s * process_sigma_yaw_rad_per_s;

        kf_ = std::make_unique<KF2D>(Q, R_nominal_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto sensor_qos = rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort();
        // Latched: the seed is published exactly once by
        // pose_estimator_lidar_node, so this node must still receive it if it
        // starts (or restarts) after Phase 2 has completed.
        auto latched_qos =
            rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local();
        auto pub_qos = rclcpp::QoS{rclcpp::KeepLast{1}}.reliable();

        init_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "/map_odom_init", latched_qos,
            std::bind(&GlobalNavKF2DNode::init_callback, this,
                      std::placeholders::_1));

        aruco_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aruco_rover_pos", sensor_qos,
            std::bind(&GlobalNavKF2DNode::aruco_callback, this,
                      std::placeholders::_1));

        recovery_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/perception/use_camera_aruco_position", latched_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool previous = camera_recovery_active_.exchange(msg->data);
                if (previous != msg->data) {
                    consecutive_rejects_ = 0;
                    RCLCPP_WARN(get_logger(),
                        "Global ArUco recovery gate is now %s",
                        msg->data ? "ENABLED" : "DISABLED");
                }
            });

        map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/global_nav_kf/map_odom", pub_qos);

        last_predict_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / broadcast_rate_hz_),
            std::bind(&GlobalNavKF2DNode::timer_callback, this));

        RCLCPP_INFO(get_logger(),
            "global_nav_kf_2d started: rate=%.1f Hz, meas sigma=(%.3f m, %.2f deg), "
            "process sigma=(%.4f m/s, %.3f deg/s), chi2 gate=%.3f. "
            "Waiting for /map_odom_init before broadcasting map->odom.",
            broadcast_rate_hz_, meas_sigma_xy_m, meas_sigma_yaw_deg,
            process_sigma_xy_m_per_s, process_sigma_yaw_deg_per_s, gate_chi2_);
    }

private:
    /* ================================================================ */
    /*  Seed from pose_estimator_lidar_node's Phase-2 transform         */
    /* ================================================================ */
    void init_callback(
        const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        if (initialized_) {
            RCLCPP_WARN(get_logger(),
                "Ignoring /map_odom_init: the filter is already seeded");
            return;
        }

        const auto &t = msg->transform.translation;
        const double yaw = tf2::getYaw(msg->transform.rotation);
        if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(yaw)) {
            RCLCPP_ERROR(get_logger(),
                "Ignoring /map_odom_init: non-finite transform");
            return;
        }

        kf_->reset(Vec3(t.x, t.y, yaw));
        initialized_ = true;
        last_predict_time_ = now();

        RCLCPP_INFO(get_logger(),
            "Seeded from /map_odom_init: map->odom t=(%.3f, %.3f) yaw=%.2f deg. "
            "Taking over the map->odom broadcast.",
            t.x, t.y, yaw * 180.0 / M_PI);

        // Broadcast straight away: pose_estimator_lidar_node has already
        // stopped, so waiting for the next timer tick would leave a TF gap.
        broadcast();
    }

    /* ================================================================ */
    /*  odom -> base_link at the measurement stamp                      */
    /* ================================================================ */
    bool lookup_odom_base(
        const builtin_interfaces::msg::Time &stamp,
        tf2::Transform &T_odom_base)
    {
        const bool zero_stamp = (stamp.sec == 0 && stamp.nanosec == 0u);
        geometry_msgs::msg::TransformStamped tf;

        try {
            if (!zero_stamp) {
                const rclcpp::Time t(stamp, get_clock()->get_clock_type());
                if (tf_buffer_->canTransform(
                        "odom", "base_link", t,
                        rclcpp::Duration::from_seconds(0.05))) {
                    tf = tf_buffer_->lookupTransform("odom", "base_link", t);
                    tf2::fromMsg(tf.transform, T_odom_base);
                    return true;
                }
            }
            if (!tf_buffer_->canTransform(
                    "odom", "base_link", tf2::TimePointZero,
                    tf2::durationFromSec(0.05))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Reject ArUco pose: odom -> base_link not available yet");
                return false;
            }
            tf = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
            tf2::fromMsg(tf.transform, T_odom_base);
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject ArUco pose: odom -> base_link lookup failed: %s",
                ex.what());
            return false;
        }
    }

    /* ================================================================ */
    /*  Per-measurement noise, in the measured T_map_base frame          */
    /*                                                                  */
    /*  pose_estimator_lidar_node publishes P = (sum H_i^T R_i^-1 H_i)^-1 */
    /*  over its solver inliers, inflated by the chi2/dof of the fit.    */
    /*  That block is anisotropic and has non-zero x-yaw / y-yaw terms,  */
    /*  so all nine entries are read.  A message whose block is not a    */
    /*  usable covariance (non-finite, non-positive diagonal, or not     */
    /*  positive definite) falls back to the nominal R.                  */
    /* ================================================================ */
    // Not const: the throttled warning below needs a non-const Clock&, which
    // Node::get_clock() only returns from a non-const context.
    Mat3 measurement_noise(const nav_msgs::msg::Odometry &msg)
    {
        // 6x6 row-major over [x y z roll pitch yaw]: the x/y/yaw block.
        Mat3 R;
        R << msg.pose.covariance[0],  msg.pose.covariance[1],  msg.pose.covariance[5],
             msg.pose.covariance[6],  msg.pose.covariance[7],  msg.pose.covariance[11],
             msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35];

        if (!R.allFinite() ||
            R(0, 0) <= 0.0 || R(1, 1) <= 0.0 || R(2, 2) <= 0.0) {
            return R_nominal_;
        }

        // Rounding in transport can leave the block very slightly asymmetric.
        R = 0.5 * (R + R.transpose()).eval();
        if (!R.ldlt().isPositive()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "ArUco pose covariance is not positive definite, using the "
                "nominal R instead");
            return R_nominal_;
        }
        return R;
    }

    /* ================================================================ */
    /*  ... propagated into the T_map_odom state frame                   */
    /*                                                                  */
    /*  z = T_map_base * inverse(T_odom_base), i.e. with u = t_odom_base */
    /*  and psi_mo = psi_mb - psi_ob:                                    */
    /*                                                                  */
    /*      t_mo = t_mb - R(psi_mo) * u                                  */
    /*                                                                  */
    /*  so the measured yaw enters map->odom xy through the lever arm u. */
    /*  The coupling grows with |u|, i.e. with how far the rover has     */
    /*  driven from the odom origin: at 40 m of odom travel a 3 deg yaw  */
    /*  sigma is worth ~2 m of xy uncertainty, which the gate would      */
    /*  otherwise never see.                                             */
    /* ================================================================ */
    static Mat3 measurement_noise_map_odom(
        const Mat3 &R_map_base,
        const tf2::Transform &T_odom_base,
        double yaw_map_odom)
    {
        const double ux = T_odom_base.getOrigin().x();
        const double uy = T_odom_base.getOrigin().y();
        const double c = std::cos(yaw_map_odom);
        const double s = std::sin(yaw_map_odom);

        Mat3 J = Mat3::Identity();
        J(0, 2) =  s * ux + c * uy;
        J(1, 2) = -c * ux + s * uy;
        return J * R_map_base * J.transpose();
    }

    /* ================================================================ */
    /*  Measurement update from /aruco_rover_pos                        */
    /* ================================================================ */
    void aruco_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Before the seed arrives these messages are the init-phase samples
        // (rover assumed at erc_start_pos), not a solved global pose.
        if (!initialized_) return;

        if (msg->header.frame_id != "map") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject ArUco pose: expected map-frame measurement, got frame '%s'",
                msg->header.frame_id.c_str());
            return;
        }

        const auto &p = msg->pose.pose.position;
        const double map_base_yaw = tf2::getYaw(msg->pose.pose.orientation);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(map_base_yaw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject ArUco pose: non-finite map-frame pose");
            return;
        }

        tf2::Transform T_odom_base;
        if (!lookup_odom_base(msg->header.stamp, T_odom_base)) return;

        // z = T_map_odom = T_map_base * inverse(T_odom_base), planar.
        tf2::Quaternion q_map_base;
        q_map_base.setRPY(0.0, 0.0, map_base_yaw);
        const tf2::Transform T_map_base(
            q_map_base, tf2::Vector3(p.x, p.y, 0.0));
        const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        const tf2::Vector3 &t = T_map_odom.getOrigin();
        const Vec3 z(t.x(), t.y(), tf2::getYaw(T_map_odom.getRotation()));
        if (!z.allFinite()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject ArUco pose: non-finite map->odom measurement");
            return;
        }

        predict_to_now();

        const Mat3 R = measurement_noise_map_odom(
            measurement_noise(*msg), T_odom_base, z(2));
        const Vec3 y = kf_->innovation(z);
        const double chi2 = kf_->chi2(y, R);
        if (!std::isfinite(chi2) || chi2 > gate_chi2_) {
            ++consecutive_rejects_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Reject ArUco pose (chi2=%.2f > %.2f, %d in a row): "
                "innovation=(%.3f m, %.3f m, %.2f deg), "
                "meas sigma=(%.3f m, %.2f deg)",
                chi2, gate_chi2_, consecutive_rejects_,
                y(0), y(1), y(2) * 180.0 / M_PI,
                std::sqrt(R(0, 0)), std::sqrt(R(2, 2)) * 180.0 / M_PI);

            // A badly seeded filter would otherwise reject every measurement
            // forever; widening P lets consistent measurements back in.
            if (max_consecutive_rejects_ > 0 &&
                consecutive_rejects_ >= max_consecutive_rejects_) {
                const bool inflation_allowed =
                    !inflate_only_in_camera_recovery_ ||
                    camera_recovery_active_.load();
                consecutive_rejects_ = 0;
                if (inflation_allowed) {
                    kf_->inflate(4.0);
                    RCLCPP_WARN(get_logger(),
                        "Inflating map->odom covariance after %d consistent-cycle "
                        "rejections in verified recovery mode",
                        max_consecutive_rejects_);
                } else {
                    RCLCPP_ERROR(get_logger(),
                        "Not inflating map->odom covariance after %d rejected ArUco "
                        "poses because camera recovery is not active",
                        max_consecutive_rejects_);
                }
            }
            return;
        }

        consecutive_rejects_ = 0;
        kf_->update(y, R);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Accepted ArUco pose (chi2=%.2f): meas sigma=(%.3f m, %.2f deg)",
            chi2, std::sqrt(R(0, 0)), std::sqrt(R(2, 2)) * 180.0 / M_PI);
    }

    /* ================================================================ */
    /*  Periodic propagation + broadcast                                */
    /* ================================================================ */
    void timer_callback()
    {
        if (!initialized_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for /map_odom_init: map->odom is not being broadcast");
            return;
        }
        predict_to_now();
        broadcast();

        const Vec3 &x = kf_->state();
        const Mat3 &P = kf_->covariance();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "map->odom=(%.3f, %.3f, %.2f deg) sigma=(%.3f m, %.3f m, %.2f deg)",
            x(0), x(1), x(2) * 180.0 / M_PI,
            std::sqrt(P(0, 0)), std::sqrt(P(1, 1)),
            std::sqrt(P(2, 2)) * 180.0 / M_PI);
    }

    void predict_to_now()
    {
        const rclcpp::Time t_now = now();
        const double dt = (t_now - last_predict_time_).seconds();
        last_predict_time_ = t_now;
        kf_->predict(dt);
    }

    void broadcast()
    {
        const Vec3 &x = kf_->state();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x(2));

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";
        tf_msg.transform.translation.x = x(0);
        tf_msg.transform.translation.y = x(1);
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);

        const Mat3 &P = kf_->covariance();
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = tf_msg.header;
        odom_msg.child_frame_id = "odom";
        odom_msg.pose.pose.position.x = x(0);
        odom_msg.pose.pose.position.y = x(1);
        odom_msg.pose.pose.orientation = tf_msg.transform.rotation;
        // 6x6 row-major over [x y z roll pitch yaw]: fill the x/y/yaw block.
        odom_msg.pose.covariance[0]  = P(0, 0);
        odom_msg.pose.covariance[1]  = P(0, 1);
        odom_msg.pose.covariance[5]  = P(0, 2);
        odom_msg.pose.covariance[6]  = P(1, 0);
        odom_msg.pose.covariance[7]  = P(1, 1);
        odom_msg.pose.covariance[11] = P(1, 2);
        odom_msg.pose.covariance[30] = P(2, 0);
        odom_msg.pose.covariance[31] = P(2, 1);
        odom_msg.pose.covariance[35] = P(2, 2);
        map_odom_pub_->publish(odom_msg);
    }

    /* ---- filter ---- */
    std::unique_ptr<KF2D> kf_;
    bool initialized_{false};
    int  consecutive_rejects_{0};
    rclcpp::Time last_predict_time_;

    /* ---- parameters ---- */
    double broadcast_rate_hz_{20.0};
    Mat3   R_nominal_{Mat3::Zero()};
    double gate_chi2_{16.27};
    int    max_consecutive_rejects_{10};
    bool   inflate_only_in_camera_recovery_{true};

    /* ---- ROS ---- */
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr init_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr aruco_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_mode_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic_bool camera_recovery_active_{false};

    /* ---- TF ---- */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalNavKF2DNode>());
    rclcpp::shutdown();
    return 0;
}
