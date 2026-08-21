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
// state equation:        x(t+1) = x(t) + w,  w ~ N(0, Q*s*dt)
// propagate covariance:  P(t+1) = P(t) + Q*s*dt
// measurement equation:  z(t)   = x(t) + v,  v ~ N(0, R)      (H = I)
// compute Kalman gain:   K = P*H^T*(H*P*H^T + R)^-1 = P*(P + R)^-1
// update covariance:     P = (I - K)*P*(I - K)^T + K*R*K^T    (Joseph form)
//
// /aruco_rover_pos measures T_map_base, not the state, so each measurement is
// converted with the live odom->base_link TF:
//
//     z = T_map_odom = T_map_base * inverse(T_odom_base)
//
// The constant-position model is only valid on that quantity: map->odom is the
// slowly drifting correction, whereas map->base_link moves with the rover.
//
// s is a motion-dependent process noise scale derived from the local EKF's
// velocity on /fused_nav_ekf_odom: odom drifts with distance travelled, not
// with wall time, so a parked rover must not be allowed to wander map->odom.
// See process_noise_scale().

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

    /** Constant-position propagation: only the covariance grows.
     *  q_scale in (0, 1] shrinks the process noise while the rover is barely
     *  moving; see GlobalNavKF2DNode::process_noise_scale(). */
    void predict(double dt, double q_scale)
    {
        if (dt > 0.0 && q_scale > 0.0) P_ += Q_ * (q_scale * dt);
    }

    /** Innovation z - x, with the yaw component wrapped. */
    Vec3 innovation(const Vec3 &z) const
    {
        Vec3 y = z - x_;
        y(2) = wrap_angle(y(2));
        return y;
    }

    /** Squared Mahalanobis distance of an innovation (3 DOF). */
    double chi2(const Vec3 &y) const
    {
        const Mat3 S = P_ + R_;
        return y.dot(S.ldlt().solve(y));
    }

    void update(const Vec3 &y)
    {
        const Mat3 S = P_ + R_;
        const Mat3 K = P_ * S.inverse();

        x_ += K * y;
        x_(2) = wrap_angle(x_(2));

        // Joseph form: stays symmetric positive semi-definite even with a
        // suboptimal gain.
        const Mat3 I = Mat3::Identity();
        P_ = (I - K) * P_ * (I - K).transpose() + K * R_ * K.transpose();
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
    Mat3 R_;  // measurement noise covariance
};

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */
class GlobalNavKF2DNode : public rclcpp::Node
{
public:
    GlobalNavKF2DNode() : Node("global_nav_kf_2d")
    {
        broadcast_rate_hz_ = declare_parameter<double>("broadcast_rate_hz", 20.0);
        const double meas_sigma_xy_m =
            declare_parameter<double>("meas_sigma_xy_m", 0.25);
        const double meas_sigma_yaw_deg =
            declare_parameter<double>("meas_sigma_yaw_deg", 5.0);
        const double process_sigma_xy_m_per_s =
            declare_parameter<double>("process_sigma_xy_m_per_s", 0.02);
        const double process_sigma_yaw_deg_per_s =
            declare_parameter<double>("process_sigma_yaw_deg_per_s", 0.5);
        gate_chi2_ = declare_parameter<double>("mahalanobis_gate_chi2", 7.815);
        max_consecutive_rejects_ =
            declare_parameter<int>("max_consecutive_rejects", 10);

        /* ---- motion-adaptive process noise ---- */
        stationary_speed_mps_ =
            declare_parameter<double>("stationary_speed_mps", 0.05);
        const double stationary_yaw_rate_dps =
            declare_parameter<double>("stationary_yaw_rate_dps", 3.0);
        stationary_q_scale_ =
            declare_parameter<double>("stationary_q_scale", 0.05);
        twist_timeout_sec_ =
            declare_parameter<double>("twist_timeout_sec", 0.5);
        stationary_yaw_rate_rps_ = stationary_yaw_rate_dps * M_PI / 180.0;

        if (!(broadcast_rate_hz_ > 0.0) ||
            !(meas_sigma_xy_m > 0.0) || !(meas_sigma_yaw_deg > 0.0) ||
            !(process_sigma_xy_m_per_s > 0.0) ||
            !(process_sigma_yaw_deg_per_s > 0.0)) {
            RCLCPP_ERROR(get_logger(),
                "broadcast_rate_hz and all sigmas must be positive");
            throw std::runtime_error("invalid global_nav_kf parameters");
        }
        if (!(stationary_speed_mps_ > 0.0) ||
            !(stationary_yaw_rate_rps_ > 0.0) ||
            !(twist_timeout_sec_ > 0.0) ||
            !(stationary_q_scale_ > 0.0) || stationary_q_scale_ > 1.0) {
            RCLCPP_ERROR(get_logger(),
                "stationary_speed_mps, stationary_yaw_rate_dps and "
                "twist_timeout_sec must be positive, and stationary_q_scale "
                "must lie in (0, 1] (got %.6f)", stationary_q_scale_);
            throw std::runtime_error("invalid global_nav_kf motion parameters");
        }

        const double meas_sigma_yaw_rad = meas_sigma_yaw_deg * M_PI / 180.0;
        const double process_sigma_yaw_rad_per_s =
            process_sigma_yaw_deg_per_s * M_PI / 180.0;

        Mat3 R = Mat3::Zero();
        R(0, 0) = meas_sigma_xy_m * meas_sigma_xy_m;
        R(1, 1) = meas_sigma_xy_m * meas_sigma_xy_m;
        R(2, 2) = meas_sigma_yaw_rad * meas_sigma_yaw_rad;

        Mat3 Q = Mat3::Zero();
        Q(0, 0) = process_sigma_xy_m_per_s * process_sigma_xy_m_per_s;
        Q(1, 1) = process_sigma_xy_m_per_s * process_sigma_xy_m_per_s;
        Q(2, 2) = process_sigma_yaw_rad_per_s * process_sigma_yaw_rad_per_s;

        kf_ = std::make_unique<KF2D>(Q, R);

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

        ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fused_nav_ekf_odom", sensor_qos,
            std::bind(&GlobalNavKF2DNode::ekf_odom_callback, this,
                      std::placeholders::_1));

        map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/global_nav_kf/map_odom", pub_qos);

        last_predict_time_ = now();
        last_twist_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / broadcast_rate_hz_),
            std::bind(&GlobalNavKF2DNode::timer_callback, this));

        RCLCPP_INFO(get_logger(),
            "global_nav_kf_2d started: rate=%.1f Hz, meas sigma=(%.3f m, %.2f deg), "
            "process sigma=(%.4f m/s, %.3f deg/s), chi2 gate=%.3f. "
            "Process noise scales down to x%.3f below %.3f m/s / %.2f deg/s. "
            "Waiting for /map_odom_init before broadcasting map->odom.",
            broadcast_rate_hz_, meas_sigma_xy_m, meas_sigma_yaw_deg,
            process_sigma_xy_m_per_s, process_sigma_yaw_deg_per_s, gate_chi2_,
            stationary_q_scale_, stationary_speed_mps_, stationary_yaw_rate_dps);
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
    /*  Rover velocity from the local EKF                               */
    /*                                                                  */
    /*  Only magnitudes are used, so it does not matter that the twist   */
    /*  is expressed in the body frame.                                  */
    /* ================================================================ */
    void ekf_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &v = msg->twist.twist.linear;
        const double speed = std::hypot(v.x, v.y);
        const double yaw_rate = msg->twist.twist.angular.z;
        if (!std::isfinite(speed) || !std::isfinite(yaw_rate)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Ignoring EKF twist: non-finite velocity");
            return;
        }

        last_speed_mps_ = speed;
        last_yaw_rate_rps_ = std::fabs(yaw_rate);
        last_twist_time_ = now();
        have_twist_ = true;
    }

    /* ================================================================ */
    /*  Motion-dependent process noise scale                            */
    /*                                                                  */
    /*  map->odom drifts because odom drifts, and odom drift accumulates */
    /*  with distance travelled rather than with wall time.  So Q is     */
    /*  scaled by how fast the rover is actually moving, normalised by   */
    /*  the stationary thresholds:                                       */
    /*                                                                   */
    /*      m       = max(speed/v_thresh, |yaw_rate|/w_thresh)           */
    /*      q_scale = clamp(m, stationary_q_scale, 1.0)                  */
    /*                                                                   */
    /*  At or above the thresholds Q is untouched.  Parked, it collapses */
    /*  to the stationary_q_scale floor, so P stops inflating and the    */
    /*  filter keeps the transform it has instead of drifting toward     */
    /*  whatever the next noisy solve says.  The floor is deliberately   */
    /*  not zero: gyro bias and thermal drift do not stop when the       */
    /*  wheels do, and a frozen P would eventually reject every          */
    /*  measurement through the Mahalanobis gate.                        */
    /*                                                                   */
    /*  Missing or stale twist falls back to full Q — assuming           */
    /*  "stationary" on absent data would silently make the filter       */
    /*  overconfident.                                                   */
    /* ================================================================ */
    double process_noise_scale()
    {
        if (!have_twist_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No /fused_nav_ekf_odom yet: using full process noise");
            return 1.0;
        }
        if ((now() - last_twist_time_).seconds() > twist_timeout_sec_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "/fused_nav_ekf_odom stale (> %.2f s): using full process noise",
                twist_timeout_sec_);
            return 1.0;
        }

        const double motion = std::max(
            last_speed_mps_ / stationary_speed_mps_,
            last_yaw_rate_rps_ / stationary_yaw_rate_rps_);
        return std::clamp(motion, stationary_q_scale_, 1.0);
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

        const Vec3 y = kf_->innovation(z);
        const double chi2 = kf_->chi2(y);
        if (!std::isfinite(chi2) || chi2 > gate_chi2_) {
            ++consecutive_rejects_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Reject ArUco pose (chi2=%.2f > %.2f, %d in a row): "
                "innovation=(%.3f m, %.3f m, %.2f deg)",
                chi2, gate_chi2_, consecutive_rejects_,
                y(0), y(1), y(2) * 180.0 / M_PI);

            // A badly seeded filter would otherwise reject every measurement
            // forever; widening P lets consistent measurements back in.
            if (max_consecutive_rejects_ > 0 &&
                consecutive_rejects_ >= max_consecutive_rejects_) {
                kf_->inflate(4.0);
                consecutive_rejects_ = 0;
                RCLCPP_WARN(get_logger(),
                    "Inflating map->odom covariance after %d consecutive "
                    "rejections", max_consecutive_rejects_);
            }
            return;
        }

        consecutive_rejects_ = 0;
        kf_->update(y);
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
            "map->odom=(%.3f, %.3f, %.2f deg) sigma=(%.3f m, %.3f m, %.2f deg) "
            "speed=%.3f m/s yaw_rate=%.2f deg/s q_scale=%.3f",
            x(0), x(1), x(2) * 180.0 / M_PI,
            std::sqrt(P(0, 0)), std::sqrt(P(1, 1)),
            std::sqrt(P(2, 2)) * 180.0 / M_PI,
            last_speed_mps_, last_yaw_rate_rps_ * 180.0 / M_PI, last_q_scale_);
    }

    void predict_to_now()
    {
        const rclcpp::Time t_now = now();
        const double dt = (t_now - last_predict_time_).seconds();
        last_predict_time_ = t_now;
        last_q_scale_ = process_noise_scale();
        kf_->predict(dt, last_q_scale_);
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
    double gate_chi2_{7.815};
    int    max_consecutive_rejects_{10};
    double stationary_speed_mps_{0.05};
    double stationary_yaw_rate_rps_{3.0 * M_PI / 180.0};
    double stationary_q_scale_{0.05};
    double twist_timeout_sec_{0.5};

    /* ---- latest EKF velocity ---- */
    bool   have_twist_{false};
    double last_speed_mps_{0.0};
    double last_yaw_rate_rps_{0.0};
    double last_q_scale_{1.0};
    rclcpp::Time last_twist_time_;

    /* ---- ROS ---- */
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr init_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr aruco_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

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
