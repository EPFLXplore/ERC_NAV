#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <Eigen/Dense>

#include <cmath>
#include <optional>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <mutex>

#ifdef HAS_ECOS
extern "C" {
#include "ecos.h"
}
#endif

// CHANGE LANDMARK AND ERC_START_POS

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

static inline builtin_interfaces::msg::Time stamp_now(rclcpp::Node *node)
{
    builtin_interfaces::msg::Time t;
    int64_t ns = node->now().nanoseconds();
    t.sec  = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
}

static inline double wrap(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
    double hz = yaw * 0.5;
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(hz);
    q.w = std::cos(hz);
    return q;
}

static double quat_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R = eq.toRotationMatrix();
    return std::atan2(R(1, 0), R(0, 0));
}

/* ------------------------------------------------------------------ */
/*  Measurement for the SOCP solver                                   */
/* ------------------------------------------------------------------ */

struct Measurement {
    double ax, ay;   // landmark absolute position
    double range;    // measured range
    double vx, vy;   // bearing unit vector (pointing from landmark to rover)
};

struct ValidMarker {
    int landmark_index;
    int msg_index;
    double base_x;
    double base_y;
    double range;
    double bearing_rad;
};

namespace {

/* When ECOS_setup fails (AMD, infeasible model, or lib/header mismatch), use the
 * centroid of p_k = a_k + r_k * v_k as a coarse map position (v: landmark→rover). */
std::optional<std::pair<double, double>>
fallback_xy_range_bearing_centroid(const std::vector<Measurement> &meas)
{
    if (meas.size() < 2)
        return std::nullopt;
    double sx = 0.0, sy = 0.0;
    for (const auto &m : meas) {
        sx += m.ax + m.range * m.vx;
        sy += m.ay + m.range * m.vy;
    }
    const double inv = 1.0 / static_cast<double>(meas.size());
    return std::make_pair(sx * inv, sy * inv);
}

} // namespace

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */

class PoseEstimatorLidarNode : public rclcpp::Node
{
public:
    /* ---- two-phase init state ---- */
    // CAMERA: initialise map->odom from /aruco_markers (camera bearings only).
    // CUBE  : re-iterate using merged cube markers (see below).
    //         This phase requires the Phase-1 map->odom to be broadcast so that
    //         lidar_phi_filter and detect_cube can label cubes correctly.
    // DONE  : continuous post-init updates from merged cube markers.
    //
    // Cube inputs: /cube_markers (detect_cube, batched LiDAR centres) and
    // /cube_markers_phi (lidar_phi_filter, fast camera fallback e.g. drill sector).
    // Both published to the same topic would interleave; the pose node merges
    // the last message from each within CUBE_MERGE_TTL_SEC (prefer detect_cube
    // when both have the same marker id).
    enum class InitPhase { CAMERA, CUBE, DONE };

    PoseEstimatorLidarNode() : Node("pose_estimator_lidar_node")
    {
        /* ---- state ---- */
        x_estimate_ = 0.0;
        y_estimate_ = 0.0;
        yaw_estimate_ = 0.0;
        solved_new_xy_ = false;
        measured_new_yaw_ = false;
        time_of_last_pose_ = now();
        time_of_last_yaw_meas_ = now();

        /* ---- landmark_poses_ positions ---- */
        // In constructor, replace the hardcoded landmark_poses_ = {...} with:
        declare_parameter<std::vector<double>>("landmark_poses", std::vector<double>{});
        auto flat_landmarks = get_parameter("landmark_poses").as_double_array();

        landmark_poses_.clear();
        for (size_t i = 0; i + 1 < flat_landmarks.size(); i += 2) {
            landmark_poses_.emplace_back(flat_landmarks[i], flat_landmarks[i + 1]);
        }

        declare_parameter<std::vector<double>>("erc_start_pos", {0.0, 0.0});
        auto flat_erc = get_parameter("erc_start_pos").as_double_array();

        if (flat_erc.size() == 2) {
            erc_start_pos_ = {flat_erc[0], flat_erc[1]};
        } else {
            RCLCPP_ERROR(get_logger(), "erc_start_pos must have exactly 2 values, got %zu", flat_erc.size());
        }
        /* ---- init averaging ---- */
        init_phase_ = InitPhase::CAMERA;
        init_counter_phase1_ = 0;
        init_counter_phase2_ = 0;
        last_callback_time_ = now();

        /* ---- callback groups ---- */
        solver_cbg_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rt_cbg_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        /* ---- subscribers ---- */
        rclcpp::QoS sensor_qos(10);
        sensor_qos.best_effort();

        rclcpp::SubscriptionOptions solver_opts;
        solver_opts.callback_group = solver_cbg_;
        // Phase 1 input: camera-only ArUco detections from multiview_aruco_node.
        aruco_sub_ = create_subscription<
            ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers",
            sensor_qos,
            std::bind(&PoseEstimatorLidarNode::aruco_callback, this,
                      std::placeholders::_1),
            solver_opts);
        // Phase 2+ : detect_cube (batched) + lidar_phi_filter (fast single-marker).
        cube_sub_ = create_subscription<
            ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/cube_markers",
            sensor_qos,
            std::bind(&PoseEstimatorLidarNode::cube_detect_callback, this,
                      std::placeholders::_1),
            solver_opts);
        cube_phi_sub_ = create_subscription<
            ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/cube_markers_phi",
            sensor_qos,
            std::bind(&PoseEstimatorLidarNode::cube_phi_callback, this,
                      std::placeholders::_1),
            solver_opts);

        rclcpp::QoS ekf_qos(1);
        ekf_qos.best_effort();

        rclcpp::SubscriptionOptions rt_opts;
        rt_opts.callback_group = rt_cbg_;
        ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fused_nav_ekf_odom",
            ekf_qos,
            std::bind(&PoseEstimatorLidarNode::ekf_callback, this,
                      std::placeholders::_1),
            rt_opts);

        /* ---- publisher ---- */
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/aruco_rover_pos", ekf_qos);

        /* ---- TF ---- */
        tf_buffer_  = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        tf_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PoseEstimatorLidarNode::republish_tf, this),
            rt_cbg_);
    }

private:
    /* ================================================================ */
    /*  Constants & state                                               */
    /* ================================================================ */
    static constexpr double MAP_SIZE           = 300.0;
    /** Max age (seconds) of last /cube_markers vs /cube_markers_phi to merge. */
    static constexpr double CUBE_MERGE_TTL_SEC = 0.65;
    // Phase 1: camera-bearings-only init samples.
    static constexpr int    NBR_INIT_CALLBACKS_PHASE1 = 15;
    // Phase 2: cube-refined init samples (after Phase-1 TF is broadcast).
    static constexpr int    NBR_INIT_CALLBACKS_PHASE2 = 15;
    static constexpr double CALLBACK_PERIOD_LIMIT = 1.0 / 15.0;
    static constexpr double MAX_TRANSLATION_JUMP  = 0.8;
    static constexpr double MAX_YAW_JUMP = 45.0 * M_PI / 180.0;
    /* Previously used to throttle yaw; throttling after a fresh (x,y) solve
     * leaves heading inconsistent with position (wrong map→odom yaw). */

    static constexpr double LAMBDA_BEARING = 9999.0; //999999.0
    static constexpr double MAP_XMIN = -60.0;
    static constexpr double MAP_XMAX =  60.0;
    static constexpr double MAP_YMIN = -60.0;
    static constexpr double MAP_YMAX =  60.0;

    std::array<double, 2> erc_start_pos_;
    std::vector<std::pair<double, double>> landmark_poses_;

    /* ---- pose state ---- */
    double x_estimate_, y_estimate_, yaw_estimate_;
    bool   solved_new_xy_, measured_new_yaw_;
    rclcpp::Time time_of_last_pose_;
    rclcpp::Time time_of_last_yaw_meas_;

    /* ---- EKF odom state ---- */
    double odom_pos_x_{0.0}, odom_pos_y_{0.0}, odom_yaw_{0.0};

    /* ---- current map->base_link estimate ---- */
    double curr_map_base_x_{0.0}, curr_map_base_y_{0.0},
           curr_map_base_yaw_{0.0};

    /* ---- init accumulation ---- */
    InitPhase init_phase_;
    int  init_counter_phase1_;
    int  init_counter_phase2_;
    rclcpp::Time last_callback_time_;
    /** Last accepted handle_marker_message valid marker count (post-init rate-limit bypass). */
    int last_handled_valid_marker_count_{-1};
    std::vector<geometry_msgs::msg::TransformStamped> phase1_tfs_;
    std::vector<double> phase1_yaws_;
    std::vector<geometry_msgs::msg::TransformStamped> phase2_tfs_;
    std::vector<double> phase2_yaws_;

    /* ---- TF ---- */
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::optional<geometry_msgs::msg::TransformStamped> prev_map_odom_tf_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    /* ---- ROS ---- */
    rclcpp::CallbackGroup::SharedPtr solver_cbg_, rt_cbg_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr
        aruco_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr
        cube_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr
        cube_phi_sub_;

    std::mutex cube_merge_mutex_;
    ros2_aruco_interfaces::msg::ArucoMarkers last_cube_detect_;
    ros2_aruco_interfaces::msg::ArucoMarkers last_cube_phi_;
    bool have_last_cube_detect_{false};
    bool have_last_cube_phi_{false};
    double last_cube_detect_recv_sec_{-1.0};
    double last_cube_phi_recv_sec_{-1.0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;

    /* ================================================================ */
    /*  Helper: 4×4 pose matrix from (x, y, yaw)                       */
    /* ================================================================ */
    static Eigen::Matrix4d pose_to_mat(double x, double y, double yaw)
    {
        double c = std::cos(yaw), s = std::sin(yaw);
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M(0, 0) = c;  M(0, 1) = -s;
        M(1, 0) = s;  M(1, 1) =  c;
        M(0, 3) = x;  M(1, 3) = y;
        return M;
    }

    /* ================================================================ */
    /*  Build map→odom TF from map-frame pose                          */
    /* ================================================================ */
    geometry_msgs::msg::TransformStamped build_map_odom_tf(
        double map_x, double map_y, double map_yaw)
    {
        Eigen::Matrix4d T_map_base =
            pose_to_mat(map_x, map_y, map_yaw);
        Eigen::Matrix4d T_odom_base =
            pose_to_mat(odom_pos_x_, odom_pos_y_, odom_yaw_);
        Eigen::Matrix4d T_map_odom =
            T_map_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp_now(this);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id  = "odom";
        tf_msg.transform.translation.x = T_map_odom(0, 3);
        tf_msg.transform.translation.y = T_map_odom(1, 3);
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation =
            yaw_to_quat(std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
        return tf_msg;
    }

    /* ================================================================ */
    /*  Circular mean of yaw angles                                     */
    /* ================================================================ */
    static double circular_mean_yaw(const std::vector<double> &yaws)
    {
        double sx = 0.0, cx = 0.0;
        for (double y : yaws) { sx += std::sin(y); cx += std::cos(y); }
        return std::atan2(sx, cx);
    }

    /* ================================================================ */
    /*  Deduce yaw from position + bearings                             */
    /* ================================================================ */
    double deduce_yaw(
        double est_x, double est_y,
        const std::vector<ValidMarker> &valid_markers,
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg) const
    {
        (void)msg;
        std::vector<double> yaw_list;
        yaw_list.reserve(valid_markers.size());
        for (const auto &marker : valid_markers) {
            auto &lm = landmark_poses_[marker.landmark_index];
            double bearing_map =
                std::atan2(lm.second - est_y, lm.first - est_x);
            double yaw_component = wrap(bearing_map - marker.bearing_rad);
            RCLCPP_INFO(get_logger(),
                "[YAW SOLVER DEBUG] est=(%.3f, %.3f) id=%d "
                "lm=(%.3f, %.3f) bearing_map=%.2f deg "
                "marker_bearing=%.2f deg yaw_component=%.2f deg",
                est_x, est_y, marker.landmark_index,
                lm.first, lm.second,
                bearing_map * 180.0 / M_PI,
                marker.bearing_rad * 180.0 / M_PI,
                yaw_component * 180.0 / M_PI);
            yaw_list.push_back(yaw_component);
        }
        return circular_mean_yaw(yaw_list);
    }

    /* ================================================================ */
    /*  Publish /aruco_rover_pos odometry                               */
    /* ================================================================ */
    void publish_odom(double x, double y, double yaw)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp_now(this);
        msg.header.frame_id = "map";
        msg.child_frame_id  = "base_link";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.orientation = yaw_to_quat(yaw);
        odom_pub_->publish(msg);
    }

    /* ================================================================ */
    /*  EKF odom callback                                               */
    /* ================================================================ */
    void ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_pos_x_ = msg->pose.pose.position.x;
        odom_pos_y_ = msg->pose.pose.position.y;
        odom_yaw_   = quat_to_yaw(msg->pose.pose.orientation);
    }

    /* ================================================================ */
    /*  Periodic TF re-broadcast (20 Hz)                                */
    /* ================================================================ */
    void republish_tf()
    {
        geometry_msgs::msg::TransformStamped tf;
        if (prev_map_odom_tf_.has_value()) {
            tf = prev_map_odom_tf_.value();
        } else {
            tf.header.frame_id = "map";
            tf.child_frame_id  = "odom";
            tf.transform.rotation.w = 1.0;
        }
        tf.header.stamp = stamp_now(this);
        tf_broadcaster_->sendTransform(tf);
    }

    /* ================================================================ */
    /*  Refresh current map→base_link from TF chain                    */
    /* ================================================================ */
    void update_curr_map_base()
    {
        try {
            auto tf = tf_buffer_->lookupTransform(
                "map", "odom", tf2::TimePointZero);
            double tf_yaw = quat_to_yaw(tf.transform.rotation);
            Eigen::Matrix4d T_map_odom = pose_to_mat(
                tf.transform.translation.x,
                tf.transform.translation.y, tf_yaw);
            Eigen::Matrix4d T_odom_base = pose_to_mat(
                odom_pos_x_, odom_pos_y_, odom_yaw_);

            if (std::abs(odom_pos_x_) < 1e-4 &&
                std::abs(odom_pos_y_) < 1e-4) {
                curr_map_base_x_   = x_estimate_;
                curr_map_base_y_   = y_estimate_;
                curr_map_base_yaw_ = yaw_estimate_;
            } else {
                Eigen::Matrix4d T_map_base = T_map_odom * T_odom_base;
                curr_map_base_x_   = T_map_base(0, 3);
                curr_map_base_y_   = T_map_base(1, 3);
                curr_map_base_yaw_ =
                    std::atan2(T_map_base(1, 0), T_map_base(0, 0));
                if (!solved_new_xy_) {
                    x_estimate_   = curr_map_base_x_;
                    y_estimate_   = curr_map_base_y_;
                    yaw_estimate_ = curr_map_base_yaw_;
                }
            }
        } catch (const tf2::TransformException &) {
            if (!prev_map_odom_tf_.has_value()) {
                curr_map_base_x_   = odom_pos_x_;
                curr_map_base_y_   = odom_pos_y_;
                curr_map_base_yaw_ = odom_yaw_;
            }
        }
    }

    /** base_link <- source_frame at ArucoMarkers.header.stamp when non-zero; else latest. */
    bool lookup_tf_base_from_source_at_msg_stamp(
        const std::string &source_frame,
        const builtin_interfaces::msg::Time &stamp,
        geometry_msgs::msg::TransformStamped &tf_out)
    {
        try {
            const bool zero_stamp =
                (stamp.sec == 0u && stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_->canTransform(
                        "base_link", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                    return false;
                }
                tf_out = tf_buffer_->lookupTransform(
                    "base_link", source_frame, tf2::TimePointZero);
                return true;
            }
            const rclcpp::Time t(stamp, get_clock()->get_clock_type());
            if (tf_buffer_->canTransform(
                    "base_link", source_frame, t, rclcpp::Duration::from_seconds(0.05))) {
                tf_out = tf_buffer_->lookupTransform(
                    "base_link", source_frame, t, rclcpp::Duration::from_seconds(0.1));
                return true;
            }
            if (!tf_buffer_->canTransform(
                    "base_link", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                return false;
            }
            tf_out = tf_buffer_->lookupTransform(
                "base_link", source_frame, tf2::TimePointZero);
            return true;
        } catch (const tf2::TransformException &) {
            try {
                if (!tf_buffer_->canTransform(
                        "base_link", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                    return false;
                }
                tf_out = tf_buffer_->lookupTransform(
                    "base_link", source_frame, tf2::TimePointZero);
                return true;
            } catch (const tf2::TransformException &) {
                return false;
            }
        }
    }

    bool marker_position_in_base_link(
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg,
        size_t marker_msg_index,
        double &base_x,
        double &base_y)
    {
        if (marker_msg_index >= msg.poses.size()) {
            return false;
        }

        const auto &p = msg.poses[marker_msg_index].position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            return false;
        }

        const std::string &source_frame = msg.header.frame_id;
        if (source_frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] cube_markers has empty frame_id, skipping marker");
            return false;
        }

        if (source_frame == "base_link") {
            base_x = p.x;
            base_y = p.y;
            return std::isfinite(base_x) && std::isfinite(base_y);
        }

        geometry_msgs::msg::TransformStamped tf;
        if (!lookup_tf_base_from_source_at_msg_stamp(source_frame, msg.header.stamp, tf)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] TF unavailable %s -> base_link at msg stamp, skipping marker",
                source_frame.c_str());
            return false;
        }

        try {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = source_frame;
            point_in.header.stamp = msg.header.stamp;
            point_in.point = p;
            tf2::doTransform(point_in, point_out, tf);

            base_x = point_out.point.x;
            base_y = point_out.point.y;
            return std::isfinite(base_x) && std::isfinite(base_y);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] Transform %s -> base_link failed: %s",
                source_frame.c_str(), ex.what());
            return false;
        }
    }

    bool marker_bearing_in_base_link(
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg,
        size_t marker_msg_index,
        double &bearing_rad)
    {
        if (marker_msg_index >= msg.ar_angles_list.size()) {
            return false;
        }

        const double source_bearing = msg.ar_angles_list[marker_msg_index] * M_PI / 180.0;
        if (!std::isfinite(source_bearing)) {
            return false;
        }

        const std::string &source_frame = msg.header.frame_id;
        if (source_frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] cube_markers has empty frame_id, skipping bearing");
            return false;
        }

        if (source_frame == "base_link") {
            bearing_rad = wrap(source_bearing);
            return true;
        }

        geometry_msgs::msg::TransformStamped tf;
        if (!lookup_tf_base_from_source_at_msg_stamp(source_frame, msg.header.stamp, tf)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] TF unavailable %s -> base_link at msg stamp, skipping bearing",
                source_frame.c_str());
            return false;
        }

        try {
            geometry_msgs::msg::PointStamped origin_in, origin_out;
            origin_in.header.frame_id = source_frame;
            origin_in.header.stamp = msg.header.stamp;
            origin_in.point.x = 0.0;
            origin_in.point.y = 0.0;
            origin_in.point.z = 0.0;

            geometry_msgs::msg::PointStamped bearing_in, bearing_out;
            bearing_in.header = origin_in.header;
            bearing_in.point.x = std::cos(source_bearing);
            bearing_in.point.y = std::sin(source_bearing);
            bearing_in.point.z = 0.0;

            tf2::doTransform(origin_in, origin_out, tf);
            tf2::doTransform(bearing_in, bearing_out, tf);

            const double dx = bearing_out.point.x - origin_out.point.x;
            const double dy = bearing_out.point.y - origin_out.point.y;
            if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) < 1e-6) {
                return false;
            }

            bearing_rad = std::atan2(dy, dx);
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[pose_estimator_lidar] Bearing transform %s -> base_link failed: %s",
                source_frame.c_str(), ex.what());
            return false;
        }
    }

    /* ================================================================ */
    /*  MAD-based robust average of init TFs                            */
    /* ================================================================ */
    geometry_msgs::msg::TransformStamped calculate_robust_tf_avg(
        const std::vector<geometry_msgs::msg::TransformStamped> &tf_list,
        const std::vector<double> &yaw_list)
    {
        const int N = static_cast<int>(tf_list.size());
        Eigen::MatrixXd t(N, 2);
        for (int i = 0; i < N; ++i) {
            t(i, 0) = tf_list[i].transform.translation.x;
            t(i, 1) = tf_list[i].transform.translation.y;
        }

        /* median */
        auto col_median = [](Eigen::VectorXd v) {
            std::sort(v.data(), v.data() + v.size());
            int n = static_cast<int>(v.size());
            return (n % 2 == 0)
                ? 0.5 * (v[n / 2 - 1] + v[n / 2])
                : v[n / 2];
        };
        double med_x = col_median(t.col(0));
        double med_y = col_median(t.col(1));

        /* MAD */
        Eigen::VectorXd dists(N);
        for (int i = 0; i < N; ++i)
            dists(i) = std::hypot(t(i, 0) - med_x, t(i, 1) - med_y);
        double mad = col_median(dists);
        double thresh = std::max(3.0 * mad, 0.5);

        /* inlier mask */
        int inlier_count = 0;
        double sum_x = 0, sum_y = 0;
        for (int i = 0; i < N; ++i) {
            if (dists(i) < thresh) {
                sum_x += t(i, 0);
                sum_y += t(i, 1);
                ++inlier_count;
            }
        }
        if (inlier_count < 3) {
            sum_x = t.col(0).sum();
            sum_y = t.col(1).sum();
            inlier_count = N;
        }
        double final_x = sum_x / inlier_count;
        double final_y = sum_y / inlier_count;
        double avg_yaw = circular_mean_yaw(yaw_list);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp_now(this);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id  = "odom";
        tf_msg.transform.translation.x = final_x;
        tf_msg.transform.translation.y = final_y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = yaw_to_quat(avg_yaw);

        RCLCPP_INFO(get_logger(),
            "Robust init TF: t=(%.3f, %.3f), yaw=%.2f deg",
            final_x, final_y, avg_yaw * 180.0 / M_PI);
        return tf_msg;
    }

    /* ================================================================ */
    /*  Build measurements for SOCP solver                              */
    /* ================================================================ */
    std::vector<Measurement> build_measurements(
        const std::vector<ValidMarker> &valid_markers,
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg) const
    {
        (void)msg;
        std::vector<Measurement> meas;
        meas.reserve(valid_markers.size());
        for (const auto &marker : valid_markers) {
            double lm_x = landmark_poses_[marker.landmark_index].first;
            double lm_y = landmark_poses_[marker.landmark_index].second;
            if (marker.range < 1e-3) continue;
            meas.push_back({lm_x, lm_y, marker.range,
                            -std::cos(marker.bearing_rad),
                            -std::sin(marker.bearing_rad)});
        }
        return meas;
    }

    std::optional<std::pair<double, double>> solve_range_least_squares(
        const std::vector<Measurement> &meas)
    {
        if (meas.size() < 3) {
            return std::nullopt;
        }

        const auto &ref = meas.front();
        Eigen::MatrixXd A(static_cast<int>(meas.size() - 1), 2);
        Eigen::VectorXd b(static_cast<int>(meas.size() - 1));

        for (size_t i = 1; i < meas.size(); ++i) {
            const auto &m = meas[i];
            A(static_cast<int>(i - 1), 0) = 2.0 * (m.ax - ref.ax);
            A(static_cast<int>(i - 1), 1) = 2.0 * (m.ay - ref.ay);
            b(static_cast<int>(i - 1)) =
                ref.range * ref.range - m.range * m.range +
                m.ax * m.ax - ref.ax * ref.ax +
                m.ay * m.ay - ref.ay * ref.ay;
        }

        const Eigen::Vector2d xy =
            A.colPivHouseholderQr().solve(b);
        if (!xy.allFinite()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Range least-squares failed: non-finite xy");
            return std::nullopt;
        }
        if (xy.x() < MAP_XMIN || xy.x() > MAP_XMAX ||
            xy.y() < MAP_YMIN || xy.y() > MAP_YMAX) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Range least-squares rejected: xy=(%.3f, %.3f) outside map bounds",
                xy.x(), xy.y());
            return std::nullopt;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Range least-squares fallback xy=(%.3f, %.3f)",
            xy.x(), xy.y());
        return std::make_pair(xy.x(), xy.y());
    }

    /* ================================================================ */
    /*  SOCP solver (ECOS C API)                                        */
    /*                                                                  */
    /*  Variables z = [x(2), w(2M), t(M)]  ∈ R^{2+3M}                  */
    /*  Minimise  Σ t_k - ṽ_k'w_k                                     */
    /*  subject to  ||x - a_k - w_k|| ≤ t_k     (M SOC-3 cones)       */
    /*              ||w_k||           ≤ r_k       (M SOC-3 cones)       */
    /*              x ∈ [xmin,xmax] × [ymin,ymax] (4 linear ineqs)    */
    /* ================================================================ */
    std::optional<std::pair<double, double>> solve_ecos(
        const std::vector<Measurement> &meas,
        const char *log_ctx = nullptr) const
    {
#ifndef HAS_ECOS
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "ECOS not compiled in – n>=3 solver disabled");
        (void)meas;
        (void)log_ctx;
        return std::nullopt;
#else
        const char *ctx = (log_ctx && log_ctx[0]) ? log_ctx : "ECOS";
        const int M = static_cast<int>(meas.size());
        if (M < 2) {
            RCLCPP_WARN(get_logger(),
                "[%s] skip solve: M=%d (need M>=2)", ctx, M);
            return std::nullopt;
        }

        for (int k = 0; k < M; ++k) {
            const double vnorm = std::hypot(meas[k].vx, meas[k].vy);
            RCLCPP_INFO(get_logger(),
                "[%s] meas[%d] lm=(%.4f, %.4f) r=%.4f v=(%.5f, %.5f) |v|=%.6f",
                ctx, k, meas[k].ax, meas[k].ay, meas[k].range,
                meas[k].vx, meas[k].vy, vnorm);
        }

        /* ECOS API (embotech): ECOS_setup(n, m, p, l, ...)
         *   n = primal variables, m = rows of G (conic inequalities),
         *   p = equality rows (A), l = linear / positive orthant part.
         * We previously passed (n, 0, p_grows, ...) which swaps m and p
         * and makes ECOS_setup return NULL. */
        const int n_var  = 2 + 3 * M;
        const int m_g    = 4 + 6 * M;   // rows of G
        const int p_eq   = 0;           // no equality constraints A x = b
        const int l_lp   = 4;           // first l_lp rows of h are linear ineqs
        const int ncones = 2 * M;
        const int nnz    = 4 + 7 * M;

        std::vector<pfloat> c(n_var, 0.0);
        std::vector<pfloat> h(m_g, 0.0);
        std::vector<pfloat> Gpr;   Gpr.reserve(nnz);
        std::vector<idxint> Gir;   Gir.reserve(nnz);
        std::vector<idxint> Gjc(static_cast<size_t>(n_var + 1), 0);
        std::vector<idxint> q(ncones, 3);

        /* ---- objective c ---- */
        for (int k = 0; k < M; ++k) {
            double scale = LAMBDA_BEARING / meas[k].range;
            c[2 + 2 * k]     = -scale * meas[k].vx;
            c[2 + 2 * k + 1] = -scale * meas[k].vy;
            c[2 + 2 * M + k] = 1.0;
        }

        /* ---- RHS h ---- */
        h[0] = -MAP_XMIN;   h[1] = MAP_XMAX;
        h[2] = -MAP_YMIN;   h[3] = MAP_YMAX;
        for (int k = 0; k < M; ++k) {
            int b = 4 + 6 * k;
            h[b]     = 0.0;
            h[b + 1] = -meas[k].ax;
            h[b + 2] = -meas[k].ay;
            h[b + 3] = meas[k].range;
            h[b + 4] = 0.0;
            h[b + 5] = 0.0;
        }

        /* ---- G in CCS (column-by-column) ---- */

        /* column 0 : x[0] */
        Gir.push_back(0); Gpr.push_back(-1.0);
        Gir.push_back(1); Gpr.push_back( 1.0);
        for (int k = 0; k < M; ++k) {
            Gir.push_back(4 + 6 * k + 1);
            Gpr.push_back(-1.0);
        }
        Gjc[1] = static_cast<idxint>(Gpr.size());

        /* column 1 : x[1] */
        Gir.push_back(2); Gpr.push_back(-1.0);
        Gir.push_back(3); Gpr.push_back( 1.0);
        for (int k = 0; k < M; ++k) {
            Gir.push_back(4 + 6 * k + 2);
            Gpr.push_back(-1.0);
        }
        Gjc[2] = static_cast<idxint>(Gpr.size());

        /* columns 2..2+2M-1 : w_k[0], w_k[1] */
        for (int k = 0; k < M; ++k) {
            /* w_k[0] */
            Gir.push_back(4 + 6 * k + 1); Gpr.push_back( 1.0);
            Gir.push_back(4 + 6 * k + 4); Gpr.push_back(-1.0);
            Gjc[2 + 2 * k + 1] = static_cast<idxint>(Gpr.size());

            /* w_k[1] */
            Gir.push_back(4 + 6 * k + 2); Gpr.push_back( 1.0);
            Gir.push_back(4 + 6 * k + 5); Gpr.push_back(-1.0);
            Gjc[2 + 2 * k + 2] = static_cast<idxint>(Gpr.size());
        }

        /* columns 2+2M .. 2+3M-1 : t_k */
        for (int k = 0; k < M; ++k) {
            Gir.push_back(4 + 6 * k); Gpr.push_back(-1.0);
            Gjc[2 + 2 * M + k + 1] = static_cast<idxint>(Gpr.size());
        }

        assert(static_cast<int>(Gpr.size()) == nnz);

        idxint sum_q = 0;
        for (int i = 0; i < ncones; ++i)
            sum_q += q[i];
        if (static_cast<idxint>(l_lp) + sum_q != static_cast<idxint>(m_g)) {
            RCLCPP_ERROR(get_logger(),
                "[%s] ECOS dimension mismatch: l_lp=%d + sum(q)=%d != m_g=%d "
                "(header/lib cone layout may not match this build)",
                ctx, l_lp, static_cast<int>(sum_q), m_g);
            auto fb = fallback_xy_range_bearing_centroid(meas);
            if (fb)
                RCLCPP_INFO(get_logger(),
                    "[%s] using centroid fallback xy=(%.4f, %.4f)", ctx,
                    fb->first, fb->second);
            return fb;
        }

        /* ---- ECOS setup ---- */
        pwork *w = ECOS_setup(
            static_cast<idxint>(n_var),
            static_cast<idxint>(m_g),
            static_cast<idxint>(p_eq),
            static_cast<idxint>(l_lp),
            static_cast<idxint>(ncones),
            q.data(),
            static_cast<idxint>(0),
            Gpr.data(), Gjc.data(), Gir.data(),
            nullptr, nullptr, nullptr,
            c.data(), h.data(), nullptr);

        if (!w) {
            RCLCPP_WARN(get_logger(),
                "[%s] ECOS_setup returned NULL (n_var=%d m_g=%d p_eq=%d M=%d) "
                "— check linked libecos vs headers; using centroid fallback",
                ctx, n_var, m_g, p_eq, M);
            auto fb = fallback_xy_range_bearing_centroid(meas);
            if (fb)
                RCLCPP_INFO(get_logger(),
                    "[%s] centroid fallback xy=(%.4f, %.4f)", ctx,
                    fb->first, fb->second);
            return fb;
        }

        w->stgs->verbose  = 0;
        w->stgs->maxit    = 900;
        w->stgs->feastol  = 1e-6;
        w->stgs->reltol   = 1e-6;

        idxint flag = ECOS_solve(w);

        std::optional<std::pair<double, double>> result;
        if (flag == ECOS_OPTIMAL ||
            flag == (ECOS_OPTIMAL + ECOS_INACC_OFFSET)) {
            result = {w->x[0], w->x[1]};
            RCLCPP_INFO(get_logger(),
                "[%s] ECOS_solve flag=%d -> x=(%.4f, %.4f) (OPTIMAL)",
                ctx, static_cast<int>(flag), w->x[0], w->x[1]);
        } else {
            RCLCPP_WARN(get_logger(),
                "[%s] ECOS_solve flag=%d (no optimal solution; "
                "ECOS_OPTIMAL=%d ECOS_INACC_OFFSET=%d)",
                ctx, static_cast<int>(flag),
                static_cast<int>(ECOS_OPTIMAL),
                static_cast<int>(ECOS_INACC_OFFSET));
            auto fb = fallback_xy_range_bearing_centroid(meas);
            if (fb) {
                RCLCPP_INFO(get_logger(),
                    "[%s] using centroid fallback xy=(%.4f, %.4f)", ctx,
                    fb->first, fb->second);
                result = fb;
            }
        }

        ECOS_cleanup(w, 0);
        if (!result.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "ECOS failed for n>=3; using range least-squares fallback");
            result = solve_range_least_squares(meas);
        }
        return result;
#endif
    }

    /* ================================================================ */
    /*  Phase-1 input: camera-only ArUco detections                     */
    /*  Active only while init_phase_ == CAMERA.                        */
    /* ================================================================ */
    void aruco_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (init_phase_ != InitPhase::CAMERA) return;
        handle_marker_message(msg, /*from_camera=*/true);
    }

    /* ================================================================ */
    /*  Merge /cube_markers + /cube_markers_phi → one measurement set   */
    /* ================================================================ */
    ros2_aruco_interfaces::msg::ArucoMarkers
    build_merged_cube_markers_unlocked()
    {
        ros2_aruco_interfaces::msg::ArucoMarkers out;
        const double t = now().seconds();
        auto fresh = [t](double recv_sec) {
            return recv_sec >= 0.0 && (t - recv_sec) <= CUBE_MERGE_TTL_SEC;
        };

        const bool use_det = have_last_cube_detect_ &&
            fresh(last_cube_detect_recv_sec_);
        const bool use_phi = have_last_cube_phi_ &&
            fresh(last_cube_phi_recv_sec_);

        if (use_det) {
            out = last_cube_detect_;
        }
        if (use_phi) {
            const auto &phi = last_cube_phi_;
            for (size_t i = 0; i < phi.marker_ids.size(); ++i) {
                const int64_t id = phi.marker_ids[i];
                if (std::find(out.marker_ids.begin(), out.marker_ids.end(), id)
                    != out.marker_ids.end()) {
                    continue;
                }
                if (i >= phi.poses.size() || i >= phi.ar_angles_list.size())
                    continue;
                out.marker_ids.push_back(id);
                out.poses.push_back(phi.poses[i]);
                out.ar_angles_list.push_back(phi.ar_angles_list[i]);
                if (i < phi.landmark_map_pos_x.size() &&
                    i < phi.landmark_map_pos_y.size()) {
                    out.landmark_map_pos_x.push_back(phi.landmark_map_pos_x[i]);
                    out.landmark_map_pos_y.push_back(phi.landmark_map_pos_y[i]);
                } else if (id >= 0 &&
                    id < static_cast<int64_t>(landmark_poses_.size())) {
                    const size_t li = static_cast<size_t>(id);
                    out.landmark_map_pos_x.push_back(landmark_poses_[li].first);
                    out.landmark_map_pos_y.push_back(landmark_poses_[li].second);
                } else {
                    out.landmark_map_pos_x.push_back(0.0);
                    out.landmark_map_pos_y.push_back(0.0);
                }
            }
        }

        if (!out.marker_ids.empty()) {
            out.header.stamp = stamp_now(this);
            if (out.header.frame_id.empty())
                out.header.frame_id = "base_link";
        }

        return out;
    }

    void dispatch_merged_cube_markers()
    {
        if (init_phase_ == InitPhase::CAMERA)
            return;

        ros2_aruco_interfaces::msg::ArucoMarkers merged;
        {
            std::lock_guard<std::mutex> lk(cube_merge_mutex_);
            merged = build_merged_cube_markers_unlocked();
        }
        if (merged.marker_ids.empty())
            return;

        auto sp = std::make_shared<ros2_aruco_interfaces::msg::ArucoMarkers>(
            std::move(merged));
        handle_marker_message(sp, /*from_camera=*/false);
    }

    /* ================================================================ */
    /*  Phase-2 input + post-init updates: merged cube markers          */
    /*  Ignored while init_phase_ == CAMERA.                            */
    /* ================================================================ */
    void cube_detect_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (init_phase_ == InitPhase::CAMERA) return;
        {
            std::lock_guard<std::mutex> lk(cube_merge_mutex_);
            last_cube_detect_ = *msg;
            have_last_cube_detect_ = true;
            last_cube_detect_recv_sec_ = now().seconds();
        }
        dispatch_merged_cube_markers();
    }

    void cube_phi_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (init_phase_ == InitPhase::CAMERA) return;
        {
            std::lock_guard<std::mutex> lk(cube_merge_mutex_);
            last_cube_phi_ = *msg;
            have_last_cube_phi_ = true;
            last_cube_phi_recv_sec_ = now().seconds();
        }
        dispatch_merged_cube_markers();
    }

    /* ================================================================ */
    /*  Shared processing: validate markers, run init or post-init      */
    /*  update, accumulate samples for the active phase, broadcast TF.  */
    /* ================================================================ */
    void handle_marker_message(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg,
        bool from_camera)
    {
        rclcpp::Time t_now = now();

        update_curr_map_base();

        const char *src_tag = from_camera ? "P1" : "P2";
        const int phase_id = static_cast<int>(init_phase_);

        const size_t n_ids = msg->marker_ids.size();
        const size_t n_poses = msg->poses.size();
        const size_t n_angles = msg->ar_angles_list.size();
        if (n_ids != n_poses || n_ids != n_angles) {
            RCLCPP_WARN(get_logger(),
                "[%s MSG SHAPE] marker_ids=%zu poses=%zu ar_angles_list=%zu "
                "(indices must align; short arrays cause skips)",
                src_tag, n_ids, n_poses, n_angles);
        }

        /* ---- validate markers ---- */
        std::vector<ValidMarker> valid_markers;
        for (size_t k = 0; k < msg->marker_ids.size(); ++k) {
            const int64_t raw_id = msg->marker_ids[k];
            if (k >= msg->poses.size()) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=no_pose_slot",
                    src_tag, k, static_cast<long long>(raw_id));
                continue;
            }
            if (k >= msg->ar_angles_list.size()) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=no_ar_angle_slot",
                    src_tag, k, static_cast<long long>(raw_id));
                continue;
            }

            int idx = static_cast<int>(raw_id);
            if (idx < 0 || idx >= static_cast<int>(landmark_poses_.size())) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=id_out_of_table "
                    "(valid index range 0..%zu)",
                    src_tag, k, static_cast<long long>(raw_id),
                    landmark_poses_.size() - 1);
                continue;
            }
            auto &lm = landmark_poses_[idx];
            if (std::abs(lm.first)  >= MAP_SIZE ||
                std::abs(lm.second) >= MAP_SIZE) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu id=%d reason=landmark_unmapped_or_sentinel "
                    "lm=(%.3f, %.3f)",
                    src_tag, k, idx, lm.first, lm.second);
                continue;
            }

            double base_x = 0.0;
            double base_y = 0.0;
            if (!marker_position_in_base_link(*msg, k, base_x, base_y)) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu id=%d frame=\"%s\" reason=position_to_base_link_failed",
                    src_tag, k, idx, msg->header.frame_id.c_str());
                continue;
            }

            const double range = std::hypot(base_x, base_y);
            if (!std::isfinite(range) || range < 1e-3) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu id=%d reason=bad_range "
                    "base=(%.4f, %.4f) range=%.6g",
                    src_tag, k, idx, base_x, base_y, range);
                continue;
            }

            double bearing_rad = 0.0;
            if (!marker_bearing_in_base_link(*msg, k, bearing_rad)) {
                RCLCPP_INFO(get_logger(),
                    "[%s SKIP marker] msg_i=%zu id=%d frame=\"%s\" reason=bearing_to_base_link_failed",
                    src_tag, k, idx, msg->header.frame_id.c_str());
                continue;
            }

            const double lm_bearing_from_start = std::atan2(
                lm.second - erc_start_pos_[1],
                lm.first - erc_start_pos_[0]);
            const double yaw_from_start = wrap(
                lm_bearing_from_start - bearing_rad);
            RCLCPP_INFO(get_logger(),
                "[%s MARKER DEBUG] phase=%d msg_i=%zu id=%d frame=%s "
                "lm=(%.3f, %.3f) base=(%.3f, %.3f) range=%.3f "
                "bearing=%.2f deg lm_bearing_start=%.2f deg "
                "yaw_from_start=%.2f deg",
                src_tag, phase_id, k, idx, msg->header.frame_id.c_str(),
                lm.first, lm.second, base_x, base_y, range,
                bearing_rad * 180.0 / M_PI,
                lm_bearing_from_start * 180.0 / M_PI,
                yaw_from_start * 180.0 / M_PI);

            valid_markers.push_back({
                idx,
                static_cast<int>(k),
                base_x,
                base_y,
                range,
                bearing_rad
            });
        }

        int n = static_cast<int>(valid_markers.size());
        if (n < 1) return;

        const double dt_since_last =
            (t_now - last_callback_time_).seconds();
        const bool marker_count_increased =
            n > last_handled_valid_marker_count_;
        if (init_phase_ == InitPhase::DONE && !marker_count_increased &&
            dt_since_last < CALLBACK_PERIOD_LIMIT) {
            return;
        }
        last_callback_time_ = t_now;
        last_handled_valid_marker_count_ = n;

        bool is_measurement_valid = false;
        std::optional<geometry_msgs::msg::TransformStamped> transform_msg;

        RCLCPP_INFO(get_logger(),
            "[%s DEBUG] phase=%d raw_markers=%zu valid_markers=%d",
            src_tag, phase_id, msg->marker_ids.size(), n);
        if (msg->marker_ids.size() != static_cast<size_t>(n)) {
            RCLCPP_INFO(get_logger(),
                "[%s FILTER SUMMARY] dropped %zu of %zu raw markers "
                "(see [SKIP marker] lines above)",
                src_tag,
                msg->marker_ids.size() - static_cast<size_t>(n),
                msg->marker_ids.size());
        }

        /* ============================================================ */
        /*  INITIALISATION PHASE (Phase 1 OR Phase 2)                   */
        /*  Same algorithm; only the input source and accumulator       */
        /*  buffer differ.                                              */
        /* ============================================================ */
        if (init_phase_ != InitPhase::DONE) {

            if (n == 1) {
                const auto &mA = valid_markers[0];
                auto &A = landmark_poses_[mA.landmark_index];
                double x0 = erc_start_pos_[0], y0 = erc_start_pos_[1];
                double yawA = wrap(
                    std::atan2(A.second - y0, A.first - x0) - mA.bearing_rad);
                RCLCPP_INFO(get_logger(),
                    "[%s INIT n=1 DEBUG] id=%d lm=(%.3f, %.3f) "
                    "bearing=%.2f deg lm_bearing=%.2f deg yawA=%.2f deg",
                    src_tag, mA.landmark_index, A.first, A.second,
                    mA.bearing_rad * 180.0 / M_PI,
                    std::atan2(A.second - y0, A.first - x0) * 180.0 / M_PI,
                    yawA * 180.0 / M_PI);

                yaw_estimate_ = yawA;
                x_estimate_ = x0;
                y_estimate_ = y0;
                measured_new_yaw_ = true;
                time_of_last_yaw_meas_ = t_now;

                RCLCPP_INFO(get_logger(),
                    "[%s INIT n=1] yaw = %.2f deg",
                    src_tag, yaw_estimate_ * 180.0 / M_PI);

                transform_msg = build_map_odom_tf(x_estimate_, y_estimate_, yaw_estimate_);
                is_measurement_valid = true;

            } else if (n == 2) {
                const auto &mA = valid_markers[0];
                const auto &mB = valid_markers[1];
                auto &A = landmark_poses_[mA.landmark_index];
                auto &B = landmark_poses_[mB.landmark_index];
                double x0 = erc_start_pos_[0], y0 = erc_start_pos_[1];

                double yawA = wrap(
                    std::atan2(A.second - y0, A.first - x0) - mA.bearing_rad);
                double yawB = wrap(
                    std::atan2(B.second - y0, B.first - x0) - mB.bearing_rad);
                RCLCPP_INFO(get_logger(),
                    "[%s INIT n=2 DEBUG] idA=%d bearingA=%.2f deg "
                    "lmBearingA=%.2f deg yawA=%.2f deg | "
                    "idB=%d bearingB=%.2f deg lmBearingB=%.2f deg "
                    "yawB=%.2f deg",
                    src_tag,
                    mA.landmark_index,
                    mA.bearing_rad * 180.0 / M_PI,
                    std::atan2(A.second - y0, A.first - x0) * 180.0 / M_PI,
                    yawA * 180.0 / M_PI,
                    mB.landmark_index,
                    mB.bearing_rad * 180.0 / M_PI,
                    std::atan2(B.second - y0, B.first - x0) * 180.0 / M_PI,
                    yawB * 180.0 / M_PI);
                yaw_estimate_ = wrap(0.5 * (yawA + yawB));
                x_estimate_ = x0;
                y_estimate_ = y0;
                measured_new_yaw_ = true;
                time_of_last_yaw_meas_ = t_now;

                RCLCPP_INFO(get_logger(),
                    "[%s INIT n=2] yaw = %.2f deg",
                    src_tag, yaw_estimate_ * 180.0 / M_PI);

                transform_msg = build_map_odom_tf(x_estimate_, y_estimate_, yaw_estimate_);
                is_measurement_valid = true;

            } else if (n >= 3) {
                auto measurements = build_measurements(valid_markers, *msg);
                if (static_cast<int>(measurements.size()) != n) {
                    RCLCPP_WARN(get_logger(),
                        "[%s INIT n>=%d] build_measurements: valid_markers=%d "
                        "but SOCP M=%zu (markers with range<1e-3 are dropped)",
                        src_tag, n, measurements.size());
                }
                RCLCPP_INFO(get_logger(),
                    "[%s INIT n>=%d] running solver with %zu measurements",
                    src_tag, n, measurements.size());
                auto xy = solve_ecos(measurements, "[P1 INIT n>=3]");

                if (xy.has_value()) {
                    double dx = xy->first  - erc_start_pos_[0];
                    double dy = xy->second - erc_start_pos_[1];
                    double dist_from_start = std::hypot(dx, dy);
                    RCLCPP_INFO(get_logger(),
                        "[%s INIT solver] solution P=(%.3f, %.3f), "
                        "dist_from_start=%.3f m",
                        src_tag, xy->first, xy->second, dist_from_start);
                    if (dist_from_start < 1.0) {
                        x_estimate_ = xy->first;
                        y_estimate_ = xy->second;
                        solved_new_xy_ = true;
                        time_of_last_pose_ = t_now;

                        yaw_estimate_ = deduce_yaw(
                            xy->first, xy->second, valid_markers, *msg);
                        measured_new_yaw_ = true;
                        time_of_last_yaw_meas_ = t_now;

                        RCLCPP_INFO(get_logger(),
                            "[%s INIT n>=%d] P=(%.3f, %.3f), yaw=%.2f deg",
                            src_tag, n, xy->first, xy->second,
                            yaw_estimate_ * 180.0 / M_PI);

                        transform_msg = build_map_odom_tf(
                            x_estimate_, y_estimate_, yaw_estimate_);
                        is_measurement_valid = true;
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[%s INIT] Rejected: solution (%.3f,%.3f) "
                            "too far from start (%.2f m)",
                            src_tag, xy->first, xy->second,
                            dist_from_start);
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[%s INIT n>=%d] solver returned no solution "
                        "(measurements=%zu)",
                        src_tag, n, measurements.size());
                }
            }

        /* ============================================================ */
        /*  POST-INIT CONTINUOUS UPDATES                                */
        /* ============================================================ */
        } else {
            if (n >= 2) {
                auto measurements =
                    build_measurements(valid_markers, *msg);
                if (static_cast<int>(measurements.size()) != n) {
                    RCLCPP_WARN(get_logger(),
                        "[UPDATE n>=%d] build_measurements: valid_markers=%d "
                        "but SOCP M=%zu (markers with range<1e-3 are dropped)",
                        n, measurements.size());
                }
                RCLCPP_INFO(get_logger(),
                    "[UPDATE n>=%d] running solver with %zu measurements",
                    n, measurements.size());
                auto xy = solve_ecos(measurements, "[UPDATE n>=2]");

                if (xy.has_value()) {
                    double dx = xy->first  - curr_map_base_x_;
                    double dy = xy->second - curr_map_base_y_;
                    double jump = std::hypot(dx, dy);
                    RCLCPP_INFO(get_logger(),
                        "[UPDATE solver] solution P=(%.3f, %.3f), "
                        "current=(%.3f, %.3f), jump=%.3f m",
                        xy->first, xy->second,
                        curr_map_base_x_, curr_map_base_y_, jump);

                    if (jump <= MAX_TRANSLATION_JUMP) {
                        x_estimate_ = xy->first;
                        y_estimate_ = xy->second;
                        solved_new_xy_ = true;
                        time_of_last_pose_ = t_now;

                        /* Always recompute yaw when (x,y) change: yaw is
                         * atan2(lm - P) - bearing in base; it must use the same
                         * P as the solver output, not a stale heading. */
                        yaw_estimate_ = deduce_yaw(
                            xy->first, xy->second,
                            valid_markers, *msg);
                        measured_new_yaw_ = true;
                        time_of_last_yaw_meas_ = t_now;
                        RCLCPP_INFO(get_logger(),
                            "[UPDATE] yaw = %.2f deg",
                            yaw_estimate_ * 180.0 / M_PI);
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[UPDATE] Rejected jump: %.2f m candidate=(%.3f, %.3f) current=(%.3f, %.3f)",
                            jump, xy->first, xy->second,
                            curr_map_base_x_, curr_map_base_y_);
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[UPDATE n>=%d] solver returned no solution "
                        "(measurements=%zu)",
                        n, measurements.size());
                }
            }

            if (solved_new_xy_) {
                auto tf = build_map_odom_tf(
                    x_estimate_, y_estimate_, yaw_estimate_);
                prev_map_odom_tf_ = tf;
                transform_msg = tf;
                is_measurement_valid = true;
            }

            if (prev_map_odom_tf_.has_value()) {
                auto tf_out = prev_map_odom_tf_.value();
                tf_out.header.stamp = stamp_now(this);
                tf_broadcaster_->sendTransform(tf_out);
                if (solved_new_xy_) {
                    RCLCPP_INFO(get_logger(),
                        "[UPDATE] x=%.3f, y=%.3f, yaw=%.2f deg",
                        x_estimate_, y_estimate_,
                        yaw_estimate_ * 180.0 / M_PI);
                }
            }
        }

        /* ============================================================ */
        /*  INITIALISATION ACCUMULATION (per-phase)                     */
        /* ============================================================ */
        if (is_measurement_valid && transform_msg.has_value()) {
            if (init_phase_ == InitPhase::CAMERA && from_camera) {
                accumulate_phase1(*transform_msg);
            } else if (init_phase_ == InitPhase::CUBE && !from_camera) {
                accumulate_phase2(*transform_msg);
            }
        }

        /* ---- publish odom ---- */
        if (is_measurement_valid)
            publish_odom(x_estimate_, y_estimate_, yaw_estimate_);

        /* ---- reset per-callback flags ---- */
        solved_new_xy_ = false;
        measured_new_yaw_ = false;
    }

    /* ================================================================ */
    /*  Phase-1 accumulation: camera-only init samples.                 */
    /*  When full, broadcast the robust-averaged TF so that             */
    /*  lidar_phi_filter and detect_cube can produce correctly          */
    /*  labelled cubes. Then transition to Phase 2.                     */
    /* ================================================================ */
    void accumulate_phase1(
        const geometry_msgs::msg::TransformStamped &tf_sample)
    {
        if (init_counter_phase1_ >= NBR_INIT_CALLBACKS_PHASE1) return;

        ++init_counter_phase1_;
        phase1_tfs_.push_back(tf_sample);
        phase1_yaws_.push_back(yaw_estimate_);

        RCLCPP_INFO(get_logger(),
            "[P1 ACCUM %d/%d] yaw=%.2f deg",
            init_counter_phase1_, NBR_INIT_CALLBACKS_PHASE1,
            yaw_estimate_ * 180.0 / M_PI);

        if (init_counter_phase1_ < NBR_INIT_CALLBACKS_PHASE1) return;

        // Phase 1 complete: broadcast the camera-derived map->odom TF
        // so downstream lidar nodes can label cubes correctly.
        prev_map_odom_tf_ =
            calculate_robust_tf_avg(phase1_tfs_, phase1_yaws_);
        tf_broadcaster_->sendTransform(prev_map_odom_tf_.value());
        init_phase_ = InitPhase::CUBE;

        RCLCPP_INFO(get_logger(),
            "PHASE 1 DONE (camera bearings): broadcasting initial "
            "map->odom TF. Switching to PHASE 2 (cube refinement).");
    }

    /* ================================================================ */
    /*  Phase-2 accumulation: lidar-refined cube samples.               */
    /*  Refines the Phase-1 TF using detect_cube output, which is now   */
    /*  reliable because the Phase-1 map->base_link TF is broadcast.    */
    /* ================================================================ */
    void accumulate_phase2(
        const geometry_msgs::msg::TransformStamped &tf_sample)
    {
        if (init_counter_phase2_ >= NBR_INIT_CALLBACKS_PHASE2) return;

        ++init_counter_phase2_;
        phase2_tfs_.push_back(tf_sample);
        phase2_yaws_.push_back(yaw_estimate_);

        RCLCPP_INFO(get_logger(),
            "[P2 ACCUM %d/%d] yaw=%.2f deg",
            init_counter_phase2_, NBR_INIT_CALLBACKS_PHASE2,
            yaw_estimate_ * 180.0 / M_PI);

        if (init_counter_phase2_ < NBR_INIT_CALLBACKS_PHASE2) return;

        // Phase 2 complete: refine TF and enter post-init mode.
        auto refined =
            calculate_robust_tf_avg(phase2_tfs_, phase2_yaws_);

        if (prev_map_odom_tf_.has_value()) {
            const auto &p1 = prev_map_odom_tf_.value().transform;
            const auto &p2 = refined.transform;
            double dxy = std::hypot(
                p2.translation.x - p1.translation.x,
                p2.translation.y - p1.translation.y);
            double dyaw =
                std::fabs(wrap(quat_to_yaw(p2.rotation) -
                               quat_to_yaw(p1.rotation))) * 180.0 / M_PI;
            RCLCPP_INFO(get_logger(),
                "PHASE 2 refinement vs PHASE 1: dxy=%.3f m, dyaw=%.2f deg",
                dxy, dyaw);
        }

        prev_map_odom_tf_ = refined;
        tf_broadcaster_->sendTransform(prev_map_odom_tf_.value());
        init_phase_ = InitPhase::DONE;

        RCLCPP_INFO(get_logger(),
            "PHASE 2 DONE (cube refinement): INITIALIZED map->odom TF. "
            "Rate-limiting to %.1f Hz",
            1.0 / CALLBACK_PERIOD_LIMIT);
    }
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseEstimatorLidarNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
