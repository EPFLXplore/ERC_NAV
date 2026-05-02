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

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */

class PoseEstimatorLidarNode : public rclcpp::Node
{
public:
    /* ---- two-phase init state ---- */
    // CAMERA: initialise map->odom from /aruco_markers (camera bearings only).
    // CUBE  : re-iterate using /cube_markers (lidar-refined cube centers).
    //         This phase requires the Phase-1 map->odom to be broadcast so that
    //         lidar_phi_filter and detect_cube can label cubes correctly.
    // DONE  : continuous post-init updates from /cube_markers.
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
        // Phase 2 input + post-init updates: lidar-refined cube centers from detect_cube.
        cube_sub_ = create_subscription<
            ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/cube_markers",
            sensor_qos,
            std::bind(&PoseEstimatorLidarNode::cube_callback, this,
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
            std::chrono::milliseconds(200),
            std::bind(&PoseEstimatorLidarNode::republish_tf, this),
            rt_cbg_);
    }

private:
    /* ================================================================ */
    /*  Constants & state                                               */
    /* ================================================================ */
    static constexpr double MAP_SIZE           = 300.0;
    // Phase 1: camera-bearings-only init samples.
    static constexpr int    NBR_INIT_CALLBACKS_PHASE1 = 15;
    // Phase 2: cube-refined init samples (after Phase-1 TF is broadcast).
    static constexpr int    NBR_INIT_CALLBACKS_PHASE2 = 15;
    static constexpr double CALLBACK_PERIOD_LIMIT = 1.0 / 15.0;
    static constexpr double MAX_TRANSLATION_JUMP  = 0.8;
    static constexpr double MAX_YAW_JUMP = 45.0 * M_PI / 180.0;
    static constexpr double MIN_YAW_DT  = 1.0;

    static constexpr double LAMBDA_BEARING = 999999.0;
    static constexpr double MAP_XMIN = -60.0;
    static constexpr double MAP_XMAX =  60.0;
    static constexpr double MAP_YMIN = -60.0;
    static constexpr double MAP_YMAX =  60.0;

    const std::array<double, 2> erc_start_pos_{0.0, 0.0};

    const std::vector<std::pair<double, double>> landmark_poses_ = {
        {0.85, -0.8},          // id 51
        {999999, 999999},             // id 52
        {1.38, 1.08},       // id 53
        {999999, 999999},       // id 54
        {999999, 999999},       // id 55
        {999999, 999999},       // id 56
        {999999, 999999},       // id 57
        {-1.56, 0.27},       // id 58
        {999999, 999999},       // id 59
        {999999, 999999},       // id 60
        {999999, 999999},       // id 61
        {999999, 999999},       // id 62
        {999999, 999999},       // id 63
    };

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
            yaw_list.push_back(wrap(bearing_map - marker.bearing_rad));
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
    /*  Periodic TF re-broadcast (5 Hz)                                 */
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

        try {
            if (!tf_buffer_->canTransform(
                    "base_link", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[pose_estimator_lidar] TF unavailable %s -> base_link, skipping marker",
                    source_frame.c_str());
                return false;
            }

            const auto tf = tf_buffer_->lookupTransform(
                "base_link", source_frame, tf2::TimePointZero);

            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = source_frame;
            point_in.header.stamp = rclcpp::Time(0);
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

        try {
            if (!tf_buffer_->canTransform(
                    "base_link", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[pose_estimator_lidar] TF unavailable %s -> base_link, skipping bearing",
                    source_frame.c_str());
                return false;
            }

            const auto tf = tf_buffer_->lookupTransform(
                "base_link", source_frame, tf2::TimePointZero);

            geometry_msgs::msg::PointStamped origin_in, origin_out;
            origin_in.header.frame_id = source_frame;
            origin_in.header.stamp = rclcpp::Time(0);
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
        const std::vector<Measurement> &meas) const
    {
#ifndef HAS_ECOS
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "ECOS not compiled in – n>=3 solver disabled");
        (void)meas;
        return std::nullopt;
#else
        const int M = static_cast<int>(meas.size());
        if (M < 2) return std::nullopt;

        const int n     = 2 + 3 * M;
        const int p     = 4 + 6 * M;   // total G rows
        const int l     = 4;            // linear inequality rows
        const int ncones = 2 * M;
        const int nnz   = 4 + 7 * M;

        std::vector<pfloat> c(n, 0.0);
        std::vector<pfloat> h(p, 0.0);
        std::vector<pfloat> Gpr;   Gpr.reserve(nnz);
        std::vector<idxint> Gir;   Gir.reserve(nnz);
        std::vector<idxint> Gjc(n + 1, 0);
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

        /* ---- ECOS setup ---- */
        pwork *w = ECOS_setup(
            n, 0, p, l, ncones, q.data(), 0,
            Gpr.data(), Gjc.data(), Gir.data(),
            nullptr, nullptr, nullptr,
            c.data(), h.data(), nullptr);

        if (!w) return std::nullopt;

        w->stgs->verbose  = 0;
        w->stgs->maxit    = 900;
        w->stgs->feastol  = 1e-6;
        w->stgs->reltol   = 1e-6;

        idxint flag = ECOS_solve(w);

        std::optional<std::pair<double, double>> result;
        if (flag == ECOS_OPTIMAL ||
            flag == (ECOS_OPTIMAL + ECOS_INACC_OFFSET)) {
            result = {w->x[0], w->x[1]};
        }

        ECOS_cleanup(w, 0);
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
    /*  Phase-2 input + post-init updates: lidar-refined cube centers   */
    /*  Ignored while init_phase_ == CAMERA (Phase-1 must finish first  */
    /*  so that detect_cube has a valid map->base_link TF to label      */
    /*  cubes correctly).                                               */
    /* ================================================================ */
    void cube_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if (init_phase_ == InitPhase::CAMERA) return;
        handle_marker_message(msg, /*from_camera=*/false);
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
        double dt_since_last =
            (t_now - last_callback_time_).seconds();
        // Rate-limit only after both init phases are complete.
        if (dt_since_last < CALLBACK_PERIOD_LIMIT &&
            init_phase_ == InitPhase::DONE)
            return;
        last_callback_time_ = t_now;

        update_curr_map_base();

        /* ---- validate markers ---- */
        std::vector<ValidMarker> valid_markers;
        for (size_t k = 0; k < msg->marker_ids.size(); ++k) {
            int idx = static_cast<int>(msg->marker_ids[k]);
            if (idx < 0 || idx >= static_cast<int>(landmark_poses_.size()))
                continue;
            auto &lm = landmark_poses_[idx];
            if (std::abs(lm.first)  >= MAP_SIZE ||
                std::abs(lm.second) >= MAP_SIZE)
                continue;

            double base_x = 0.0;
            double base_y = 0.0;
            if (!marker_position_in_base_link(*msg, k, base_x, base_y))
                continue;

            const double range = std::hypot(base_x, base_y);
            if (!std::isfinite(range) || range < 1e-3)
                continue;

            double bearing_rad = 0.0;
            if (!marker_bearing_in_base_link(*msg, k, bearing_rad))
                continue;

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

        bool is_measurement_valid = false;
        std::optional<geometry_msgs::msg::TransformStamped> transform_msg;

        const char *src_tag = from_camera ? "P1" : "P2";

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
                auto xy = solve_ecos(measurements);

                if (xy.has_value()) {
                    double dx = xy->first  - erc_start_pos_[0];
                    double dy = xy->second - erc_start_pos_[1];
                    if (std::hypot(dx, dy) < 1.0) {
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
                            std::hypot(dx, dy));
                    }
                }
            }

        /* ============================================================ */
        /*  POST-INIT CONTINUOUS UPDATES                                */
        /* ============================================================ */
        } else {
            if (n >= 3) {
                auto measurements =
                    build_measurements(valid_markers, *msg);
                auto xy = solve_ecos(measurements);

                if (xy.has_value()) {
                    double dx = xy->first  - curr_map_base_x_;
                    double dy = xy->second - curr_map_base_y_;
                    double jump = std::hypot(dx, dy);

                    if (jump <= MAX_TRANSLATION_JUMP) {
                        x_estimate_ = xy->first;
                        y_estimate_ = xy->second;
                        solved_new_xy_ = true;
                        time_of_last_pose_ = t_now;

                        double dt_yaw =
                            (t_now - time_of_last_yaw_meas_).seconds();
                        if (dt_yaw >= MIN_YAW_DT) {
                            yaw_estimate_ = deduce_yaw(
                                xy->first, xy->second,
                                valid_markers, *msg);
                            measured_new_yaw_ = true;
                            time_of_last_yaw_meas_ = t_now;
                            RCLCPP_INFO(get_logger(),
                                "[UPDATE] yaw = %.2f deg",
                                yaw_estimate_ * 180.0 / M_PI);
                        }
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[UPDATE] Rejected jump: %.2f m", jump);
                    }
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
