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
#include <mutex>
#include <unordered_map>
#include <limits>

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

struct ValidMarker {
    int landmark_index;
    int msg_index;
    double base_x;
    double base_y;
    double range;
    double bearing_rad;
};

struct Se2Observation {
    int landmark_index;
    int msg_index;
    Eigen::Vector2d a_map;
    Eigen::Vector2d b_base;
    double range;
    double bearing_rad;
    double scoring_sigma_xy2;
};

struct Se2Solution {
    double x;
    double y;
    double yaw;
    int inlier_count;
    double rms_residual;
    double max_residual;
    double pos_3sigma;
};

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

    static constexpr double SE2_SIGMA_R = 0.03;
    static constexpr double SE2_SIGMA_ALPHA = 2.0 * M_PI / 180.0;
    static constexpr double SE2_SIGMA_LANDMARK = 0.01;
    static constexpr double SE2_HUBER_DELTA = 2.5;
    static constexpr double SE2_CHI2_GATE = 11.83;
    static constexpr double SE2_ABS_RESIDUAL_GATE = 0.15;
    static constexpr double SE2_PAIR_DIST_GATE = 0.15;
    static constexpr double SE2_MIN_PAIR_BASELINE = 0.5;
    static constexpr double SE2_FINAL_RMS_GATE = 0.08;
    static constexpr double SE2_FINAL_MAX_RESIDUAL_GATE = 0.12;
    static constexpr double SE2_FINAL_POS_3SIGMA_GATE = 0.10;
    static constexpr double SE2_MIN_MARKER_SPREAD = 0.5;
    static constexpr double SE2_AMBIGUITY_COST_RATIO = 1.25;
    static constexpr double SE2_SAME_POSE_TRANSLATION = 0.05;
    static constexpr double SE2_SAME_POSE_YAW = 3.0 * M_PI / 180.0;
    static constexpr int SE2_GN_ITERS = 5;

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

    struct ScoredSe2Hypothesis {
        Eigen::Vector2d t{Eigen::Vector2d::Zero()};
        double yaw{0.0};
        std::vector<int> inliers;
        double cost{0.0};
        double rms{std::numeric_limits<double>::infinity()};
    };

    static Eigen::Matrix2d rot2(double yaw)
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        Eigen::Matrix2d R;
        R << c, -s,
             s,  c;
        return R;
    }

    static double huber_cost(double d)
    {
        if (d <= SE2_HUBER_DELTA)
            return 0.5 * d * d;
        return SE2_HUBER_DELTA * (d - 0.5 * SE2_HUBER_DELTA);
    }

    static int required_se2_inliers(int n_obs)
    {
        if (n_obs == 3)
            return 3;
        return std::max(3, (6 * n_obs + 9) / 10);
    }

    std::vector<Se2Observation> build_se2_observations(
        const std::vector<ValidMarker> &valid_markers,
        const char *log_ctx) const
    {
        std::unordered_map<int, int> id_counts;
        for (const auto &m : valid_markers)
            ++id_counts[m.landmark_index];

        std::vector<Se2Observation> obs;
        obs.reserve(valid_markers.size());
        for (const auto &m : valid_markers) {
            const auto count_it = id_counts.find(m.landmark_index);
            if (count_it != id_counts.end() && count_it->second > 1) {
                RCLCPP_INFO(get_logger(),
                    "[%s SE2 SKIP marker] msg_i=%d id=%d reason=duplicate_id_in_callback",
                    log_ctx, m.msg_index, m.landmark_index);
                continue;
            }

            const auto &lm = landmark_poses_[m.landmark_index];
            const Eigen::Vector2d a(lm.first, lm.second);
            const Eigen::Vector2d b(
                m.range * std::cos(m.bearing_rad),
                m.range * std::sin(m.bearing_rad));
            const double sigma_xy2 =
                SE2_SIGMA_R * SE2_SIGMA_R +
                m.range * m.range * SE2_SIGMA_ALPHA * SE2_SIGMA_ALPHA +
                SE2_SIGMA_LANDMARK * SE2_SIGMA_LANDMARK;
            obs.push_back({
                m.landmark_index,
                m.msg_index,
                a,
                b,
                m.range,
                m.bearing_rad,
                std::max(sigma_xy2, 1e-8)
            });
        }
        return obs;
    }

    Eigen::Matrix2d se2_information_matrix(
        const Se2Observation &obs,
        double yaw) const
    {
        const double c = std::cos(obs.bearing_rad);
        const double s = std::sin(obs.bearing_rad);
        Eigen::Matrix<double, 2, 2> J;
        J << c, -obs.range * s,
             s,  obs.range * c;

        Eigen::Matrix2d sigma_ralpha = Eigen::Matrix2d::Zero();
        sigma_ralpha(0, 0) = SE2_SIGMA_R * SE2_SIGMA_R;
        sigma_ralpha(1, 1) = SE2_SIGMA_ALPHA * SE2_SIGMA_ALPHA;

        const Eigen::Matrix2d sigma_b = J * sigma_ralpha * J.transpose();
        Eigen::Matrix2d sigma_e =
            rot2(yaw) * sigma_b * rot2(yaw).transpose();
        sigma_e(0, 0) += SE2_SIGMA_LANDMARK * SE2_SIGMA_LANDMARK;
        sigma_e(1, 1) += SE2_SIGMA_LANDMARK * SE2_SIGMA_LANDMARK;

        double det = sigma_e.determinant();
        if (!std::isfinite(det) || det <= 1e-12) {
            sigma_e += 1e-6 * Eigen::Matrix2d::Identity();
        }
        return sigma_e.inverse();
    }

    Eigen::Vector2d se2_residual(
        const Se2Observation &obs,
        const Eigen::Vector2d &t,
        double yaw) const
    {
        return obs.a_map - t - rot2(yaw) * obs.b_base;
    }

    ScoredSe2Hypothesis score_se2_hypothesis(
        const std::vector<Se2Observation> &obs,
        const Eigen::Vector2d &t,
        double yaw,
        bool full_covariance) const
    {
        ScoredSe2Hypothesis score;
        score.t = t;
        score.yaw = yaw;

        double residual_sq_sum = 0.0;
        for (size_t k = 0; k < obs.size(); ++k) {
            const Eigen::Vector2d e = se2_residual(obs[k], t, yaw);
            const double eps = e.norm();
            const double d2 = full_covariance
                ? std::max(0.0,
                    e.dot(se2_information_matrix(obs[k], yaw) * e))
                : e.squaredNorm() / obs[k].scoring_sigma_xy2;

            if (d2 < SE2_CHI2_GATE && eps < SE2_ABS_RESIDUAL_GATE) {
                score.inliers.push_back(static_cast<int>(k));
                score.cost += huber_cost(std::sqrt(std::max(0.0, d2)));
                residual_sq_sum += e.squaredNorm();
            }
        }

        if (!score.inliers.empty()) {
            score.rms = std::sqrt(
                residual_sq_sum / static_cast<double>(score.inliers.size()));
        }
        return score;
    }

    bool refine_se2_pose(
        const std::vector<Se2Observation> &obs,
        const std::vector<int> &inliers,
        Eigen::Vector2d &t,
        double &yaw) const
    {
        if (inliers.size() < 3)
            return false;

        for (int iter = 0; iter < SE2_GN_ITERS; ++iter) {
            Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            const Eigen::Matrix2d R = rot2(yaw);
            Eigen::Matrix2d S;
            S << 0.0, -1.0,
                 1.0,  0.0;

            for (int idx : inliers) {
                const auto &o = obs[static_cast<size_t>(idx)];
                const Eigen::Vector2d e = se2_residual(o, t, yaw);
                const Eigen::Matrix2d Omega = se2_information_matrix(o, yaw);
                const double d2 =
                    std::max(0.0, e.dot(Omega * e));
                const double d = std::sqrt(d2);
                const double w_rob =
                    (d <= SE2_HUBER_DELTA || d < 1e-12)
                    ? 1.0
                    : SE2_HUBER_DELTA / d;

                const Eigen::Vector2d RSb = R * S * o.b_base;
                Eigen::Matrix<double, 2, 3> H;
                H << -1.0,  0.0, -RSb.x(),
                      0.0, -1.0, -RSb.y();

                A += w_rob * H.transpose() * Omega * H;
                g += w_rob * H.transpose() * Omega * e;
            }

            A += 1e-9 * Eigen::Matrix3d::Identity();
            const Eigen::LDLT<Eigen::Matrix3d> ldlt(A);
            if (ldlt.info() != Eigen::Success)
                return false;

            const Eigen::Vector3d dx = ldlt.solve(-g);
            if (!dx.allFinite())
                return false;

            t.x() += dx.x();
            t.y() += dx.y();
            yaw = wrap(yaw + dx.z());

            if (dx.norm() < 1e-6)
                break;
        }
        return t.allFinite() && std::isfinite(yaw);
    }

    bool compute_se2_pose_covariance(
        const std::vector<Se2Observation> &obs,
        const std::vector<int> &inliers,
        const Eigen::Vector2d &t,
        double yaw,
        double &pos_3sigma) const
    {
        (void)t;
        Eigen::Matrix3d Lambda = Eigen::Matrix3d::Zero();
        const Eigen::Matrix2d R = rot2(yaw);
        Eigen::Matrix2d S;
        S << 0.0, -1.0,
             1.0,  0.0;

        for (int idx : inliers) {
            const auto &o = obs[static_cast<size_t>(idx)];
            const Eigen::Vector2d RSb = R * S * o.b_base;
            Eigen::Matrix<double, 2, 3> H;
            H << -1.0,  0.0, -RSb.x(),
                  0.0, -1.0, -RSb.y();
            Lambda += H.transpose() * se2_information_matrix(o, yaw) * H;
        }

        Lambda += 1e-9 * Eigen::Matrix3d::Identity();
        const Eigen::LDLT<Eigen::Matrix3d> ldlt(Lambda);
        if (ldlt.info() != Eigen::Success)
            return false;

        const Eigen::Matrix3d P =
            ldlt.solve(Eigen::Matrix3d::Identity());
        if (!P.allFinite())
            return false;

        const Eigen::Matrix2d Pxy = P.block<2, 2>(0, 0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(Pxy);
        if (eig.info() != Eigen::Success)
            return false;

        const double lambda_max = eig.eigenvalues().maxCoeff();
        if (!std::isfinite(lambda_max) || lambda_max < 0.0)
            return false;

        pos_3sigma = 3.0 * std::sqrt(lambda_max);
        return std::isfinite(pos_3sigma);
    }

    static double marker_spread(
        const std::vector<Se2Observation> &obs,
        const std::vector<int> &inliers)
    {
        double max_dist = 0.0;
        for (size_t i = 0; i < inliers.size(); ++i) {
            for (size_t j = i + 1; j < inliers.size(); ++j) {
                max_dist = std::max(max_dist,
                    (obs[static_cast<size_t>(inliers[i])].b_base -
                     obs[static_cast<size_t>(inliers[j])].b_base).norm());
            }
        }
        return max_dist;
    }

    std::optional<Se2Solution> solve_robust_se2(
        const std::vector<ValidMarker> &valid_markers,
        const char *log_ctx)
    {
        const char *ctx = (log_ctx && log_ctx[0]) ? log_ctx : "SE2";
        const auto obs = build_se2_observations(valid_markers, ctx);
        const int n_obs = static_cast<int>(obs.size());
        if (n_obs < 3) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: %d unique markers after duplicate-ID filtering (need >=3)",
                ctx, n_obs);
            return std::nullopt;
        }

        std::vector<ScoredSe2Hypothesis> hypotheses;
        for (int i = 0; i < n_obs; ++i) {
            for (int j = i + 1; j < n_obs; ++j) {
                const Eigen::Vector2d da = obs[j].a_map - obs[i].a_map;
                const Eigen::Vector2d db = obs[j].b_base - obs[i].b_base;
                const double da_norm = da.norm();
                const double db_norm = db.norm();
                if (da_norm < SE2_MIN_PAIR_BASELINE ||
                    db_norm < SE2_MIN_PAIR_BASELINE) {
                    continue;
                }
                if (std::fabs(da_norm - db_norm) >= SE2_PAIR_DIST_GATE)
                    continue;

                const double yaw = wrap(
                    std::atan2(da.y(), da.x()) -
                    std::atan2(db.y(), db.x()));
                const Eigen::Matrix2d R = rot2(yaw);
                const Eigen::Vector2d t =
                    0.5 * ((obs[i].a_map - R * obs[i].b_base) +
                           (obs[j].a_map - R * obs[j].b_base));
                auto scored = score_se2_hypothesis(
                    obs, t, yaw, /*full_covariance=*/false);
                if (!scored.inliers.empty())
                    hypotheses.push_back(std::move(scored));
            }
        }

        if (hypotheses.empty()) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: no compatible pairwise hypotheses",
                ctx);
            return std::nullopt;
        }

        auto better = [](const ScoredSe2Hypothesis &a,
                         const ScoredSe2Hypothesis &b) {
            if (a.inliers.size() != b.inliers.size())
                return a.inliers.size() > b.inliers.size();
            if (a.cost != b.cost)
                return a.cost < b.cost;
            return a.rms < b.rms;
        };
        std::sort(hypotheses.begin(), hypotheses.end(), better);

        const ScoredSe2Hypothesis best_initial = hypotheses.front();
        const int required = required_se2_inliers(n_obs);
        if (static_cast<int>(best_initial.inliers.size()) < required) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: best initial inliers=%zu required=%d observations=%d",
                ctx, best_initial.inliers.size(), required, n_obs);
            return std::nullopt;
        }

        Eigen::Vector2d t = best_initial.t;
        double yaw = best_initial.yaw;
        if (!refine_se2_pose(obs, best_initial.inliers, t, yaw)) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: Gauss-Newton refinement failed",
                ctx);
            return std::nullopt;
        }

        ScoredSe2Hypothesis final_score =
            score_se2_hypothesis(obs, t, yaw, /*full_covariance=*/true);
        if (final_score.inliers != best_initial.inliers &&
            static_cast<int>(final_score.inliers.size()) >= required) {
            if (!refine_se2_pose(obs, final_score.inliers, t, yaw)) {
                RCLCPP_WARN(get_logger(),
                    "[%s SE2] rejected: final inlier refinement failed",
                    ctx);
                return std::nullopt;
            }
            final_score =
                score_se2_hypothesis(obs, t, yaw, /*full_covariance=*/true);
        }

        if (static_cast<int>(final_score.inliers.size()) < required) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: final inliers=%zu required=%d observations=%d",
                ctx, final_score.inliers.size(), required, n_obs);
            return std::nullopt;
        }

        double max_residual = 0.0;
        double residual_sq_sum = 0.0;
        for (int idx : final_score.inliers) {
            const Eigen::Vector2d e =
                se2_residual(obs[static_cast<size_t>(idx)], t, yaw);
            const double eps = e.norm();
            max_residual = std::max(max_residual, eps);
            residual_sq_sum += e.squaredNorm();
        }
        const double rms = std::sqrt(
            residual_sq_sum /
            static_cast<double>(final_score.inliers.size()));

        double pos_3sigma = std::numeric_limits<double>::infinity();
        if (!compute_se2_pose_covariance(
                obs, final_score.inliers, t, yaw, pos_3sigma)) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: covariance computation failed",
                ctx);
            return std::nullopt;
        }

        const double spread = marker_spread(obs, final_score.inliers);
        const bool residuals_ok =
            rms < SE2_FINAL_RMS_GATE &&
            max_residual < SE2_FINAL_MAX_RESIDUAL_GATE;
        const bool covariance_ok =
            pos_3sigma < SE2_FINAL_POS_3SIGMA_GATE;
        const bool spread_ok = spread > SE2_MIN_MARKER_SPREAD;

        bool ambiguity_ok = true;
        if (hypotheses.size() > 1) {
            const auto &second = hypotheses[1];
            const bool same_pose =
                (second.t - best_initial.t).norm() < SE2_SAME_POSE_TRANSLATION &&
                std::fabs(wrap(second.yaw - best_initial.yaw)) < SE2_SAME_POSE_YAW;
            const bool cost_clear =
                second.cost > SE2_AMBIGUITY_COST_RATIO * best_initial.cost;
            ambiguity_ok = same_pose || cost_clear;
        }

        if (!residuals_ok || !covariance_ok || !spread_ok || !ambiguity_ok) {
            RCLCPP_WARN(get_logger(),
                "[%s SE2] rejected: inliers=%zu/%d rms=%.4f max=%.4f "
                "pos3sigma=%.4f spread=%.4f ambiguity_ok=%d",
                ctx, final_score.inliers.size(), n_obs, rms, max_residual,
                pos_3sigma, spread, ambiguity_ok ? 1 : 0);
            return std::nullopt;
        }

        RCLCPP_INFO(get_logger(),
            "[%s SE2] accepted: P=(%.3f, %.3f) yaw=%.2f deg "
            "inliers=%zu/%d rms=%.4f max=%.4f pos3sigma=%.4f spread=%.4f",
            ctx, t.x(), t.y(), yaw * 180.0 / M_PI,
            final_score.inliers.size(), n_obs, rms, max_residual,
            pos_3sigma, spread);

        return Se2Solution{
            t.x(),
            t.y(),
            yaw,
            static_cast<int>(final_score.inliers.size()),
            rms,
            max_residual,
            pos_3sigma
        };
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
                RCLCPP_INFO(get_logger(),
                    "[%s INIT n>=%d] running SE(2) range-bearing solver with %d validated markers",
                    src_tag, n, n);
                auto se2 = solve_robust_se2(valid_markers, src_tag);

                if (se2.has_value()) {
                    double dx = se2->x - erc_start_pos_[0];
                    double dy = se2->y - erc_start_pos_[1];
                    double dist_from_start = std::hypot(dx, dy);
                    RCLCPP_INFO(get_logger(),
                        "[%s INIT SE2 solver] solution P=(%.3f, %.3f), "
                        "yaw=%.2f deg dist_from_start=%.3f m",
                        src_tag, se2->x, se2->y,
                        se2->yaw * 180.0 / M_PI, dist_from_start);
                    if (dist_from_start < 1.0) {
                        x_estimate_ = se2->x;
                        y_estimate_ = se2->y;
                        yaw_estimate_ = se2->yaw;
                        solved_new_xy_ = true;
                        time_of_last_pose_ = t_now;

                        measured_new_yaw_ = true;
                        time_of_last_yaw_meas_ = t_now;

                        RCLCPP_INFO(get_logger(),
                            "[%s INIT n>=%d] P=(%.3f, %.3f), yaw=%.2f deg "
                            "inliers=%d rms=%.4f max=%.4f pos3sigma=%.4f",
                            src_tag, n, x_estimate_, y_estimate_,
                            yaw_estimate_ * 180.0 / M_PI, se2->inlier_count,
                            se2->rms_residual, se2->max_residual,
                            se2->pos_3sigma);

                        transform_msg = build_map_odom_tf(
                            x_estimate_, y_estimate_, yaw_estimate_);
                        is_measurement_valid = true;
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[%s INIT] Rejected: solution (%.3f,%.3f) "
                            "too far from start (%.2f m)",
                            src_tag, se2->x, se2->y,
                            dist_from_start);
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[%s INIT n>=%d] SE(2) solver rejected this callback",
                        src_tag, n);
                }
            }

        /* ============================================================ */
        /*  POST-INIT CONTINUOUS UPDATES                                */
        /* ============================================================ */
        } else {
            if (n >= 1) {
                RCLCPP_INFO(get_logger(),
                    "[UPDATE SE2] running post-init SE(2) solver with %d validated markers",
                    n);
                auto se2 = solve_robust_se2(valid_markers, "UPDATE");

                if (se2.has_value()) {
                    double dx = se2->x - curr_map_base_x_;
                    double dy = se2->y - curr_map_base_y_;
                    double jump = std::hypot(dx, dy);
                    RCLCPP_INFO(get_logger(),
                        "[UPDATE SE2 solver] solution P=(%.3f, %.3f), yaw=%.2f deg "
                        "current=(%.3f, %.3f), jump=%.3f m",
                        se2->x, se2->y, se2->yaw * 180.0 / M_PI,
                        curr_map_base_x_, curr_map_base_y_, jump);

                    if (jump <= MAX_TRANSLATION_JUMP) {
                        x_estimate_ = se2->x;
                        y_estimate_ = se2->y;
                        yaw_estimate_ = se2->yaw;
                        solved_new_xy_ = true;
                        measured_new_yaw_ = true;
                        time_of_last_pose_ = t_now;
                        time_of_last_yaw_meas_ = t_now;
                        RCLCPP_INFO(get_logger(),
                            "[UPDATE SE2] accepted after jump gate: inliers=%d "
                            "rms=%.4f max=%.4f pos3sigma=%.4f",
                            se2->inlier_count, se2->rms_residual,
                            se2->max_residual, se2->pos_3sigma);
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[UPDATE SE2] Rejected jump: %.2f m candidate=(%.3f, %.3f) current=(%.3f, %.3f)",
                            jump, se2->x, se2->y,
                            curr_map_base_x_, curr_map_base_y_);
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[UPDATE SE2] solver rejected this callback");
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
