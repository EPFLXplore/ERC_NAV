/*
 * PoseEstimatorLidarNode estimates the rover pose in the map frame using known
 * landmark positions and detected ArUco/cube markers.
 *
 * The node first initializes map->odom in two phases:
 * 1. CAMERA phase: uses /aruco_markers camera bearings to estimate the initial
 *    yaw and pose near erc_start_pos.  This phase is time-bounded by
 *    init_camera_timeout_sec (see below).
 * 2. CUBE phase: uses merged LiDAR/camera cube detections from /cube_markers
 *    and /cube_markers_phi to refine the transform.
 *
 * Phase-1 fallback (ERC start procedure): the rover is placed by hand on its
 * start point and aimed, as accurately as possible, at one known landmark
 * (L15 from S1, L7 from S2).  backup_erc_map_yaw_rad in the S1/S2 params file
 * is the rover map yaw implied by that manual alignment.  If the cameras never
 * see enough landmarks within init_camera_timeout_sec, Phase 1 completes from
 * (erc_start_pos, backup_erc_map_yaw_rad) instead of waiting forever with an
 * identity map->odom, then continues into Phase 2 as usual.
 *
 * After initialization, the node continuously publishes the rover's map-frame
 * pose from valid marker range/bearing measurements with nonlinear
 * optimization.  It stops owning map->odom at the end of Phase 2: the refined
 * transform is published once on the latched /map_odom_init topic and the TF
 * broadcast stops there.  global_nav_kf_2d_node seeds itself from that message
 * and owns map->odom from then on, correcting it with the post-init
 * /aruco_rover_pos measurements.
 *
 * The latest EKF odometry from /fused_nav_ekf_odom is used to build the
 * map->odom transform during initialization, while the estimated map-frame
 * rover pose is published on /aruco_rover_pos.
 */


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
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <mutex>

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

/* Shortest signed angular difference a - b in degrees, in (-180, 180]. */
static inline double ang_diff_deg(double a, double b)
{
    return std::remainder(a - b, 360.0);
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

class RoverPoseVertex final : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override
    {
        _estimate.setZero();
    }

    void oplusImpl(const double *update) override
    {
        _estimate += Eigen::Map<const Eigen::Vector3d>(update);
        _estimate[2] = wrap(_estimate[2]);
    }

    bool read(std::istream &) override { return false; }
    bool write(std::ostream &) const override { return false; }
};

class RangeBearingEdge final
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, RoverPoseVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RangeBearingEdge(
        const Eigen::Vector2d &landmark,
        double measured_range,
        double measured_bearing)
        : landmark_(landmark),
          measured_range_(measured_range),
          measured_bearing_(measured_bearing)
    {
    }

    void computeError() override
    {
        const auto *vertex = static_cast<const RoverPoseVertex *>(_vertices[0]);
        const Eigen::Vector3d &pose = vertex->estimate();
        const double dx = landmark_.x() - pose.x();
        const double dy = landmark_.y() - pose.y();

        _error[0] = std::hypot(dx, dy) - measured_range_;
        _error[1] = wrap(std::atan2(dy, dx) - pose.z() - measured_bearing_);
    }

    void linearizeOplus() override
    {
        const auto *vertex = static_cast<const RoverPoseVertex *>(_vertices[0]);
        const Eigen::Vector3d &pose = vertex->estimate();
        const double dx = landmark_.x() - pose.x();
        const double dy = landmark_.y() - pose.y();
        const double range_sq = dx * dx + dy * dy;
        const double range = std::sqrt(range_sq);

        _jacobianOplusXi.setZero();
        if (range < 1e-9) {
            return;
        }

        _jacobianOplusXi(0, 0) = -dx / range;
        _jacobianOplusXi(0, 1) = -dy / range;
        _jacobianOplusXi(1, 0) = dy / range_sq;
        _jacobianOplusXi(1, 1) = -dx / range_sq;
        _jacobianOplusXi(1, 2) = -1.0;
    }

    bool read(std::istream &) override { return false; }
    bool write(std::ostream &) const override { return false; }

private:
    Eigen::Vector2d landmark_;
    double measured_range_;
    double measured_bearing_;
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

        /* Fallback heading used when Phase 1 sees no landmarks: the rover is
         * aligned by hand towards L15 (from S1) / L7 (from S2), and this is the
         * map yaw that alignment implies.  Non-finite => no backup available. */
        declare_parameter<double>(
            "backup_erc_map_yaw_rad",
            std::numeric_limits<double>::quiet_NaN());
        backup_erc_map_yaw_rad_ =
            get_parameter("backup_erc_map_yaw_rad").as_double();

        /* <= 0 disables the timeout (Phase 1 then waits for cameras forever). */
        declare_parameter<double>("init_camera_timeout_sec", 40.0);
        init_camera_timeout_sec_ =
            get_parameter("init_camera_timeout_sec").as_double();

        if (std::isfinite(backup_erc_map_yaw_rad_)) {
            RCLCPP_INFO(get_logger(),
                "Phase-1 fallback armed: backup_erc_map_yaw_rad=%.5f rad "
                "(%.2f deg) at erc_start_pos=(%.3f, %.3f), timeout=%.1f s",
                backup_erc_map_yaw_rad_,
                backup_erc_map_yaw_rad_ * 180.0 / M_PI,
                erc_start_pos_[0], erc_start_pos_[1],
                init_camera_timeout_sec_);
        } else {
            RCLCPP_WARN(get_logger(),
                "backup_erc_map_yaw_rad not set: Phase 1 has no fallback and "
                "will wait for camera landmarks indefinitely");
        }

        declare_parameter<double>("range_sigma_m", 0.20);
        declare_parameter<double>("bearing_sigma_deg", 5.0);
        range_sigma_m_ = get_parameter("range_sigma_m").as_double();
        const double bearing_sigma_deg = get_parameter("bearing_sigma_deg").as_double();
        bearing_sigma_rad_ = bearing_sigma_deg * M_PI / 180.0;
        if (!std::isfinite(range_sigma_m_) || range_sigma_m_ <= 0.0 ||
            !std::isfinite(bearing_sigma_rad_) || bearing_sigma_rad_ <= 0.0) {
            RCLCPP_ERROR(
                get_logger(),
                "range_sigma_m and bearing_sigma_deg must both be finite and positive "
                "(got %.6f m and %.6f deg)",
                range_sigma_m_, bearing_sigma_deg);
            throw std::runtime_error("invalid nonlinear localization measurement sigmas");
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

        /* Handover seed for global_nav_kf_2d_node: published exactly once at
         * the end of Phase 2.  Latched so the KF still gets it if it starts
         * (or restarts) after initialization has completed. */
        rclcpp::QoS init_qos(1);
        init_qos.reliable().transient_local();
        map_odom_init_pub_ =
            create_publisher<geometry_msgs::msg::TransformStamped>(
                "/map_odom_init", init_qos);

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

        /* Phase-1 watchdog.  Runs on solver_cbg_ (MutuallyExclusive) so it is
         * serialised with the marker callbacks: init_phase_, the phase-1
         * accumulators and prev_map_odom_tf_ are only ever touched from that
         * group, and this timer must not race them. */
        init_start_time_ = now();
        init_timeout_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&PoseEstimatorLidarNode::check_init_timeout, this),
            solver_cbg_);
    }

private:
    /* ================================================================ */
    /*  Constants & state                                               */
    /* ================================================================ */
    static constexpr double MAP_SIZE           = 300.0;
    /** Max age (seconds) of last /cube_markers vs /cube_markers_phi to merge. */
    static constexpr double CUBE_MERGE_TTL_SEC = 0.65;
    // Phase 1: camera-bearings-only init samples.
    static constexpr int    NBR_INIT_CALLBACKS_PHASE1 = 5;
    // Phase 2: cube-refined init samples (after Phase-1 TF is broadcast).
    static constexpr int    NBR_INIT_CALLBACKS_PHASE2 = 10;
    static constexpr double CALLBACK_PERIOD_LIMIT = 1.0 / 15.0;
    static constexpr double MAX_TRANSLATION_JUMP  = 0.8;
    static constexpr double MAX_YAW_JUMP = 45.0 * M_PI / 180.0;
    /** Phase-2 init: max deviation of a marker's yaw_from_start vs the
     *  phase-1 yaw before the marker is treated as a mislabeled cube. */
    static constexpr double INIT_YAW_GATE_RAD = 10.0 * M_PI / 180.0;
    /** Same gate when phase 1 fell back to the hand-alignment backup yaw: the
     *  reference is a manual aim, not a measurement, so it must tolerate a
     *  larger alignment error or every phase-2 cube gets rejected. */
    static constexpr double INIT_YAW_GATE_BACKUP_RAD = 45.0 * M_PI / 180.0;
    /** On timeout, this many real phase-1 samples beat the backup yaw. */
    static constexpr int MIN_PHASE1_SAMPLES_ON_TIMEOUT = 3;
    /* Previously used to throttle yaw; throttling after a fresh (x,y) solve
     * leaves heading inconsistent with position (wrong map→odom yaw). */

    static constexpr double MAP_XMIN = -16.0;
    static constexpr double MAP_XMAX =  16.0;
    static constexpr double MAP_YMIN = -6.0;
    static constexpr double MAP_YMAX =  30.0;

    std::array<double, 2> erc_start_pos_;
    std::vector<std::pair<double, double>> landmark_poses_;
    double backup_erc_map_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
    double init_camera_timeout_sec_{40.0};
    double range_sigma_m_{0.20};
    double bearing_sigma_rad_{5.0 * M_PI / 180.0};

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
    /** True once curr_map_base_* came from a real map->odom TF lookup. */
    bool   have_curr_map_base_{false};

    /* ---- init accumulation ---- */
    InitPhase init_phase_;
    int  init_counter_phase1_;
    int  init_counter_phase2_;
    /* Phase-1 averaged yaw, reference for the phase-2 yaw gate. */
    double phase1_yaw_ref_{0.0};
    bool   have_phase1_yaw_ref_{false};
    /* Phase 1 completed from backup_erc_map_yaw_rad instead of camera samples. */
    bool   phase1_from_backup_{false};
    rclcpp::Time init_start_time_;
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
    rclcpp::TimerBase::SharedPtr init_timeout_timer_;

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

    /* ---- bearing diagnostics (all touched from solver_cbg_ only) ---- */
    std::unordered_map<int, double> last_camera_bearing_deg_;
    std::unordered_map<std::string, rclcpp::Time> last_bearing_log_time_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
        map_odom_init_pub_;

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
            // RCLCPP_INFO(get_logger(),
            //     "[YAW SOLVER DEBUG] est=(%.3f, %.3f) id=%d "
            //     "lm=(%.3f, %.3f) bearing_map=%.2f deg "
            //     "marker_bearing=%.2f deg yaw_component=%.2f deg",
            //     est_x, est_y, marker.landmark_index,
            //     lm.first, lm.second,
            //     bearing_map * 180.0 / M_PI,
            //     marker.bearing_rad * 180.0 / M_PI,
            //     yaw_component * 180.0 / M_PI);
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
    /*  Periodic TF re-broadcast (20 Hz), initialization only           */
    /*                                                                  */
    /*  accumulate_phase2 cancels this timer when Phase 2 completes;    */
    /*  the phase check is a second gate so a queued tick can never     */
    /*  fight global_nav_kf_2d_node for ownership of map->odom.         */
    /* ================================================================ */
    void republish_tf()
    {
        if (init_phase_ == InitPhase::DONE) return;

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
            have_curr_map_base_ = true;
        } catch (const tf2::TransformException &) {
            if (!prev_map_odom_tf_.has_value()) {
                curr_map_base_x_   = odom_pos_x_;
                curr_map_base_y_   = odom_pos_y_;
                curr_map_base_yaw_ = odom_yaw_;
                have_curr_map_base_ = false;
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
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] cube_markers has empty frame_id, skipping marker");
            return false;
        }

        if (source_frame == "base_link") {
            base_x = p.x;
            base_y = p.y;
            return std::isfinite(base_x) && std::isfinite(base_y);
        }

        geometry_msgs::msg::TransformStamped tf;
        if (!lookup_tf_base_from_source_at_msg_stamp(source_frame, msg.header.stamp, tf)) {
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] TF unavailable %s -> base_link at msg stamp, skipping marker",
            //     source_frame.c_str());
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
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] Transform %s -> base_link failed: %s",
            //     source_frame.c_str(), ex.what());
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
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] cube_markers has empty frame_id, skipping bearing");
            return false;
        }

        if (source_frame == "base_link") {
            bearing_rad = wrap(source_bearing);
            return true;
        }

        geometry_msgs::msg::TransformStamped tf;
        if (!lookup_tf_base_from_source_at_msg_stamp(source_frame, msg.header.stamp, tf)) {
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] TF unavailable %s -> base_link at msg stamp, skipping bearing",
            //     source_frame.c_str());
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
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[pose_estimator_lidar] Bearing transform %s -> base_link failed: %s",
            //     source_frame.c_str(), ex.what());
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

        /* yaw: same MAD outlier rejection as translation, on circular
         * deviations from the plain mean (one bad sample otherwise
         * shifts the unfiltered circular mean by dev/N). */
        double avg_yaw = circular_mean_yaw(yaw_list);
        int yaw_inlier_count = static_cast<int>(yaw_list.size());
        if (yaw_list.size() >= 3) {
            std::vector<double> dev(yaw_list.size());
            for (size_t i = 0; i < yaw_list.size(); ++i)
                dev[i] = wrap(yaw_list[i] - avg_yaw);

            auto vec_median = [](std::vector<double> v) {
                std::sort(v.begin(), v.end());
                const size_t m = v.size();
                return (m % 2 == 0)
                    ? 0.5 * (v[m / 2 - 1] + v[m / 2])
                    : v[m / 2];
            };
            const double med = vec_median(dev);
            std::vector<double> abs_dev(dev.size());
            for (size_t i = 0; i < dev.size(); ++i)
                abs_dev[i] = std::fabs(dev[i] - med);
            const double yaw_mad = vec_median(abs_dev);
            const double yaw_thresh =
                std::max(3.0 * yaw_mad, 5.0 * M_PI / 180.0);

            std::vector<double> inlier_yaws;
            inlier_yaws.reserve(yaw_list.size());
            for (size_t i = 0; i < yaw_list.size(); ++i) {
                if (std::fabs(dev[i] - med) < yaw_thresh)
                    inlier_yaws.push_back(yaw_list[i]);
            }
            if (inlier_yaws.size() >= 3) {
                avg_yaw = circular_mean_yaw(inlier_yaws);
                yaw_inlier_count = static_cast<int>(inlier_yaws.size());
            }
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp_now(this);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id  = "odom";
        tf_msg.transform.translation.x = final_x;
        tf_msg.transform.translation.y = final_y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = yaw_to_quat(avg_yaw);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Robust init TF: t=(%.3f, %.3f), yaw=%.2f deg "
            "(xy inliers %d/%d, yaw inliers %d/%d)",
            final_x, final_y, avg_yaw * 180.0 / M_PI,
            inlier_count, N, yaw_inlier_count, N);
        return tf_msg;
    }

    std::optional<Eigen::Vector3d> solve_nonlinear_range_bearing(
        const std::vector<ValidMarker> &valid_markers)
    {
        if (valid_markers.size() < 2u ||
            !std::isfinite(curr_map_base_x_) ||
            !std::isfinite(curr_map_base_y_) ||
            !std::isfinite(curr_map_base_yaw_)) {
            return std::nullopt;
        }

        using BlockSolver = g2o::BlockSolver<
            g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>;
        auto linear_solver =
            std::make_unique<g2o::LinearSolverDense<BlockSolver::PoseMatrixType>>();
        auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        optimizer.setAlgorithm(
            new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver)));

        auto *pose_vertex = new RoverPoseVertex();
        pose_vertex->setId(0);
        pose_vertex->setEstimate(Eigen::Vector3d(
            curr_map_base_x_, curr_map_base_y_, curr_map_base_yaw_));
        if (!optimizer.addVertex(pose_vertex)) {
            delete pose_vertex;
            return std::nullopt;
        }

        Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
        information(0, 0) = 1.0 / (range_sigma_m_ * range_sigma_m_);
        information(1, 1) = 1.0 / (bearing_sigma_rad_ * bearing_sigma_rad_);

        for (const auto &marker : valid_markers) {
            const auto &landmark = landmark_poses_[marker.landmark_index];
            auto *edge = new RangeBearingEdge(
                Eigen::Vector2d(landmark.first, landmark.second),
                marker.range,
                marker.bearing_rad);

            edge->setVertex(0, pose_vertex);
            edge->setInformation(information);

            // Robust Huber loss
            auto *huber = new g2o::RobustKernelHuber();
            huber->setDelta(2.5);
            edge->setRobustKernel(huber);

            if (!optimizer.addEdge(edge)) {
                delete edge;
                return std::nullopt;
            }
        }

        optimizer.initializeOptimization();
        const int iterations = optimizer.optimize(30);
        if (iterations <= 0) {
            return std::nullopt;
        }

        Eigen::Vector3d result = pose_vertex->estimate();
        result[2] = wrap(result[2]);
        if (!result.allFinite() ||
            result.x() < MAP_XMIN || result.x() > MAP_XMAX ||
            result.y() < MAP_YMIN || result.y() > MAP_YMAX) {
            return std::nullopt;
        }

        optimizer.computeActiveErrors();
        if (!std::isfinite(optimizer.chi2())) {
            return std::nullopt;
        }
        return result;
    }

    /* ================================================================ */
    /*  Per-source bearing diagnostics                                  */
    /*                                                                  */
    /*  CAM   = /aruco_markers     (camera-only bearings)               */
    /*  LIDAR = /cube_markers      (detect_cube LiDAR centres)          */
    /*  PHI   = /cube_markers_phi  (camera fallback, drill sector)      */
    /*                                                                  */
    /*  Per marker: raw = ar_angles_list as published; base = that      */
    /*  angle transformed to base_link; posBrg = atan2(y, x) of the     */
    /*  transformed position. dAngPos = base - posBrg: non-zero means   */
    /*  the source's angle and position fields disagree (frame or       */
    /*  convention bug in the publisher). dCam = base - last CAM        */
    /*  bearing for the same id: the camera-vs-LiDAR discrepancy.       */
    /*  Logged at most once per second per source.                      */
    /* ================================================================ */
    void log_bearing_diagnostics(
        const char *src_tag,
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg,
        bool is_camera_source)
    {
        const size_t n = std::min(
            msg.marker_ids.size(),
            std::min(msg.poses.size(), msg.ar_angles_list.size()));
        if (n == 0) return;

        constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
        const rclcpp::Time t_now = now();
        rclcpp::Time &last_log = last_bearing_log_time_[src_tag];
        const bool do_log = (last_log.nanoseconds() == 0) ||
            (t_now - last_log).seconds() >= 1.0;

        /* Rover pose in map, from the live TF chain via update_curr_map_base()
         * (NaN before any map->odom TF exists).  Must not come from
         * prev_map_odom_tf_: after the Phase-2 handover that member is frozen
         * while global_nav_kf_2d_node keeps moving the real transform, which
         * would silently bias exp/dExp. */
        double rov_x = NaN, rov_y = NaN, rov_yaw_rad = NaN;
        if (have_curr_map_base_) {
            rov_x = curr_map_base_x_;
            rov_y = curr_map_base_y_;
            rov_yaw_rad = curr_map_base_yaw_;
        }
        const double rov_yaw_deg = rov_yaw_rad * 180.0 / M_PI;

        std::string line;
        char buf[288];
        for (size_t k = 0; k < n; ++k) {
            const int id = static_cast<int>(msg.marker_ids[k]);
            const double raw_deg = msg.ar_angles_list[k];

            double bearing_rad = 0.0;
            const bool have_ang =
                marker_bearing_in_base_link(msg, k, bearing_rad);
            const double ang_deg =
                have_ang ? bearing_rad * 180.0 / M_PI : NaN;

            double bx = NaN, by = NaN;
            const bool have_pos =
                marker_position_in_base_link(msg, k, bx, by);
            const double pos_brg_deg =
                have_pos ? std::atan2(by, bx) * 180.0 / M_PI : NaN;
            const double range = have_pos ? std::hypot(bx, by) : NaN;

            const double d_ang_pos = (have_ang && have_pos)
                ? ang_diff_deg(ang_deg, pos_brg_deg) : NaN;

            if (is_camera_source && have_ang) {
                last_camera_bearing_deg_[id] = ang_deg;
            }

            if (!do_log) continue;

            double d_cam = NaN;
            if (!is_camera_source && have_ang) {
                const auto it = last_camera_bearing_deg_.find(id);
                if (it != last_camera_bearing_deg_.end()) {
                    d_cam = ang_diff_deg(ang_deg, it->second);
                }
            }

            /* Expected bearing to this landmark given the rover's map
             * pose/heading; dExp = measured - expected. */
            double exp_deg = NaN, d_exp = NaN;
            if (std::isfinite(rov_yaw_rad) &&
                id >= 0 && id < static_cast<int>(landmark_poses_.size())) {
                const auto &lm = landmark_poses_[id];
                if (std::abs(lm.first) < MAP_SIZE &&
                    std::abs(lm.second) < MAP_SIZE) {
                    exp_deg = wrap(std::atan2(lm.second - rov_y,
                                              lm.first - rov_x) -
                                   rov_yaw_rad) * 180.0 / M_PI;
                    if (have_ang) {
                        d_exp = ang_diff_deg(ang_deg, exp_deg);
                    }
                }
            }

            std::snprintf(buf, sizeof(buf),
                " | id=%d raw=%.1f base=%.1f posBrg=%.1f dAngPos=%.1f "
                "r=%.2f pos=(%.2f, %.2f) exp=%.1f dExp=%.1f dCam=%.1f",
                id, raw_deg, ang_deg, pos_brg_deg, d_ang_pos,
                range, bx, by, exp_deg, d_exp, d_cam);
            line += buf;
        }

        if (do_log && !line.empty()) {
            last_log = t_now;
            RCLCPP_INFO(get_logger(),
                "[BEARING %s] rover_map=(%.2f, %.2f) yaw_map=%.1f deg "
                "frame=%s%s",
                src_tag, rov_x, rov_y, rov_yaw_deg,
                msg.header.frame_id.c_str(), line.c_str());
        }
    }

    /* ================================================================ */
    /*  Phase-1 input: camera-only ArUco detections                     */
    /*  Active only while init_phase_ == CAMERA.                        */
    /* ================================================================ */
    void aruco_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        /* Diagnostics + camera-reference update run in every phase so
         * LIDAR/PHI bearings stay comparable after init. */
        log_bearing_diagnostics("CAM", *msg, /*is_camera_source=*/true);
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
        log_bearing_diagnostics("LIDAR", *msg, /*is_camera_source=*/false);
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
        log_bearing_diagnostics("PHI", *msg, /*is_camera_source=*/false);
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
            // RCLCPP_WARN(get_logger(),
            //     "[%s MSG SHAPE] marker_ids=%zu poses=%zu ar_angles_list=%zu "
            //     "(indices must align; short arrays cause skips)",
            //     src_tag, n_ids, n_poses, n_angles);
        }

        /* ---- validate markers ---- */
        std::vector<ValidMarker> valid_markers;
        for (size_t k = 0; k < msg->marker_ids.size(); ++k) {
            const int64_t raw_id = msg->marker_ids[k];
            if (k >= msg->poses.size()) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=no_pose_slot",
                //     src_tag, k, static_cast<long long>(raw_id));
                continue;
            }
            if (k >= msg->ar_angles_list.size()) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=no_ar_angle_slot",
                //     src_tag, k, static_cast<long long>(raw_id));
                continue;
            }

            int idx = static_cast<int>(raw_id);
            if (idx < 0 || idx >= static_cast<int>(landmark_poses_.size())) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu raw_id=%lld reason=id_out_of_table "
                //     "(valid index range 0..%zu)",
                //     src_tag, k, static_cast<long long>(raw_id),
                //     landmark_poses_.size() - 1);
                continue;
            }
            auto &lm = landmark_poses_[idx];
            if (std::abs(lm.first)  >= MAP_SIZE ||
                std::abs(lm.second) >= MAP_SIZE) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu id=%d reason=landmark_unmapped_or_sentinel "
                //     "lm=(%.3f, %.3f)",
                //     src_tag, k, idx, lm.first, lm.second);
                continue;
            }

            double base_x = 0.0;
            double base_y = 0.0;
            if (!marker_position_in_base_link(*msg, k, base_x, base_y)) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu id=%d frame=\"%s\" reason=position_to_base_link_failed",
                //     src_tag, k, idx, msg->header.frame_id.c_str());
                continue;
            }

            const double range = std::hypot(base_x, base_y);
            if (!std::isfinite(range) || range < 1e-3) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu id=%d reason=bad_range "
                //     "base=(%.4f, %.4f) range=%.6g",
                //     src_tag, k, idx, base_x, base_y, range);
                continue;
            }

            double bearing_rad = 0.0;
            if (!marker_bearing_in_base_link(*msg, k, bearing_rad)) {
                // RCLCPP_INFO(get_logger(),
                //     "[%s SKIP marker] msg_i=%zu id=%d frame=\"%s\" reason=bearing_to_base_link_failed",
                //     src_tag, k, idx, msg->header.frame_id.c_str());
                continue;
            }

            if (init_phase_ != InitPhase::DONE) {
                const double lm_bearing_from_start = std::atan2(
                    lm.second - erc_start_pos_[1],
                    lm.first - erc_start_pos_[0]);
                const double yaw_from_start = wrap(
                    lm_bearing_from_start - bearing_rad);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[%s MARKER DEBUG] phase=%d msg_i=%zu id=%d frame=%s "
                    "lm=(%.3f, %.3f) base=(%.3f, %.3f) range=%.3f "
                    "bearing=%.2f deg lm_bearing_start=%.2f deg "
                    "yaw_from_start=%.2f deg",
                    src_tag, phase_id, k, idx, msg->header.frame_id.c_str(),
                    lm.first, lm.second, base_x, base_y, range,
                    bearing_rad * 180.0 / M_PI,
                    lm_bearing_from_start * 180.0 / M_PI,
                    yaw_from_start * 180.0 / M_PI);

                /* Phase-2 yaw gate: a marker whose implied yaw disagrees
                 * with the phase-1 yaw is a mislabeled cube (e.g. detect_cube
                 * sector computed before map->odom was valid); one such
                 * sample tilts the unfiltered init yaw average. */
                if (init_phase_ == InitPhase::CUBE && have_phase1_yaw_ref_) {
                    const double yaw_gate = phase1_from_backup_
                        ? INIT_YAW_GATE_BACKUP_RAD : INIT_YAW_GATE_RAD;
                    const double yaw_dev =
                        std::fabs(wrap(yaw_from_start - phase1_yaw_ref_));
                    if (yaw_dev > yaw_gate) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[%s SKIP marker] id=%d yaw_from_start=%.1f deg "
                            "deviates %.1f deg from phase-1 yaw %.1f deg "
                            "(gate %.0f deg) — likely mislabeled cube",
                            src_tag, idx,
                            yaw_from_start * 180.0 / M_PI,
                            yaw_dev * 180.0 / M_PI,
                            phase1_yaw_ref_ * 180.0 / M_PI,
                            yaw_gate * 180.0 / M_PI);
                        continue;
                    }
                }
            }

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

        // RCLCPP_INFO(get_logger(),
        //     "[%s DEBUG] phase=%d raw_markers=%zu valid_markers=%d",
        //     src_tag, phase_id, msg->marker_ids.size(), n);
        if (msg->marker_ids.size() != static_cast<size_t>(n)) {
            // RCLCPP_INFO(get_logger(),
            //     "[%s FILTER SUMMARY] dropped %zu of %zu raw markers "
            //     "(see [SKIP marker] lines above)",
            //     src_tag,
            //     msg->marker_ids.size() - static_cast<size_t>(n),
            //     msg->marker_ids.size());
        }

        /* ============================================================ */
        /*  INITIALISATION PHASE (Phase 1 OR Phase 2)                   */
        /*  Same algorithm; only the input source and accumulator       */
        /*  buffer differ.                                              */
        /* ============================================================ */
        if (init_phase_ != InitPhase::DONE) {
            x_estimate_ = erc_start_pos_[0];
            y_estimate_ = erc_start_pos_[1];
            yaw_estimate_ = deduce_yaw(
                x_estimate_, y_estimate_, valid_markers, *msg);
            measured_new_yaw_ = true;
            time_of_last_yaw_meas_ = t_now;

            transform_msg = build_map_odom_tf(
                x_estimate_, y_estimate_, yaw_estimate_);
            is_measurement_valid = true;

        /* ============================================================ */
        /*  POST-INIT CONTINUOUS UPDATES                                */
        /* ============================================================ */
        } else {
            if (n >= 2) {
                auto pose = solve_nonlinear_range_bearing(valid_markers);

                if (pose.has_value()) {
                    double dx = pose->x() - curr_map_base_x_;
                    double dy = pose->y() - curr_map_base_y_;
                    double jump = std::hypot(dx, dy);
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[UPDATE solver] solution P=(%.3f, %.3f), "
                        "current=(%.3f, %.3f), jump=%.3f m",
                        pose->x(), pose->y(),
                        curr_map_base_x_, curr_map_base_y_, jump);

                    if (jump <= MAX_TRANSLATION_JUMP) {
                        x_estimate_ = pose->x();
                        y_estimate_ = pose->y();
                        yaw_estimate_ = pose->z();
                        solved_new_xy_ = true;
                        time_of_last_pose_ = t_now;
                        measured_new_yaw_ = true;
                        time_of_last_yaw_meas_ = t_now;
                        // RCLCPP_INFO(get_logger(),
                        //     "[UPDATE] yaw = %.2f deg",
                        //     yaw_estimate_ * 180.0 / M_PI);
                    } else {
                        RCLCPP_WARN_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            "[UPDATE] Rejected jump: %.2f m candidate=(%.3f, %.3f) current=(%.3f, %.3f)",
                            jump, pose->x(), pose->y(),
                            curr_map_base_x_, curr_map_base_y_);
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[UPDATE n>=%d] nonlinear solver returned no solution", n);
                }
            }

            if (solved_new_xy_) {
                // Keep map->odom fixed after Phase 2.  The nonlinear result
                // is published below on /aruco_rover_pos (in map) so the EKF
                // can fuse it after converting it into its odom frame.
                is_measurement_valid = true;
            }

            // republish_tf() continues to broadcast the fixed Phase-2
            // map->odom transform at 20 Hz.  Do not publish a solver-derived
            // transform here.
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

        // RCLCPP_INFO(get_logger(),
        //     "[P1 ACCUM %d/%d] yaw=%.2f deg",
        //     init_counter_phase1_, NBR_INIT_CALLBACKS_PHASE1,
        //     yaw_estimate_ * 180.0 / M_PI);

        if (init_counter_phase1_ < NBR_INIT_CALLBACKS_PHASE1) return;

        // Phase 1 complete: broadcast the camera-derived map->odom TF
        // so downstream lidar nodes can label cubes correctly.
        finalize_phase1(
            calculate_robust_tf_avg(phase1_tfs_, phase1_yaws_),
            /*from_backup=*/false, "camera samples");
    }

    /* ================================================================ */
    /*  Commit the Phase-1 map->odom TF and hand over to Phase 2.       */
    /*  Called either with the robust average of camera samples or,     */
    /*  on timeout, with the backup-yaw transform.                      */
    /*  Runs on solver_cbg_ only.                                       */
    /* ================================================================ */
    void finalize_phase1(
        const geometry_msgs::msg::TransformStamped &tf_final,
        bool from_backup,
        const char *reason)
    {
        prev_map_odom_tf_ = tf_final;
        phase1_yaw_ref_ = quat_to_yaw(tf_final.transform.rotation);
        have_phase1_yaw_ref_ = true;
        phase1_from_backup_ = from_backup;
        tf_broadcaster_->sendTransform(prev_map_odom_tf_.value());
        init_phase_ = InitPhase::CUBE;

        const double yaw_gate_deg =
            (from_backup ? INIT_YAW_GATE_BACKUP_RAD : INIT_YAW_GATE_RAD)
            * 180.0 / M_PI;
        if (from_backup) {
            RCLCPP_WARN(get_logger(),
                "PHASE 1 DONE (%s, %d/%d camera samples): map->odom yaw=%.2f deg, "
                "t=(%.3f, %.3f). Switching to PHASE 2 (cube refinement) with a "
                "%.0f deg yaw gate.",
                reason, init_counter_phase1_, NBR_INIT_CALLBACKS_PHASE1,
                phase1_yaw_ref_ * 180.0 / M_PI,
                tf_final.transform.translation.x,
                tf_final.transform.translation.y, yaw_gate_deg);
        } else {
            RCLCPP_INFO(get_logger(),
                "PHASE 1 DONE (%s, %d/%d camera samples): map->odom yaw=%.2f deg, "
                "t=(%.3f, %.3f). Switching to PHASE 2 (cube refinement) with a "
                "%.0f deg yaw gate.",
                reason, init_counter_phase1_, NBR_INIT_CALLBACKS_PHASE1,
                phase1_yaw_ref_ * 180.0 / M_PI,
                tf_final.transform.translation.x,
                tf_final.transform.translation.y, yaw_gate_deg);
        }
    }

    /* ================================================================ */
    /*  Phase-1 watchdog: bound how long we wait for camera landmarks.  */
    /*                                                                  */
    /*  Without this the node sits in CAMERA forever when no tag is      */
    /*  visible at the start line, broadcasting an identity map->odom.  */
    /*  Runs on solver_cbg_, serialised with the marker callbacks.      */
    /* ================================================================ */
    void check_init_timeout()
    {
        if (init_phase_ != InitPhase::CAMERA) return;
        if (!(init_camera_timeout_sec_ > 0.0)) return;
        if ((now() - init_start_time_).seconds() < init_camera_timeout_sec_)
            return;

        /* Real camera samples arrived, just not the full set: they beat the
         * hand-alignment guess. */
        if (init_counter_phase1_ >= MIN_PHASE1_SAMPLES_ON_TIMEOUT) {
            finalize_phase1(
                calculate_robust_tf_avg(phase1_tfs_, phase1_yaws_),
                /*from_backup=*/false, "timeout, partial camera samples");
            return;
        }

        if (!std::isfinite(backup_erc_map_yaw_rad_)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "PHASE 1 STUCK: no camera landmarks after %.1f s and "
                "backup_erc_map_yaw_rad is not set — map->odom stays identity",
                init_camera_timeout_sec_);
            return;
        }

        /* (erc_start_pos, backup yaw) only describes the rover while it is
         * still parked on its start point. */
        const double odom_dist = std::hypot(odom_pos_x_, odom_pos_y_);
        if (odom_dist > 0.5 || std::fabs(odom_yaw_) > 15.0 * M_PI / 180.0) {
            RCLCPP_WARN(get_logger(),
                "PHASE 1 backup yaw applied although odom shows the rover "
                "moved (%.2f m, %.1f deg) since start: the assumed start pose "
                "is likely wrong",
                odom_dist, odom_yaw_ * 180.0 / M_PI);
        }

        x_estimate_ = erc_start_pos_[0];
        y_estimate_ = erc_start_pos_[1];
        yaw_estimate_ = backup_erc_map_yaw_rad_;

        finalize_phase1(
            build_map_odom_tf(x_estimate_, y_estimate_, yaw_estimate_),
            /*from_backup=*/true, "no camera landmarks, backup yaw");
        publish_odom(x_estimate_, y_estimate_, yaw_estimate_);
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

        // RCLCPP_INFO(get_logger(),
        //     "[P2 ACCUM %d/%d] yaw=%.2f deg",
        //     init_counter_phase2_, NBR_INIT_CALLBACKS_PHASE2,
        //     yaw_estimate_ * 180.0 / M_PI);

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
            /* Rover map yaw per phase: map->odom yaw + current odom yaw
             * (odom ~ 0 while stationary at init, so this is ~ the TF yaw). */
            const double yaw1_deg =
                wrap(quat_to_yaw(p1.rotation) + odom_yaw_) * 180.0 / M_PI;
            const double yaw2_deg =
                wrap(quat_to_yaw(p2.rotation) + odom_yaw_) * 180.0 / M_PI;
            const double dyaw = ang_diff_deg(yaw2_deg, yaw1_deg);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "PHASE 2 refinement vs PHASE 1: rover yaw_map P1=%.2f deg "
                "-> P2=%.2f deg (dyaw=%.2f deg), dxy=%.3f m",
                yaw1_deg, yaw2_deg, dyaw, dxy);
        }

        prev_map_odom_tf_ = refined;
        init_phase_ = InitPhase::DONE;

        /* Hand map->odom over to global_nav_kf_2d_node: publish the seed, then
         * stop broadcasting so there is exactly one owner of the transform.
         * The KF broadcasts from its seed callback, so the gap is one message
         * round-trip. */
        refined.header.stamp = stamp_now(this);
        map_odom_init_pub_->publish(refined);
        tf_timer_->cancel();

        RCLCPP_INFO(get_logger(),
            "PHASE 2 DONE (cube refinement): INITIALIZED map->odom TF, handed "
            "over to global_nav_kf_2d_node on /map_odom_init. Rate-limiting to "
            "%.1f Hz",
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
