#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */

class PoseEstimatorLidarNode : public rclcpp::Node
{
public:
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
        init_callback_counter_ = 0;
        initialized_map_odom_tf_ = false;
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
    static constexpr int    NBR_INIT_CALLBACKS = 35;
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
        {0.96, 3.57},       // id 51
        {-1.68, 3.7},       // id 52
        {-4.0, -2.0}, 
        {0.0, 0.0}, 
        {999999, 999999},
        {999999, 999999}, 
        {999999, 999999}, 
        {999999, 999999},
        {999999, 999999}, 
        {999999, 999999}, 
        {999999, 999999},
        {999999, 999999}, 
        {999999, 999999}, 
        {999999, 999999},
        {999999, 999999},
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
    int  init_callback_counter_;
    bool initialized_map_odom_tf_;
    rclcpp::Time last_callback_time_;
    std::vector<geometry_msgs::msg::TransformStamped> avg_initialization_tfs_;
    std::vector<double> yaw_init_list_;

    /* ---- TF ---- */
    std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::optional<geometry_msgs::msg::TransformStamped> prev_map_odom_tf_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    /* ---- ROS ---- */
    rclcpp::CallbackGroup::SharedPtr solver_cbg_, rt_cbg_;
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
        const std::vector<std::tuple<int, int>> &valid_markers,
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg) const
    {
        std::vector<double> yaw_list;
        yaw_list.reserve(valid_markers.size());
        for (auto &[idx, k] : valid_markers) {
            auto &lm = landmark_poses_[idx];
            double bearing_map =
                std::atan2(lm.second - est_y, lm.first - est_x);
            double measured_phi =
                msg.ar_angles_list[k] * M_PI / 180.0;
            yaw_list.push_back(wrap(bearing_map - measured_phi));
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
                "odom", "map", tf2::TimePointZero);
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
        const std::vector<std::tuple<int, int>> &valid_markers,
        const ros2_aruco_interfaces::msg::ArucoMarkers &msg) const
    {
        std::vector<Measurement> meas;
        meas.reserve(valid_markers.size());
        for (auto &[idx, k] : valid_markers) {
            double lm_x = landmark_poses_[idx].first;
            double lm_y = landmark_poses_[idx].second;
            double px = msg.poses[k].position.x;
            double py = msg.poses[k].position.y;
            double r = std::hypot(px, py);
            if (r < 1e-3) continue;
            double bearing_rad = msg.ar_angles_list[k] * M_PI / 180.0;
            meas.push_back({lm_x, lm_y, r,
                            -std::cos(bearing_rad),
                            -std::sin(bearing_rad)});
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
    /*  Main callback: /cube_markers                                    */
    /* ================================================================ */
    void cube_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        rclcpp::Time t_now = now();
        double dt_since_last =
            (t_now - last_callback_time_).seconds();
        if (dt_since_last < CALLBACK_PERIOD_LIMIT &&
            initialized_map_odom_tf_)
            return;
        last_callback_time_ = t_now;

        update_curr_map_base();

        /* ---- validate markers ---- */
        using VM = std::tuple<int, int>;   // (landmark_index, msg_index)
        std::vector<VM> valid_markers;
        for (size_t k = 0; k < msg->marker_ids.size(); ++k) {
            int idx = static_cast<int>(msg->marker_ids[k]);
            if (idx < 0 || idx >= static_cast<int>(landmark_poses_.size()))
                continue;
            auto &lm = landmark_poses_[idx];
            if (std::abs(lm.first)  < MAP_SIZE &&
                std::abs(lm.second) < MAP_SIZE)
                valid_markers.emplace_back(idx, static_cast<int>(k));
        }

        int n = static_cast<int>(valid_markers.size());
        if (n < 1) return;

        bool is_measurement_valid = false;
        std::optional<geometry_msgs::msg::TransformStamped> transform_msg;

        /* ============================================================ */
        /*  INITIALISATION PHASE                                        */
        /* ============================================================ */
        if (!initialized_map_odom_tf_) {

            if (n == 1) {
                auto [iA, kA] = valid_markers[0];
                auto &A = landmark_poses_[iA];
                double phiA = msg->ar_angles_list[kA] * M_PI / 180.0;
                double x0 = erc_start_pos_[0], y0 = erc_start_pos_[1];
                double yawA = wrap(
                    std::atan2(A.second - y0, A.first - x0) - phiA);

                yaw_estimate_ = yawA;
                x_estimate_ = x0;
                y_estimate_ = y0;
                measured_new_yaw_ = true;
                time_of_last_yaw_meas_ = t_now;

                RCLCPP_INFO(get_logger(),
                    "[INIT n=1] yaw = %.2f deg",
                    yaw_estimate_ * 180.0 / M_PI);

                Eigen::Matrix4d T_map_base =
                    pose_to_mat(x0, y0, yaw_estimate_);
                Eigen::Matrix4d T_map_odom =
                    T_map_base; // odom==identity at init

                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = stamp_now(this);
                tf.header.frame_id = "map";
                tf.child_frame_id  = "odom";
                tf.transform.translation.x = T_map_odom(0, 3);
                tf.transform.translation.y = T_map_odom(1, 3);
                tf.transform.rotation = yaw_to_quat(
                    std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
                transform_msg = tf;
                is_measurement_valid = true;

            } else if (n == 2) {
                auto [iA, kA] = valid_markers[0];
                auto [iB, kB] = valid_markers[1];
                auto &A = landmark_poses_[iA];
                auto &B = landmark_poses_[iB];
                double phiA = msg->ar_angles_list[kA] * M_PI / 180.0;
                double phiB = msg->ar_angles_list[kB] * M_PI / 180.0;
                double x0 = erc_start_pos_[0], y0 = erc_start_pos_[1];

                double yawA = wrap(
                    std::atan2(A.second - y0, A.first - x0) - phiA);
                double yawB = wrap(
                    std::atan2(B.second - y0, B.first - x0) - phiB);
                yaw_estimate_ = wrap(0.5 * (yawA + yawB));
                x_estimate_ = x0;
                y_estimate_ = y0;
                measured_new_yaw_ = true;
                time_of_last_yaw_meas_ = t_now;

                RCLCPP_INFO(get_logger(),
                    "[INIT n=2] yaw = %.2f deg",
                    yaw_estimate_ * 180.0 / M_PI);

                Eigen::Matrix4d T_map_base =
                    pose_to_mat(x0, y0, yaw_estimate_);
                Eigen::Matrix4d T_map_odom = T_map_base;

                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = stamp_now(this);
                tf.header.frame_id = "map";
                tf.child_frame_id  = "odom";
                tf.transform.translation.x = T_map_odom(0, 3);
                tf.transform.translation.y = T_map_odom(1, 3);
                tf.transform.rotation = yaw_to_quat(
                    std::atan2(T_map_odom(1, 0), T_map_odom(0, 0)));
                transform_msg = tf;
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
                            "[INIT n>=%d] P=(%.3f, %.3f), yaw=%.2f deg",
                            n, xy->first, xy->second,
                            yaw_estimate_ * 180.0 / M_PI);

                        Eigen::Matrix4d T_map_base =
                            pose_to_mat(xy->first, xy->second,
                                        yaw_estimate_);
                        Eigen::Matrix4d T_map_odom = T_map_base;

                        geometry_msgs::msg::TransformStamped tf;
                        tf.header.stamp = stamp_now(this);
                        tf.header.frame_id = "map";
                        tf.child_frame_id  = "odom";
                        tf.transform.translation.x = T_map_odom(0, 3);
                        tf.transform.translation.y = T_map_odom(1, 3);
                        tf.transform.rotation = yaw_to_quat(
                            std::atan2(T_map_odom(1, 0),
                                       T_map_odom(0, 0)));
                        transform_msg = tf;
                        is_measurement_valid = true;
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "[INIT] Rejected: solution (%.3f,%.3f) "
                            "too far from start (%.2f m)",
                            xy->first, xy->second,
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
        /*  INITIALISATION ACCUMULATION                                 */
        /* ============================================================ */
        if (init_callback_counter_ < NBR_INIT_CALLBACKS &&
            is_measurement_valid && transform_msg.has_value())
        {
            ++init_callback_counter_;
            avg_initialization_tfs_.push_back(transform_msg.value());
            yaw_init_list_.push_back(yaw_estimate_);

            RCLCPP_INFO(get_logger(),
                "[ACCUM %d/%d] yaw=%.2f deg",
                init_callback_counter_, NBR_INIT_CALLBACKS,
                yaw_estimate_ * 180.0 / M_PI);

            if (init_callback_counter_ == NBR_INIT_CALLBACKS) {
                initialized_map_odom_tf_ = true;
                prev_map_odom_tf_ = calculate_robust_tf_avg(
                    avg_initialization_tfs_, yaw_init_list_);
                tf_broadcaster_->sendTransform(
                    prev_map_odom_tf_.value());
                RCLCPP_INFO(get_logger(),
                    "INITIALIZED map->odom TF. "
                    "Rate-limiting to %.1f Hz",
                    1.0 / CALLBACK_PERIOD_LIMIT);
            }
        }

        /* ---- publish odom ---- */
        if (is_measurement_valid)
            publish_odom(x_estimate_, y_estimate_, yaw_estimate_);

        /* ---- reset per-callback flags ---- */
        solved_new_xy_ = false;
        measured_new_yaw_ = false;
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
