#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "custom_msg/srv/camera_params.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>   // sudo apt install nlohmann-json3-dev

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

static inline Eigen::Matrix3d cv_mat_to_eigen3(const cv::Mat &m)
{
    Eigen::Matrix3d e;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            e(r, c) = m.at<double>(r, c);
    return e;
}

static inline double normalize_angle_deg(double deg)
{
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg - 180.0;
}

static inline double extract_yaw_xyz(const Eigen::Matrix3d &R)
{
    return std::atan2(-R(0, 1), R(0, 0));
}

static inline double extract_box_face_yaw(const Eigen::Matrix3d &R)
{
    return std::asin(std::clamp(R(0, 2), -1.0, 1.0));
}

static inline double extract_roll_xyz(const Eigen::Matrix3d &R)
{
    return std::atan2(-R(1, 2), R(2, 2));
}

static inline bool finite_cv_vec3(const cv::Vec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

static inline bool finite_pose(const geometry_msgs::msg::Pose &pose)
{
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

struct MarkerCandidate {
    cv::Vec3d tvec;
    Eigen::Matrix3d rot_3x3;
    double abs_face_yaw;
    std::vector<cv::Point2f> corners;
};

struct CameraIntrinsics {
    cv::Mat intrinsic;
    cv::Mat distortion;
};

struct CachedMarker {
    geometry_msgs::msg::Pose pose;
    double landmark_map_x;
    double landmark_map_y;
    double ar_angle_deg;
    rclcpp::Time last_seen;
};

using CompImg = sensor_msgs::msg::CompressedImage;

class MultiViewArucoNode : public rclcpp::Node
{
public:
    MultiViewArucoNode()
    : Node("aruco_node")
    {
        declare_parameter("aruco_dictionary_id", "DICT_5X5_250");
        declare_parameter("image_topic_1", "/NAV/feed_camera_nav_0");
        declare_parameter("camera_frame_1", "OAK-1_v1_1");
        declare_parameter("image_topic_2", "/NAV/feed_camera_nav_1");
        declare_parameter("camera_frame_2", "OAK-1_v1_2");
        declare_parameter("image_topic_3", "/NAV/feed_camera_nav_2");
        declare_parameter("camera_frame_3", "OAK-1_v1_3");
        // Cam 3 = OAK-D mono. Topic + frame configurable at launch; intrinsics
        // are HARDCODED below in load_all_intrinsics() (no file / no service).
        declare_parameter("image_topic_4", "/NAV/feed_camera_nav_3");
        declare_parameter("camera_frame_4", "OAK-d");
        declare_parameter("marker_size", 0.144);
        /* Longer default: tags often arrive on different camera callbacks; short TTL drops the other. */
        declare_parameter("marker_cache_ttl_sec", 1.5);
        declare_parameter("marker_fusion_debug", false);
        /* 0 = disable throttled WARN on large per-tag bearing steps. */
        declare_parameter("aruco_bearing_jump_warn_deg", 30.0);
        declare_parameter("aruco_min_marker_perimeter_rate", 0.015);
        declare_parameter("aruco_adaptive_thresh_win_size_max", 23);
        declare_parameter("aruco_error_correction_rate", 0.6);
        declare_parameter("aruco_min_tvec_norm", 0.12);
        /** 0..1: EMA on cached tag position in base_link (0 = off). Reduces jitter from noisy intrinsics. */
        declare_parameter("cache_pose_smooth_alpha", 0.25);

        // --- Calibration mode parameters ---
        // calib_mode: "auto" = fetch from service, "file" = load from JSON
        declare_parameter("calib_mode", std::string("auto"));
        // Directory containing oak_calibration_depthai_{cam_id}.json files
        declare_parameter("calib_dir", std::string(""));
        // Camera serial IDs, one per camera, in order (cam 0, 1, 2, 3)
        // Cam 3 (OAK-D) ignores its cam_ids entry — its intrinsics are hardcoded.
        declare_parameter("cam_ids", std::vector<std::string>{"", "", "", ""});
        // JPEG / camera pipeline output size (camera_node_nav x,y). Used to scale file JSON
        // intrinsics from JSON image_width/height (factory cal resolution).
        declare_parameter("image_stream_width", 1280);
        declare_parameter("image_stream_height", 720);

        marker_size_ = get_parameter("marker_size").as_double();
        calib_mode_  = get_parameter("calib_mode").as_string();
        calib_dir_   = get_parameter("calib_dir").as_string();
        cam_ids_     = get_parameter("cam_ids").as_string_array();
        image_stream_width_  = get_parameter("image_stream_width").as_int();
        image_stream_height_ = get_parameter("image_stream_height").as_int();

        camera_frames_[0] = get_parameter("camera_frame_1").as_string();
        camera_frames_[1] = get_parameter("camera_frame_2").as_string();
        camera_frames_[2] = get_parameter("camera_frame_3").as_string();
        camera_frames_[3] = get_parameter("camera_frame_4").as_string();

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        declare_parameter<std::vector<double>>("landmark_poses", std::vector<double>{});
        auto flat = get_parameter("landmark_poses").as_double_array();
        landmark_poses_.clear();
        if (flat.size() % 2 != 0) {
            RCLCPP_ERROR(get_logger(), "landmark_poses must have an even number of values");
        } else {
            for (size_t i = 0; i + 1 < flat.size(); i += 2)
                landmark_poses_.emplace_back(flat[i], flat[i + 1]);
            RCLCPP_INFO(get_logger(), "Loaded %zu landmark poses", landmark_poses_.size());
        }

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        poses_pub_   = create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10);

        // Start subscribers immediately; intrinsics loaded in background
        setup_subscribers();

        intrinsic_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            intrinsic_timer_->cancel();
            load_all_intrinsics();
            // Retry any failed cameras every 3s
            retry_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {
                bool all_ready = true;
                for (int i = 0; i < NUM_CAMERAS; ++i)
                    if (!intrinsics_ready_[i]) { all_ready = false; break; }
                if (all_ready) { retry_timer_->cancel(); return; }
                load_all_intrinsics();
            });
        });
    }

private:
    static constexpr double ARUCO_BOX_OFFSET    = 0.125;
    static constexpr double MAX_ARUCO_DIST       = 10.0;
    static constexpr int    NUM_CAMERAS          = 4;
    static constexpr int    OAKD_CAM_INDEX       = 3;
    static constexpr int64_t MARKER_CACHE_TTL_NS = 500000000LL;
    static constexpr int64_t CB_MIN_PERIOD_NS    = 100000000LL;
    const std::string base_frame_{"base_link"};

    CameraIntrinsics cam_[NUM_CAMERAS];
    std::array<bool, NUM_CAMERAS> intrinsics_ready_{false, false, false, false};
    std::string camera_frames_[NUM_CAMERAS];
    std::string calib_mode_;
    std::string calib_dir_;
    std::vector<std::string> cam_ids_;
    int image_stream_width_{1280};
    int image_stream_height_{720};

    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    double marker_size_{};

    std::vector<std::pair<double, double>> landmark_poses_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::array<rclcpp::Subscription<CompImg>::SharedPtr, NUM_CAMERAS> image_subs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr            poses_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_pub_;
    std::unordered_map<int64_t, CachedMarker> marker_cache_;
    std::mutex marker_cache_mutex_;

    std::array<int64_t, NUM_CAMERAS> last_cb_ns_{};
    rclcpp::TimerBase::SharedPtr intrinsic_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    /* ============================================================== */
    /*  Hardcoded OAK-D (cam 3) intrinsics                             */
    /*  Provided by user — calibration resolution implied by cx,cy.    */
    /*  NOT rescaled to image_stream_*; the OAK-D feed must arrive at  */
    /*  the native calibration resolution (~1280x640 here).            */
    /* ============================================================== */
    bool load_oakd_hardcoded_intrinsics(int camera_index)
    {
        static constexpr double kFx = 795.8676147460938;
        static constexpr double kFy = 795.8676147460938;
        static constexpr double kCx = 628.2177124023438;
        static constexpr double kCy = 318.68402099609375;

        // OpenCV 14-coeff rational + thin-prism layout:
        //   k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 τx τy
        static const double kDist[14] = {
             9.151253700256348,
             2.9740264415740967,
            -0.0008457531803287566,
             0.00016617916116956621,
           -24.41039276123047,
             8.823257446289062,
             4.046622276306152,
           -25.269325256347656,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        cam_[camera_index].intrinsic = (cv::Mat_<double>(3, 3) <<
            kFx, 0.0, kCx,
            0.0, kFy, kCy,
            0.0, 0.0, 1.0);

        cv::Mat D(1, 14, CV_64F);
        for (int k = 0; k < 14; ++k) D.at<double>(0, k) = kDist[k];
        cam_[camera_index].distortion = D;

        RCLCPP_INFO(get_logger(),
            "Camera %d (OAK-D): HARDCODED intrinsics loaded "
            "fx=%.4f fy=%.4f cx=%.4f cy=%.4f dist_coeffs=14",
            camera_index, kFx, kFy, kCx, kCy);
        return true;
    }

    /* ============================================================== */
    /*  Top-level intrinsics loader                                    */
    /*  Cam 0/1/2 (Oak1W): file or service per calib_mode.             */
    /*  Cam 3       (OAK-D): hardcoded values above (one-shot, no I/O).*/
    /* ============================================================== */
    void load_all_intrinsics()
    {
        // --- OAK-D (cam 3): hardcoded, no file / no service -----------
        if (!intrinsics_ready_[OAKD_CAM_INDEX]) {
            intrinsics_ready_[OAKD_CAM_INDEX] =
                load_oakd_hardcoded_intrinsics(OAKD_CAM_INDEX);
        }

        // --- Cams 0/1/2 (Oak1W): original file/service path ----------
        if (calib_mode_ == "file") {
            for (int i = 0; i < NUM_CAMERAS; ++i) {
                if (i == OAKD_CAM_INDEX) continue;
                if (!intrinsics_ready_[i])
                    intrinsics_ready_[i] = load_intrinsics_from_file(i);
            }
        } else {
            fetch_intrinsics_from_service();
        }
    }

    /* ============================================================== */
    /*  File-based calibration loader                                 */
    /*  Filename: {calib_dir}/oak_calibration_depthai_{cam_id}.json   */
    /* ============================================================== */
    bool load_intrinsics_from_file(int camera_index)
    {
        if (camera_index >= static_cast<int>(cam_ids_.size()) ||
            cam_ids_[camera_index].empty()) {
            RCLCPP_WARN(get_logger(),
                "Camera %d: no cam_id provided for file-based calibration", camera_index);
            return false;
        }

        const std::string filename = calib_dir_ +
            "/oak_calibration_depthai_" + cam_ids_[camera_index] + ".json";

        std::ifstream f(filename);
        if (!f.is_open()) {
            RCLCPP_WARN(get_logger(),
                "Camera %d: cannot open calibration file: %s", camera_index, filename.c_str());
            return false;
        }

        nlohmann::json j;
        try {
            f >> j;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(),
                "Camera %d: JSON parse error in %s: %s", camera_index, filename.c_str(), e.what());
            return false;
        }

        // Parse intrinsic_matrix (3x3 row-major nested array)
        auto K = j.at("intrinsic_matrix");
        double fx = K[0][0], cx = K[0][2];
        double fy = K[1][1], cy = K[1][2];

        int cal_w = 0;
        int cal_h = 0;
        if (j.contains("image_width") && j.contains("image_height")) {
            cal_w = j.at("image_width").get<int>();
            cal_h = j.at("image_height").get<int>();
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Camera %d [%s]: JSON missing image_width/image_height; "
                "intrinsics not scaled to image_stream_* (factory matrix used as-is)",
                camera_index, cam_ids_[camera_index].c_str());
        }
        if (image_stream_width_ > 0 && image_stream_height_ > 0 &&
            cal_w > 0 && cal_h > 0 &&
            (cal_w != image_stream_width_ || cal_h != image_stream_height_)) {
            const double sx =
                static_cast<double>(image_stream_width_) / static_cast<double>(cal_w);
            const double sy =
                static_cast<double>(image_stream_height_) / static_cast<double>(cal_h);
            fx *= sx;
            fy *= sy;
            cx *= sx;
            cy *= sy;
            RCLCPP_INFO(
                get_logger(),
                "Camera %d [%s]: scaled intrinsics %dx%d -> %dx%d (sx=%.5f sy=%.5f); "
                "distortion coeffs unchanged (approximate under anisotropic resize)",
                camera_index, cam_ids_[camera_index].c_str(),
                cal_w, cal_h, image_stream_width_, image_stream_height_, sx, sy);
        }

        if (fx < 1.0 || fy < 1.0 || cx < 1.0 || cy < 1.0) {
            RCLCPP_ERROR(get_logger(),
                "Camera %d: suspicious intrinsics in %s: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                camera_index, filename.c_str(), fx, fy, cx, cy);
            return false;
        }

        cam_[camera_index].intrinsic = (cv::Mat_<double>(3, 3) <<
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);

        // Parse distortion_coeffs (flat array, any length)
        auto d = j.at("distortion_coeffs");
        cv::Mat dist(1, static_cast<int>(d.size()), CV_64F);
        for (size_t k = 0; k < d.size(); ++k)
            dist.at<double>(0, static_cast<int>(k)) = d[k].get<double>();
        cam_[camera_index].distortion = dist;

        RCLCPP_INFO(get_logger(),
            "Camera %d [%s]: loaded from %s — fx=%.1f fy=%.1f cx=%.1f cy=%.1f dist_coeffs=%d",
            camera_index, cam_ids_[camera_index].c_str(), filename.c_str(),
            fx, fy, cx, cy, static_cast<int>(d.size()));
        return true;
    }

    /* ============================================================== */
    /*  Service-based calibration loader (original behaviour)         */
    /* ============================================================== */
    void fetch_intrinsics_from_service()
    {
        rclcpp::NodeOptions opts;
        opts.use_global_arguments(false);
        auto client_node = std::make_shared<rclcpp::Node>("intrinsics_client_tmp_0", opts);

        for (int i = 0; i < NUM_CAMERAS; ++i) {
            // OAK-D (cam 3) is hardcoded — never query a service for it.
            if (i == OAKD_CAM_INDEX) continue;
            if (intrinsics_ready_[i]) continue;

            auto client = client_node->create_client<custom_msg::srv::CameraParams>(
                "/NAV/camera_info_" + std::to_string(i));

            if (!client->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_WARN(get_logger(), "Camera %d service not available, will retry", i);
                continue;
            }

            auto req    = std::make_shared<custom_msg::srv::CameraParams::Request>();
            auto future = client->async_send_request(req);

            if (rclcpp::spin_until_future_complete(client_node, future,
                    std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_WARN(get_logger(), "Camera %d intrinsics timed out, will retry", i);
                continue;
            }

            auto response = future.get();
            if (response->fx < 1.0 || response->fy < 1.0 ||
                response->cx < 1.0 || response->cy < 1.0) {
                RCLCPP_WARN(get_logger(), "Camera %d returned invalid intrinsics, will retry", i);
                continue;
            }

            cam_[i].intrinsic = (cv::Mat_<double>(3, 3) <<
                response->fx, 0.0,          response->cx,
                0.0,          response->fy, response->cy,
                0.0,          0.0,          1.0);
            cam_[i].distortion = cv::Mat(response->distortion_coefficients);
            intrinsics_ready_[i] = true;

            RCLCPP_INFO(get_logger(),
                "Camera %d: service intrinsics OK fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                i, response->fx, response->fy, response->cx, response->cy);
        }
    }

    /* ============================================================== */
    /*  Subscribers                                                   */
    /* ============================================================== */
    void setup_subscribers()
    {
        const std::string topics[NUM_CAMERAS] = {
            get_parameter("image_topic_1").as_string(),
            get_parameter("image_topic_2").as_string(),
            get_parameter("image_topic_3").as_string(),
            get_parameter("image_topic_4").as_string()
        };
        for (int c = 0; c < NUM_CAMERAS; ++c) {
            image_subs_[c] = create_subscription<CompImg>(
                topics[c], rclcpp::SensorDataQoS(),
                [this, c](const CompImg::ConstSharedPtr msg) { image_callback(c, msg); });
        }
    }

    /* ============================================================== */
    /*  Per-camera callback                                           */
    /* ============================================================== */
    void image_callback(int camera_index, const CompImg::ConstSharedPtr &msg)
    {
        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns - last_cb_ns_[camera_index] < CB_MIN_PERIOD_NS) return;
        last_cb_ns_[camera_index] = now_ns;

        if (!intrinsics_ready_[camera_index]) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[cam %d] intrinsics not yet available, dropping frame", camera_index);
            return;
        }

        ros2_aruco_interfaces::msg::ArucoMarkers markers;
        geometry_msgs::msg::PoseArray pose_array;
        markers.header.frame_id    = base_frame_;
        pose_array.header.frame_id = base_frame_;
        markers.header.stamp    = msg->header.stamp;
        pose_array.header.stamp = msg->header.stamp;

        process_image(msg, cam_[camera_index].intrinsic, cam_[camera_index].distortion,
                      camera_frames_[camera_index], markers, pose_array);

        if (!markers.marker_ids.empty())
            update_marker_cache(markers);
        publish_cached_markers();
    }

    // ... (update_marker_cache, publish_cached_markers, process_image,
    //      transform_to_matrix, calculate_aruco_box_bearing,
    //      calculate_bearing_with_fov — all unchanged from your original)

    void update_marker_cache(const ros2_aruco_interfaces::msg::ArucoMarkers &markers)
    {
        const rclcpp::Time now = this->now();
        const size_t n = std::min({
            markers.marker_ids.size(),
            markers.poses.size(),
            markers.landmark_map_pos_x.size(),
            markers.landmark_map_pos_y.size(),
            markers.ar_angles_list.size()
        });

        for (size_t i = 0; i < n; ++i) {
            if (!finite_pose(markers.poses[i]) ||
                !std::isfinite(markers.landmark_map_pos_x[i]) ||
                !std::isfinite(markers.landmark_map_pos_y[i]) ||
                !std::isfinite(markers.ar_angles_list[i])) {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                //     "Skipping invalid cached marker id=%ld: non-finite pose/map/yaw",
                //     markers.marker_ids[i]);
                continue;
            }

            const int64_t mid = markers.marker_ids[i];
            const auto pr = marker_cache_.find(mid);
            if (bearing_jump_warn_deg_ > 0.0 && pr != marker_cache_.end()) {
                const double da = std::abs(normalize_angle_deg(
                    markers.ar_angles_list[i] - pr->second.ar_angle_deg));
                if (da >= bearing_jump_warn_deg_) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "[multiview_aruco] tag idx %lld bearing jump %.1f deg "
                        "(TF vs image stamp, intrinsics, motion blur, marker_size, or camera switch)",
                        static_cast<long long>(mid), da);
                }
            }

            geometry_msgs::msg::Pose pose_out = markers.poses[i];
            if (cache_pose_smooth_alpha_ > 0.0 && pr != marker_cache_.end() &&
                finite_pose(pr->second.pose)) {
                const double a = cache_pose_smooth_alpha_;
                const double o = 1.0 - a;
                pose_out.position.x =
                    a * pose_out.position.x + o * pr->second.pose.position.x;
                pose_out.position.y =
                    a * pose_out.position.y + o * pr->second.pose.position.y;
                pose_out.position.z =
                    a * pose_out.position.z + o * pr->second.pose.position.z;
                /* Orientation kept from current frame; jitter is mostly translation. */
            }

            double ar_deg = markers.ar_angles_list[i];
            if (cache_pose_smooth_alpha_ > 0.0 && pr != marker_cache_.end() &&
                std::isfinite(pr->second.ar_angle_deg)) {
                const double a = cache_pose_smooth_alpha_;
                const double d =
                    normalize_angle_deg(ar_deg - pr->second.ar_angle_deg);
                ar_deg = normalize_angle_deg(pr->second.ar_angle_deg + a * d);
            }

            marker_cache_[mid] = CachedMarker{
                pose_out,
                markers.landmark_map_pos_x[i],
                markers.landmark_map_pos_y[i],
                ar_deg,
                now,
            };
        }
    }

    void publish_cached_markers()
    {
        const rclcpp::Time now = this->now();

        for (auto it = marker_cache_.begin(); it != marker_cache_.end();) {
            if ((now - it->second.last_seen).nanoseconds() > marker_cache_ttl_ns_) {
                it = marker_cache_.erase(it);
            } else {
                ++it;
            }
        }

        if (marker_cache_.empty()) {
            return;
        }

        ros2_aruco_interfaces::msg::ArucoMarkers markers;
        geometry_msgs::msg::PoseArray pose_array;
        markers.header.frame_id = base_frame_;
        pose_array.header.frame_id = base_frame_;
        const int64_t now_ns = now.nanoseconds();
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
        markers.header.stamp = stamp;
        pose_array.header.stamp = stamp;

        std::vector<int64_t> ids;
        ids.reserve(marker_cache_.size());
        for (const auto &entry : marker_cache_) {
            ids.push_back(entry.first);
        }
        std::sort(ids.begin(), ids.end());

        for (const int64_t id : ids) {
            const auto &cached = marker_cache_.at(id);
            if (!finite_pose(cached.pose) ||
                !std::isfinite(cached.landmark_map_x) ||
                !std::isfinite(cached.landmark_map_y) ||
                !std::isfinite(cached.ar_angle_deg)) {
                continue;
            }

            markers.marker_ids.push_back(id);
            markers.poses.push_back(cached.pose);
            markers.landmark_map_pos_x.push_back(cached.landmark_map_x);
            markers.landmark_map_pos_y.push_back(cached.landmark_map_y);
            markers.ar_angles_list.push_back(cached.ar_angle_deg);
            pose_array.poses.push_back(cached.pose);
        }

        if (markers.marker_ids.empty()) {
            return;
        }

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //     "Publishing cached %zu poses, %zu marker_ids",
        //     pose_array.poses.size(), markers.marker_ids.size());

        poses_pub_->publish(pose_array);
        markers_pub_->publish(markers);
    }

    /* ============================================================== */
    /*  Per-camera processing                                         */
    /* ============================================================== */
    void process_image(
        const CompImg::ConstSharedPtr &img_msg,
        const cv::Mat &intrinsic_mat,
        const cv::Mat &distortion,
        const std::string &camera_frame,
        ros2_aruco_interfaces::msg::ArucoMarkers &markers,
        geometry_msgs::msg::PoseArray &pose_array)
        {
            /* ---- decode compressed JPEG ---- */
            cv::Mat raw(1, static_cast<int>(img_msg->data.size()), CV_8UC1,
                        const_cast<uint8_t *>(img_msg->data.data()));
            cv::Mat gray = cv::imdecode(raw, cv::IMREAD_GRAYSCALE);
            if (gray.empty()) return;

            /* ---- detect markers ---- */
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            cv::aruco::detectMarkers(
                gray, dictionary_, corners, ids, parameters_);
            if (!ids.empty()) {
                std::string id_str;
                for (int id : ids) id_str += std::to_string(id) + " ";
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                //     "[%s] detected IDs: %s (img %dx%d)",
                //     camera_frame.c_str(), id_str.c_str(), gray.cols, gray.rows);
            }
            if (ids.empty()) {
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,    "EARLY RETURN: no markers detected on camera %s", camera_frame.c_str());
                return;
            };

            if (intrinsic_mat.empty() || intrinsic_mat.rows != 3 || intrinsic_mat.cols != 3) {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                //     "[%s] invalid camera intrinsic matrix; skipping pose estimation",
                //     camera_frame.c_str());
                return;
            }

            /* ---- estimate poses ---- */
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, static_cast<float>(marker_size_),
                intrinsic_mat, distortion, rvecs, tvecs);

            if (rvecs.size() != ids.size() || tvecs.size() != ids.size()) {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                //     "[%s] pose estimation returned mismatched vector sizes; skipping frame",
                //     camera_frame.c_str());
                return;
            }

            /* ---- TF: camera_frame -> base_link (match image time when possible) ---- */
            geometry_msgs::msg::TransformStamped transform;
            try {
                const bool zero_stamp =
                    (img_msg->header.stamp.sec == 0u &&
                     img_msg->header.stamp.nanosec == 0u);
                if (zero_stamp) {
                    transform = tf_buffer_->lookupTransform(
                        base_frame_, camera_frame, tf2::TimePointZero);
                } else {
                    const rclcpp::Time t(img_msg->header.stamp, get_clock()->get_clock_type());
                    if (tf_buffer_->canTransform(
                            base_frame_, camera_frame, t,
                            rclcpp::Duration::from_seconds(0.05))) {
                        transform = tf_buffer_->lookupTransform(
                            base_frame_, camera_frame, t,
                            rclcpp::Duration::from_seconds(0.1));
                    } else {
                        transform = tf_buffer_->lookupTransform(
                            base_frame_, camera_frame, tf2::TimePointZero);
                    }
                }
            } catch (const tf2::TransformException &ex) {
                try {
                    transform = tf_buffer_->lookupTransform(
                        base_frame_, camera_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &ex2) {
                    (void)ex;
                    (void)ex2;
                    return;
                }
            }

            Eigen::Matrix4d T_base_cam = transform_to_matrix(transform);
            if (!T_base_cam.allFinite()) {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                //     "[%s] base<-camera TF contains non-finite values; skipping frame",
                //     camera_frame.c_str());
                return;
            }

            /* ---- accumulate candidates per marker ID ---- */
            std::unordered_map<int, std::vector<MarkerCandidate>> candidates;

            for (size_t i = 0; i < ids.size(); ++i) {
                if (!finite_cv_vec3(tvecs[i]) || !finite_cv_vec3(rvecs[i])) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d has non-finite rvec/tvec; skipping",
                    //     camera_frame.c_str(), ids[i]);
                    continue;
                }

                double dist = cv::norm(tvecs[i]);                             
                if (!std::isfinite(dist) || dist < aruco_min_tvec_norm_ ||
                    dist > MAX_ARUCO_DIST) {
                    continue;
                }

                int marker_id = ids[i];

                cv::Mat R_cv;
                cv::Rodrigues(rvecs[i], R_cv);
                Eigen::Matrix3d R_tag2cam = cv_mat_to_eigen3(R_cv);
                if (!R_tag2cam.allFinite()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d has non-finite rotation matrix; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }

                static const Eigen::Matrix3d R_180_x =
                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                        .toRotationMatrix();
                Eigen::Matrix3d R_corrected = R_180_x * R_tag2cam.transpose();
                double abs_face_yaw =
                    std::abs(extract_box_face_yaw(R_corrected));
                if (!std::isfinite(abs_face_yaw)) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d has non-finite face yaw; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }

                candidates[marker_id].push_back(
                    {tvecs[i], R_tag2cam, abs_face_yaw, corners[i]});
            }

            /* ---- best-face selection: keep face most parallel to image plane ---- */
            for (auto &[id, cands] : candidates) {
                auto best = std::min_element(cands.begin(), cands.end(),
                    [](const MarkerCandidate &a, const MarkerCandidate &b) {
                        return a.abs_face_yaw < b.abs_face_yaw;
                    });
                if (best != cands.begin())
                    cands.front() = std::move(*best);
                cands.resize(1);
            }

            /* ---- publish best markers (sorted id: stable order vs unordered_map) ---- */
            std::vector<int> cand_ids;
            cand_ids.reserve(candidates.size());
            for (const auto &kv : candidates) {
                cand_ids.push_back(kv.first);
            }
            std::sort(cand_ids.begin(), cand_ids.end());
            for (int marker_id : cand_ids) {
                const std::vector<MarkerCandidate> &cands_vec = candidates[marker_id];
                const MarkerCandidate &mc = cands_vec.front();

                // /* face-to-center offset */
                // Eigen::Vector3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
                // Eigen::Vector3d tvec_eigen(mc.tvec[0], mc.tvec[1], mc.tvec[2]);
                // Eigen::Vector3d adjusted = tvec_eigen + mc.rot_3x3 * offset;

                // /* OpenCV optical -> ROS body convention, then to base_link */
                // Eigen::Vector4d p_cam(adjusted(2), -adjusted(0),
                //                     -adjusted(1), 1.0);
                // Eigen::Vector4d p_base = T_base_cam * p_cam;

                // geometry_msgs::msg::Pose pose;
                // pose.position.x = p_base(0);
                // pose.position.y = p_base(1);
                // pose.position.z = p_base(2);
                // Eigen::Quaterniond q_rot(T_base_cam.block<3, 3>(0, 0));
                // pose.orientation.x = q_rot.x();
                // pose.orientation.y = q_rot.y();
                // pose.orientation.z = q_rot.z();
                // pose.orientation.w = q_rot.w();

                /* face-to-center offset in camera optical frame */
                Eigen::Vector3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
                Eigen::Vector3d tvec_eigen(mc.tvec[0], mc.tvec[1], mc.tvec[2]);
                Eigen::Vector3d box_center_cam = tvec_eigen + mc.rot_3x3 * offset;
                if (!box_center_cam.allFinite()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d has non-finite box center; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }

                // OpenCV optical (x=right, y=down, z=forward)
                // → ROS camera body (x=forward, y=left, z=up)
                static const Eigen::Matrix3d R_opt2ros = (Eigen::Matrix3d() <<
                    0,  0,  1,
                    -1,  0,  0,
                    0, -1,  0).finished();

                Eigen::Vector3d box_center_ros = R_opt2ros * box_center_cam;
                Eigen::Vector4d p_cam_h(box_center_ros(0), box_center_ros(1), box_center_ros(2), 1.0);
                Eigen::Vector4d p_base = T_base_cam * p_cam_h;
                if (!p_base.allFinite()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d transformed pose is non-finite; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }

                Eigen::Matrix3d R_tag_in_base =
                    T_base_cam.block<3,3>(0,0) * R_opt2ros * mc.rot_3x3;
                if (!R_tag_in_base.allFinite()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d transformed rotation is non-finite; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }
                Eigen::Quaterniond q_tag(R_tag_in_base);
                if (!q_tag.coeffs().allFinite()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "[%s] marker id=%d quaternion is non-finite; skipping",
                    //     camera_frame.c_str(), marker_id);
                    continue;
                }

                geometry_msgs::msg::Pose pose;
                pose.position.x = p_base(0);
                pose.position.y = p_base(1);
                pose.position.z = p_base(2);
                pose.orientation.x = q_tag.x();
                pose.orientation.y = q_tag.y();
                pose.orientation.z = q_tag.z();
                pose.orientation.w = q_tag.w();

                /* ERC ID mapping */
                int aruco_index = marker_id - 51;
                if (aruco_index < 0 || aruco_index >= static_cast<int>(landmark_poses_.size())) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    // "SKIP: marker_id %d (aruco_index %d) out of range on camera %s",
                    // marker_id, aruco_index, camera_frame.c_str());
                    continue;
                }
                    
                if (std::find(markers.marker_ids.begin(),
                            markers.marker_ids.end(),
                            aruco_index) != markers.marker_ids.end()) {
                    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    //     "SKIP: marker_id %d (aruco_index %d) already added this frame on camera %s",
                    //     marker_id, aruco_index, camera_frame.c_str());
                    continue;
                }


                /* ---- bearing computation ----
                 *
                 * Downstream (pose_estimator_lidar_node) treats
                 * ar_angles_list[i] as the bearing of marker i expressed
                 * in base_link (i.e. atan2(y_base, x_base) in degrees).
                 *
                 * Compute it directly from the marker pose we already
                 * transformed into base_link above (p_base). This is
                 * geometrically consistent with poses[i] and avoids
                 * mixing a tag-orientation yaw or FOV-pixel angle that
                 * does not match the line from the rover to the
                 * marker. */
                double yaw_deg =
                    std::atan2(p_base(1), p_base(0)) * 180.0 / M_PI;
                yaw_deg = normalize_angle_deg(yaw_deg);
                if (!std::isfinite(yaw_deg)) {
                    continue;
                }

                pose_array.poses.push_back(pose);
                markers.poses.push_back(pose);
                markers.marker_ids.push_back(aruco_index);
                markers.landmark_map_pos_x.push_back(
                    landmark_poses_[aruco_index].first);
                markers.landmark_map_pos_y.push_back(
                    landmark_poses_[aruco_index].second);
                markers.ar_angles_list.push_back(yaw_deg);
            }
    }    

    /* ============================================================== */
    /*  TF → 4×4 matrix                                              */
    /* ============================================================== */
    static Eigen::Matrix4d transform_to_matrix(
        const geometry_msgs::msg::TransformStamped &tf)
    {
        auto &t = tf.transform.translation;
        auto &q = tf.transform.rotation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = quat.toRotationMatrix();
        T(0, 3) = t.x;
        T(1, 3) = t.y;
        T(2, 3) = t.z;
        return T;
    }



    /* ============================================================== */
    /*  Bearing from ArUco rvec (box-center method)                   */
    /* ============================================================== */
    std::pair<double, Eigen::Matrix4d> calculate_aruco_box_bearing(
        const Eigen::Vector3d &tvec, const Eigen::Matrix3d &R_tag) const
    {
        Eigen::Vector3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
        Eigen::Vector3d box_center = tvec + R_tag * offset;

        double bearing_rad = std::atan2(-box_center(0), box_center(2));

        Eigen::Matrix3d R_yaw =
            Eigen::AngleAxisd(bearing_rad, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        Eigen::Matrix4d T_cam_box = Eigen::Matrix4d::Identity();
        T_cam_box.block<3, 3>(0, 0) = R_yaw;
        T_cam_box.block<3, 1>(0, 3) = box_center;

        return {bearing_rad * 180.0 / M_PI, T_cam_box};
    }

    /* ============================================================== */
    /*  FOV-based bearing (RealSense 55°, Brio 41°)                   */
    /* ============================================================== */
    std::optional<double> calculate_bearing_with_fov(
        const cv::Mat &cv_image,
        const std::vector<cv::Point2f> &marker_corners,
        const std::string &camera_frame,
        const geometry_msgs::msg::TransformStamped &transform) const
    {
        double fov_deg = 0.0;
        if (camera_frame == "intel_realsense_D415_camera_top_right_1" ||
            camera_frame == "intel_realsense_D415_camera_top_left_1") {
            fov_deg = 55.0;
        } else if (
            camera_frame == "Logitech_Brio_100_top_right_1" ||
            camera_frame == "Logitech_Brio_100_front_left_v1_1" ||
            camera_frame == "Logitech_Brio_100_front_right_v1_1" ||
            camera_frame == "Logitech_Brio_100_top_left_1") {
            fov_deg = 41.0;
        } else if (
            camera_frame == "OAK-1_v1_1" ||
            camera_frame == "OAK-1_v1_2" ||
            camera_frame == "OAK-1_v1_3") {
            fov_deg = 95.0;
        } else {
            return std::nullopt;
        }

        double w = cv_image.cols;
        double fpx = w / (2.0 * std::tan(fov_deg * M_PI / 360.0));

        double cx = 0.0;
        for (const auto &pt : marker_corners) cx += pt.x;
        cx /= 4.0;

        double bearing_manual_rad = -std::atan2(cx - w * 0.5, fpx);

        auto &q = transform.transform.rotation;
        Eigen::Matrix3d R_cam_base =
            Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();

        double roll  = extract_roll_xyz(R_cam_base);
        double yaw_cam2base = extract_yaw_xyz(R_cam_base);

        double yaw_base_rad = (std::abs(roll) > 0.05)
            ? yaw_cam2base - bearing_manual_rad
            : yaw_cam2base + bearing_manual_rad;

        return normalize_angle_deg(yaw_base_rad * 180.0 / M_PI);
    }
};

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiViewArucoNode>());
    rclcpp::shutdown();
    return 0;
}
