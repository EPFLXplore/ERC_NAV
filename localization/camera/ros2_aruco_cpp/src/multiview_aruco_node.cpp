#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

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

/* ------------------------------------------------------------------ */
/*  Per-camera configuration                                          */
/* ------------------------------------------------------------------ */

enum class CameraModel { PINHOLE, FISHEYE };

struct CameraConfig {
    std::string config_filename;
    std::string frame_id;       // URDF link name (TF frame)
    std::string image_topic;    // sensor_msgs/CompressedImage or sensor_msgs/Image topic
    bool compressed_image{true}; // true: CompressedImage (JPEG/PNG); false: raw Image
    std::string hardware_model; // free-text label, logging only
    CameraModel camera_model{CameraModel::PINHOLE};

    cv::Mat intrinsic;   // 3x3 CV_64F, at calibration resolution
    cv::Mat distortion;  // 1xN CV_64F: N=14 (pinhole) or 4 (fisheye)

    int calib_width{0};
    int calib_height{0};

    // Runtime rescale cache: cached_scaled_intrinsic is `intrinsic` scaled
    // from (calib_width, calib_height) to the actual decoded frame size,
    // recomputed only when that frame size changes.
    bool rescale_cached{false};
    int cached_frame_w{0};
    int cached_frame_h{0};
    cv::Mat cached_scaled_intrinsic;

    int64_t last_cb_ns{0};
};

struct CachedMarker {
    geometry_msgs::msg::Pose pose;
    double landmark_map_x;
    double landmark_map_y;
    double ar_angle_deg;
    rclcpp::Time last_seen;
};

using CompImg = sensor_msgs::msg::CompressedImage;
using RawImg  = sensor_msgs::msg::Image;

class MultiViewArucoNode : public rclcpp::Node
{
public:
    MultiViewArucoNode()
    : Node("aruco_node")
    {
        declare_parameter("aruco_dictionary_id", "DICT_5X5_250");
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

        // --- Per-camera JSON configuration ---
        // Directory containing per-camera config JSON files (see CameraConfig).
        declare_parameter("camera_config_dir", std::string(""));
        // Filenames (relative to camera_config_dir) to load, one per camera.
        // Adding a camera = drop a new JSON file here + append its filename.
        declare_parameter("camera_config_files", std::vector<std::string>{});

        marker_size_ = get_parameter("marker_size").as_double();
        const double marker_cache_ttl_sec =
            get_parameter("marker_cache_ttl_sec").as_double();
        marker_cache_ttl_ns_ = static_cast<int64_t>(
            std::max(0.0, marker_cache_ttl_sec) * 1e9);
        bearing_jump_warn_deg_ =
            get_parameter("aruco_bearing_jump_warn_deg").as_double();
        cache_pose_smooth_alpha_ = std::clamp(
            get_parameter("cache_pose_smooth_alpha").as_double(), 0.0, 1.0);
        aruco_min_tvec_norm_ = std::max(
            0.0, get_parameter("aruco_min_tvec_norm").as_double());

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->minMarkerPerimeterRate = std::max(
            0.0, get_parameter("aruco_min_marker_perimeter_rate").as_double());
        parameters_->adaptiveThreshWinSizeMax = static_cast<int>(std::max<int64_t>(
            3, get_parameter("aruco_adaptive_thresh_win_size_max").as_int()));
        parameters_->errorCorrectionRate = std::clamp(
            get_parameter("aruco_error_correction_rate").as_double(), 0.0, 1.0);
        parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

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

        const std::string camera_config_dir =
            get_parameter("camera_config_dir").as_string();
        const std::vector<std::string> camera_config_files =
            get_parameter("camera_config_files").as_string_array();
        cameras_ = load_camera_configs(camera_config_dir, camera_config_files, get_logger());
        if (cameras_.empty()) {
            RCLCPP_ERROR(get_logger(),
                "No camera configs loaded successfully; node will not process any images. "
                "Check camera_config_dir/camera_config_files.");
        }

        setup_subscribers();
    }

private:
    static constexpr double ARUCO_BOX_OFFSET    = 0.125;
    static constexpr double MAX_ARUCO_DIST       = 10.0;
    static constexpr int64_t CB_MIN_PERIOD_NS    = 100000000LL;
    const std::string base_frame_{"base_link"};

    std::vector<CameraConfig> cameras_;

    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    double marker_size_{};
    int64_t marker_cache_ttl_ns_{500000000LL};
    double bearing_jump_warn_deg_{30.0};
    double cache_pose_smooth_alpha_{0.25};
    double aruco_min_tvec_norm_{0.12};

    std::vector<std::pair<double, double>> landmark_poses_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr>                       image_subs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr            poses_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_pub_;
    std::unordered_map<int64_t, CachedMarker> marker_cache_;
    std::mutex marker_cache_mutex_;

    /* ============================================================== */
    /*  camera_model string <-> enum                                   */
    /* ============================================================== */
    static bool parse_camera_model(const std::string &s, CameraModel &out)
    {
        if (s == "pinhole") { out = CameraModel::PINHOLE; return true; }
        if (s == "fisheye") { out = CameraModel::FISHEYE; return true; }
        return false;
    }

    /* ============================================================== */
    /*  Per-camera JSON config loading                                 */
    /*  Filename: {camera_config_dir}/{camera_config_files[i]}         */
    /* ============================================================== */
    static std::optional<CameraConfig> load_single_camera_config(
        const std::string &filepath, rclcpp::Logger logger)
    {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            RCLCPP_ERROR(logger, "Camera config: cannot open file: %s", filepath.c_str());
            return std::nullopt;
        }

        nlohmann::json j;
        try {
            f >> j;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "Camera config: JSON parse error in %s: %s",
                filepath.c_str(), e.what());
            return std::nullopt;
        }

        for (const char *field : {"frame_id", "image_topic", "camera_model",
                                   "intrinsic_matrix", "distortion_coeffs",
                                   "image_width", "image_height"}) {
            if (!j.contains(field)) {
                RCLCPP_ERROR(logger, "Camera config %s: missing required field \"%s\"",
                    filepath.c_str(), field);
                return std::nullopt;
            }
        }

        CameraConfig cfg;
        cfg.config_filename = filepath;
        cfg.frame_id        = j.at("frame_id").get<std::string>();
        cfg.image_topic     = j.at("image_topic").get<std::string>();
        // Optional; defaults to true so existing configs (all CompressedImage) keep working.
        cfg.compressed_image = j.value("compressed_image", true);
        cfg.hardware_model  = j.value("hardware_model", std::string());

        const std::string model_str = j.at("camera_model").get<std::string>();
        if (!parse_camera_model(model_str, cfg.camera_model)) {
            RCLCPP_ERROR(logger,
                "Camera config %s: camera_model must be \"pinhole\" or \"fisheye\", got \"%s\"",
                filepath.c_str(), model_str.c_str());
            return std::nullopt;
        }

        auto K = j.at("intrinsic_matrix");
        const double fx = K[0][0], cx = K[0][2];
        const double fy = K[1][1], cy = K[1][2];
        if (fx < 1.0 || fy < 1.0 || cx < 1.0 || cy < 1.0) {
            RCLCPP_ERROR(logger,
                "Camera config %s: suspicious intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                filepath.c_str(), fx, fy, cx, cy);
            return std::nullopt;
        }
        cfg.intrinsic = (cv::Mat_<double>(3, 3) <<
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);

        auto d = j.at("distortion_coeffs");
        const size_t n_dist = d.size();
        if (cfg.camera_model == CameraModel::FISHEYE && n_dist != 4) {
            RCLCPP_ERROR(logger,
                "Camera config %s: fisheye camera_model requires exactly 4 distortion "
                "coefficients (k1,k2,k3,k4), got %zu", filepath.c_str(), n_dist);
            return std::nullopt;
        }
        if (cfg.camera_model == CameraModel::PINHOLE &&
            n_dist != 4 && n_dist != 5 && n_dist != 8 && n_dist != 12 && n_dist != 14) {
            RCLCPP_ERROR(logger,
                "Camera config %s: pinhole distortion_coeffs length %zu is not one of "
                "OpenCV's accepted lengths {4,5,8,12,14}", filepath.c_str(), n_dist);
            return std::nullopt;
        }
        cv::Mat dist(1, static_cast<int>(n_dist), CV_64F);
        for (size_t k = 0; k < n_dist; ++k)
            dist.at<double>(0, static_cast<int>(k)) = d[k].get<double>();
        cfg.distortion = dist;

        cfg.calib_width  = j.at("image_width").get<int>();
        cfg.calib_height = j.at("image_height").get<int>();
        if (cfg.calib_width <= 0 || cfg.calib_height <= 0) {
            RCLCPP_ERROR(logger,
                "Camera config %s: image_width/image_height must be positive (got %dx%d)",
                filepath.c_str(), cfg.calib_width, cfg.calib_height);
            return std::nullopt;
        }

        RCLCPP_INFO(logger,
            "Camera config %s: loaded frame_id=%s topic=%s (%s) model=%s hardware=%s "
            "fx=%.1f fy=%.1f cx=%.1f cy=%.1f dist_coeffs=%zu calib=%dx%d",
            filepath.c_str(), cfg.frame_id.c_str(), cfg.image_topic.c_str(),
            cfg.compressed_image ? "compressed" : "raw",
            model_str.c_str(), cfg.hardware_model.c_str(),
            fx, fy, cx, cy, n_dist, cfg.calib_width, cfg.calib_height);
        return cfg;
    }

    static std::vector<CameraConfig> load_camera_configs(
        const std::string &camera_config_dir,
        const std::vector<std::string> &camera_config_files,
        rclcpp::Logger logger)
    {
        std::vector<CameraConfig> cameras;
        cameras.reserve(camera_config_files.size());
        for (const auto &filename : camera_config_files) {
            const std::string filepath = camera_config_dir + "/" + filename;
            auto cfg = load_single_camera_config(filepath, logger);
            if (cfg) {
                cameras.push_back(std::move(*cfg));
            } else {
                RCLCPP_ERROR(logger, "Skipping camera config (failed to load): %s",
                    filepath.c_str());
            }
        }
        return cameras;
    }

    /* ============================================================== */
    /*  Subscribers                                                   */
    /* ============================================================== */
    void setup_subscribers()
    {
        image_subs_.resize(cameras_.size());
        for (size_t c = 0; c < cameras_.size(); ++c) {
            if (cameras_[c].compressed_image) {
                image_subs_[c] = create_subscription<CompImg>(
                    cameras_[c].image_topic, rclcpp::SensorDataQoS(),
                    [this, c](const CompImg::ConstSharedPtr msg) {
                        image_callback_compressed(c, msg);
                    });
            } else {
                image_subs_[c] = create_subscription<RawImg>(
                    cameras_[c].image_topic, rclcpp::SensorDataQoS(),
                    [this, c](const RawImg::ConstSharedPtr msg) {
                        image_callback_raw(c, msg);
                    });
            }
        }
    }

    /* ============================================================== */
    /*  Per-camera callbacks                                          */
    /* ============================================================== */
    void image_callback_compressed(size_t camera_index, const CompImg::ConstSharedPtr &msg)
    {
        cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
                    const_cast<uint8_t *>(msg->data.data()));
        cv::Mat gray = cv::imdecode(raw, cv::IMREAD_GRAYSCALE);
        handle_frame(camera_index, gray, msg->header);
    }

    void image_callback_raw(size_t camera_index, const RawImg::ConstSharedPtr &msg)
    {
        cv::Mat gray = raw_image_to_gray(*msg);
        handle_frame(camera_index, gray, msg->header);
    }

    /* Convert a raw sensor_msgs/Image to single-channel grayscale, no lossy re-encoding. */
    cv::Mat raw_image_to_gray(const sensor_msgs::msg::Image &msg)
    {
        namespace enc = sensor_msgs::image_encodings;
        cv::Mat gray;
        if (msg.encoding == enc::MONO8) {
            gray = cv::Mat(msg.height, msg.width, CV_8UC1,
                            const_cast<uint8_t *>(msg.data.data()), msg.step).clone();
        } else if (msg.encoding == enc::BGR8) {
            cv::Mat bgr(msg.height, msg.width, CV_8UC3,
                        const_cast<uint8_t *>(msg.data.data()), msg.step);
            cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        } else if (msg.encoding == enc::RGB8) {
            cv::Mat rgb(msg.height, msg.width, CV_8UC3,
                        const_cast<uint8_t *>(msg.data.data()), msg.step);
            cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
        } else if (msg.encoding == enc::BGRA8) {
            cv::Mat bgra(msg.height, msg.width, CV_8UC4,
                         const_cast<uint8_t *>(msg.data.data()), msg.step);
            cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
        } else if (msg.encoding == enc::RGBA8) {
            cv::Mat rgba(msg.height, msg.width, CV_8UC4,
                         const_cast<uint8_t *>(msg.data.data()), msg.step);
            cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
        } else {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "raw_image_to_gray: unsupported encoding \"%s\"", msg.encoding.c_str());
        }
        return gray;
    }

    void handle_frame(size_t camera_index, const cv::Mat &gray, const std_msgs::msg::Header &header)
    {
        if (gray.empty()) return;
        CameraConfig &cam = cameras_[camera_index];

        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns - cam.last_cb_ns < CB_MIN_PERIOD_NS) return;
        cam.last_cb_ns = now_ns;

        ros2_aruco_interfaces::msg::ArucoMarkers markers;
        geometry_msgs::msg::PoseArray pose_array;
        markers.header.frame_id    = base_frame_;
        pose_array.header.frame_id = base_frame_;
        markers.header.stamp    = header.stamp;
        pose_array.header.stamp = header.stamp;

        process_image(gray, header, cam, markers, pose_array);

        if (!markers.marker_ids.empty())
            update_marker_cache(markers);
        publish_cached_markers();
    }

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

        poses_pub_->publish(pose_array);
        markers_pub_->publish(markers);
    }

    /* ============================================================== */
    /*  Per-camera intrinsics rescale (calibration res -> decoded frame res) */
    /* ============================================================== */
    static void rescale_intrinsics_for_frame(CameraConfig &cam, int frame_w, int frame_h)
    {
        if (cam.rescale_cached &&
            cam.cached_frame_w == frame_w && cam.cached_frame_h == frame_h) {
            return;
        }

        if (frame_w == cam.calib_width && frame_h == cam.calib_height) {
            cam.cached_scaled_intrinsic = cam.intrinsic;
        } else {
            const double sx = static_cast<double>(frame_w) / static_cast<double>(cam.calib_width);
            const double sy = static_cast<double>(frame_h) / static_cast<double>(cam.calib_height);
            cv::Mat scaled = cam.intrinsic.clone();
            scaled.at<double>(0, 0) *= sx; // fx
            scaled.at<double>(1, 1) *= sy; // fy
            scaled.at<double>(0, 2) *= sx; // cx
            scaled.at<double>(1, 2) *= sy; // cy
            cam.cached_scaled_intrinsic = scaled;
            RCLCPP_INFO(rclcpp::get_logger("aruco_node"),
                "[%s] rescaled intrinsics %dx%d -> %dx%d (sx=%.5f sy=%.5f)",
                cam.frame_id.c_str(), cam.calib_width, cam.calib_height,
                frame_w, frame_h, sx, sy);
        }

        cam.rescale_cached  = true;
        cam.cached_frame_w  = frame_w;
        cam.cached_frame_h  = frame_h;
    }

    /* ============================================================== */
    /*  Per-camera processing                                         */
    /* ============================================================== */
    void process_image(
        const cv::Mat &gray,
        const std_msgs::msg::Header &header,
        CameraConfig &cam,
        ros2_aruco_interfaces::msg::ArucoMarkers &markers,
        geometry_msgs::msg::PoseArray &pose_array)
        {
            const std::string &camera_frame = cam.frame_id;

            rescale_intrinsics_for_frame(cam, gray.cols, gray.rows);

            /* ---- detect markers ---- */
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            cv::aruco::detectMarkers(
                gray, dictionary_, corners, ids, parameters_);
            if (ids.empty()) {
                return;
            };

            const cv::Mat &intrinsic_mat = cam.cached_scaled_intrinsic;
            if (intrinsic_mat.empty() || intrinsic_mat.rows != 3 || intrinsic_mat.cols != 3) {
                return;
            }

            /* ---- fisheye: undistort detected corners before pose estimation ----
             * cv::aruco::estimatePoseSingleMarkers (via solvePnP) assumes a
             * rectilinear/pinhole distortion model and cannot correctly consume
             * fisheye (equidistant) distortion coefficients. Undistort the
             * corner points first, then feed zero distortion below. */
            if (cam.camera_model == CameraModel::FISHEYE) {
                for (auto &marker_corners : corners) {
                    std::vector<cv::Point2f> undistorted;
                    cv::fisheye::undistortPoints(
                        marker_corners, undistorted,
                        intrinsic_mat, cam.distortion,
                        cv::noArray(), intrinsic_mat);
                    marker_corners = undistorted;
                }
            }

            const cv::Mat distortion = (cam.camera_model == CameraModel::FISHEYE)
                ? cv::Mat::zeros(1, 14, CV_64F)
                : cam.distortion;

            /* ---- estimate poses ---- */
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, static_cast<float>(marker_size_),
                intrinsic_mat, distortion, rvecs, tvecs);

            if (rvecs.size() != ids.size() || tvecs.size() != ids.size()) {
                return;
            }

            /* ---- TF: camera_frame -> base_link (match image time when possible) ---- */
            geometry_msgs::msg::TransformStamped transform;
            try {
                const bool zero_stamp =
                    (header.stamp.sec == 0u &&
                     header.stamp.nanosec == 0u);
                if (zero_stamp) {
                    transform = tf_buffer_->lookupTransform(
                        base_frame_, camera_frame, tf2::TimePointZero);
                } else {
                    const rclcpp::Time t(header.stamp, get_clock()->get_clock_type());
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
                return;
            }

            /* ---- accumulate candidates per marker ID ---- */
            std::unordered_map<int, std::vector<MarkerCandidate>> candidates;

            for (size_t i = 0; i < ids.size(); ++i) {
                if (!finite_cv_vec3(tvecs[i]) || !finite_cv_vec3(rvecs[i])) {
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
                    continue;
                }

                static const Eigen::Matrix3d R_180_x =
                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                        .toRotationMatrix();
                Eigen::Matrix3d R_corrected = R_180_x * R_tag2cam.transpose();
                double abs_face_yaw =
                    std::abs(extract_box_face_yaw(R_corrected));
                if (!std::isfinite(abs_face_yaw)) {
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

                /* face-to-center offset in camera optical frame */
                Eigen::Vector3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
                Eigen::Vector3d tvec_eigen(mc.tvec[0], mc.tvec[1], mc.tvec[2]);
                Eigen::Vector3d box_center_cam = tvec_eigen + mc.rot_3x3 * offset;
                if (!box_center_cam.allFinite()) {
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
                    continue;
                }

                Eigen::Matrix3d R_tag_in_base =
                    T_base_cam.block<3,3>(0,0) * R_opt2ros * mc.rot_3x3;
                if (!R_tag_in_base.allFinite()) {
                    continue;
                }
                Eigen::Quaterniond q_tag(R_tag_in_base);
                if (!q_tag.coeffs().allFinite()) {
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
                    continue;
                }

                if (std::find(markers.marker_ids.begin(),
                            markers.marker_ids.end(),
                            aruco_index) != markers.marker_ids.end()) {
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
