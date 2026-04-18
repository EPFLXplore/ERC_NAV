#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

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

// Y-axis rotation in XYZ intrinsic decomposition.  For an ArUco face on a
// box, the tag Y-axis is vertical, so this is the box yaw: how much the face
// is turned away from the camera.  |value| ≈ 0 ⟹ face is parallel to the
// image plane (most orthogonal to the optical axis).
static inline double extract_box_face_yaw(const Eigen::Matrix3d &R)
{
    return std::asin(std::clamp(R(0, 2), -1.0, 1.0));
}

static inline double extract_roll_xyz(const Eigen::Matrix3d &R)
{
    return std::atan2(-R(1, 2), R(2, 2));
}

/* ------------------------------------------------------------------ */
/*  Marker candidate kept during best-face selection                  */
/* ------------------------------------------------------------------ */

struct MarkerCandidate {
    cv::Vec3d tvec;
    Eigen::Matrix3d rot_3x3;
    double abs_face_yaw;  // box face yaw: 0 = facing camera, π/2 = edge-on
    std::vector<cv::Point2f> corners;
};

/* ------------------------------------------------------------------ */
/*  Camera intrinsics bundle                                          */
/* ------------------------------------------------------------------ */

struct CameraIntrinsics {
    cv::Mat intrinsic;
    cv::Mat distortion;
};

/* ------------------------------------------------------------------ */
/*  Node                                                              */
/* ------------------------------------------------------------------ */

using CompImg = sensor_msgs::msg::CompressedImage;
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<CompImg, CompImg, CompImg>;

class MultiViewArucoNode : public rclcpp::Node
{
public:
    MultiViewArucoNode() : Node("multi_view_aruco_node")
    {
        RCLCPP_INFO(get_logger(), "Multi camera (C++)");

        /* ---- parameters ---- */
        declare_parameter("aruco_dictionary_id", "DICT_5X5_250");
        declare_parameter("image_topic_1", "/NAV/feed_camera_nav_1");
        declare_parameter("camera_frame_1",
                          "intel_realsense_D415_camera_top_right_1");
        declare_parameter("image_topic_2", "/NAV/feed_camera_nav_2");
        declare_parameter("camera_frame_2",
                          "intel_realsense_D415_camera_top_left_1");
        declare_parameter("image_topic_3", "/NAV/feed_camera_nav_0");
        declare_parameter("camera_frame_3", "NAV_Front_Camera_1");
        declare_parameter("marker_size", 0.144);

        marker_size_ =
            get_parameter("marker_size").as_double();

        camera_frames_[0] = get_parameter("camera_frame_1").as_string();
        camera_frames_[1] = get_parameter("camera_frame_2").as_string();
        camera_frames_[2] = get_parameter("camera_frame_3").as_string();

        /* ---- ArUco dictionary & detector parameters ---- */
        dictionary_ = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_5X5_250);
        parameters_ = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod =
            cv::aruco::CORNER_REFINE_SUBPIX;

        /* ---- camera intrinsics (hardcoded) ---- */
        cam_[0].intrinsic = (cv::Mat_<double>(3, 3) <<
            849.81330075, 0.0, 658.24727095,
            0.0, 847.97183304, 345.73492019,
            0.0, 0.0, 1.0);
        cam_[0].distortion = (cv::Mat_<double>(1, 5) <<
            0.08038885, -0.30553617, -0.00123736,
            0.00491224, 0.20331065);

        cam_[1].intrinsic = (cv::Mat_<double>(3, 3) <<
            1.02275342e+03, 0.0, 6.19863827e+02,
            0.0, 1.02106485e+03, 3.72678282e+02,
            0.0, 0.0, 1.0);
        cam_[1].distortion = (cv::Mat_<double>(1, 5) <<
            1.95749872e-01, -1.07702276e+00, 1.00934065e-03,
            -3.30662919e-03, 1.27058253e+00);

        cam_[2].intrinsic = (cv::Mat_<double>(3, 3) <<
            1.96685610e+03, 0.0, 9.99954556e+02,
            0.0, 1.96039881e+03, 4.79997092e+02,
            0.0, 0.0, 1.0);
        cam_[2].distortion = (cv::Mat_<double>(1, 4) <<
            3.03940603e-01, -2.28313655e+00,
            -4.79004397e-03, 3.08347206e-03);

        /* ---- landmark map positions (indexed by aruco_index = id - 51) ---- */

        landmark_poses_ = {
            {2.6, -0.4},            // id 51
            {2.6, 0.4},             // id 52
            {999999, 999999},       // id 53
            {999999, 999999},       // id 54
            {999999, 999999},       // id 55
            {999999, 999999},       // id 56
            {999999, 999999},       // id 57
            {999999, 999999},       // id 58
            {999999, 999999},       // id 59
            {999999, 999999},       // id 60
            {999999, 999999},       // id 61
            {999999, 999999},       // id 62
            {999999, 999999},       // id 63
        }; 

        /* ---- TF ---- */
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        /* ---- publishers ---- */
        poses_pub_ =
            create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        markers_pub_ =
            create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "aruco_markers", 10);

        /* ---- synchronised subscribers (3 cameras) ---- */
        auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
        sub_[0].subscribe(this,
            get_parameter("image_topic_1").as_string(), qos);
        sub_[1].subscribe(this,
            get_parameter("image_topic_2").as_string(), qos);
        sub_[2].subscribe(this,
            get_parameter("image_topic_3").as_string(), qos);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_[0], sub_[1], sub_[2]);
        sync_->registerCallback(std::bind(
            &MultiViewArucoNode::synced_callback, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3));
    }

private:
    /* ---- constants ---- */
    static constexpr double ARUCO_BOX_OFFSET = 0.125;
    static constexpr double MAX_ARUCO_DIST   = 10.0;
    static constexpr int    NUM_CAMERAS      = 3;
    const std::string base_frame_{"base_link"};

    /* ---- camera data ---- */
    CameraIntrinsics cam_[NUM_CAMERAS];
    std::string camera_frames_[NUM_CAMERAS];

    /* ---- ArUco ---- */
    cv::Ptr<cv::aruco::Dictionary>       dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    double marker_size_{};

    /* ---- landmark poses (aruco_index -> (x, y)) ---- */
    std::vector<std::pair<double, double>> landmark_poses_;

    /* ---- TF ---- */
    std::shared_ptr<tf2_ros::Buffer>          tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* ---- subscribers / synchronizer ---- */
    message_filters::Subscriber<CompImg> sub_[NUM_CAMERAS];
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    /* ---- publishers ---- */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr
        markers_pub_;

    /* ---- rate limiter ---- */
    int64_t last_cb_ns_{0};
    static constexpr int64_t CB_MIN_PERIOD_NS = 100000000LL; // 10 Hz

    /* ============================================================== */
    /*  Synchronised callback                                         */
    /* ============================================================== */
    void synced_callback(
        const CompImg::ConstSharedPtr &msg1,
        const CompImg::ConstSharedPtr &msg2,
        const CompImg::ConstSharedPtr &msg3)
    {
        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns - last_cb_ns_ < CB_MIN_PERIOD_NS) return;
        last_cb_ns_ = now_ns;

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        //     "synced_callback fired (img sizes: %zu, %zu, %zu)",
        //     msg1->data.size(), msg2->data.size(), msg3->data.size());

        ros2_aruco_interfaces::msg::ArucoMarkers markers;
        geometry_msgs::msg::PoseArray pose_array;

        markers.header.frame_id   = base_frame_;
        pose_array.header.frame_id = base_frame_;
        markers.header.stamp   = msg1->header.stamp;
        pose_array.header.stamp = msg1->header.stamp;

        const CompImg::ConstSharedPtr msgs[NUM_CAMERAS] = {msg1, msg2, msg3};

        for (int c = 0; c < NUM_CAMERAS; ++c)
            process_image(msgs[c], cam_[c].intrinsic, cam_[c].distortion,
                          camera_frames_[c], markers, pose_array);

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
        // if (!ids.empty()) {
        //     std::string id_str;
        //     for (int id : ids) id_str += std::to_string(id) + " ";
        //     // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        //     //     "[%s] detected IDs: %s (img %dx%d)",
        //     //     camera_frame.c_str(), id_str.c_str(), gray.cols, gray.rows);
        // }
        if (ids.empty()) return;

        /* ---- estimate poses ---- */
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            corners, static_cast<float>(marker_size_),
            intrinsic_mat, distortion, rvecs, tvecs);

        /* ---- TF: camera_frame -> base_link ---- */
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                base_frame_, camera_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "TF %s -> %s unavailable: %s",
                camera_frame.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Matrix4d T_base_cam = transform_to_matrix(transform);

        /* ---- accumulate candidates per marker ID ---- */
        std::unordered_map<int, std::vector<MarkerCandidate>> candidates;

        for (size_t i = 0; i < ids.size(); ++i) {
            if (cv::norm(tvecs[i]) > MAX_ARUCO_DIST) continue;

            int marker_id = ids[i];

            cv::Mat R_cv;
            cv::Rodrigues(rvecs[i], R_cv);
            Eigen::Matrix3d R_tag2cam = cv_mat_to_eigen3(R_cv);

            static const Eigen::Matrix3d R_180_x =
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                    .toRotationMatrix();
            Eigen::Matrix3d R_corrected = R_180_x * R_tag2cam.transpose();
            double abs_face_yaw =
                std::abs(extract_box_face_yaw(R_corrected));

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

        /* ---- publish best markers ---- */
        for (auto &[marker_id, cands] : candidates) {
            const MarkerCandidate &mc = cands.front();

            /* face-to-center offset */
            Eigen::Vector3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
            Eigen::Vector3d tvec_eigen(mc.tvec[0], mc.tvec[1], mc.tvec[2]);
            Eigen::Vector3d adjusted = tvec_eigen + mc.rot_3x3 * offset;

            /* OpenCV optical -> ROS body convention, then to base_link */
            Eigen::Vector4d p_cam(adjusted(2), -adjusted(0),
                                  -adjusted(1), 1.0);
            Eigen::Vector4d p_base = T_base_cam * p_cam;

            geometry_msgs::msg::Pose pose;
            pose.position.x = p_base(0);
            pose.position.y = p_base(1);
            pose.position.z = p_base(2);
            Eigen::Quaterniond q_rot(T_base_cam.block<3, 3>(0, 0));
            pose.orientation.x = q_rot.x();
            pose.orientation.y = q_rot.y();
            pose.orientation.z = q_rot.z();
            pose.orientation.w = q_rot.w();

            /* ERC ID mapping */
            int aruco_index = marker_id - 51;
            if (aruco_index < 0 || aruco_index >= 15) continue;
            if (std::find(markers.marker_ids.begin(),
                          markers.marker_ids.end(),
                          aruco_index) != markers.marker_ids.end())
                continue;

            pose_array.poses.push_back(pose);
            markers.poses.push_back(pose);
            markers.marker_ids.push_back(aruco_index);

            /* ---- bearing computation ---- */
            auto [bearing_deg, T_cam_box] =
                calculate_aruco_box_bearing(tvec_eigen, mc.rot_3x3);
            (void)bearing_deg;   // used only for debug

            Eigen::Matrix4d T_base_box = T_base_cam * T_cam_box;
            Eigen::Matrix3d R_base_box = T_base_box.block<3, 3>(0, 0);
            double yaw_deg = normalize_angle_deg(
                extract_yaw_xyz(R_base_box) * 180.0 / M_PI);

            /* FOV-based override for RealSense / Brio cameras */
            auto fov_yaw =
                calculate_bearing_with_fov(gray, mc.corners,
                                           camera_frame, transform);
            if (fov_yaw.has_value())
                yaw_deg = fov_yaw.value();

            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            //     "tag %d bearing rv frame: %.2f°",
            //     aruco_index + 51, yaw_deg);

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
