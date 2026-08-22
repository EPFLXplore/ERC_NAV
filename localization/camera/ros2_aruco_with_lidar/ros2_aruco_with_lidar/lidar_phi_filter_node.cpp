/*
 * Conceptual state machine: camera-guided LiDAR landmark filter
 *
 * STATE 0 - INITIALIZE
 *   Load filtering parameters and known landmark positions, create the TF
 *   listener, subscribe to the raw LiDAR cloud and camera ArUco detections,
 *   and create the filtered-cloud, association, and visualization publishers.
 *
 * STATE 1 - WAIT FOR INPUT
 *   The node waits independently for either a camera detection message or a
 *   LiDAR point cloud. Camera messages update the association state; point
 *   clouds attempt to use the latest valid association state. The configured
 *   mode chooses map landmarks or camera-measured tag centers as crop origins.
 *
 * STATE 2 - UPDATE CAMERA ASSOCIATIONS (ArUco callback)
 *   Validate the parallel fields in the message and inspect every detection.
 *   Detections hidden behind the drill are excluded from LiDAR filtering and
 *   published separately as forbidden-sector cube hypotheses. The remaining
 *   detections are sorted by range, republished for downstream LiDAR/cube
 *   association, and stored with their IDs, bearings, ranges, map positions,
 *   and update time. Transition back to STATE 1.
 *
 * STATE 3 - GATE A LIDAR CLOUD (point-cloud callback)
 *   Enforce the processing-rate limit, copy the latest camera association
 *   state under its mutex, and reject the cloud when no detections exist or
 *   when they are older than the configured TTL. Stale state is cleared.
 *   A rejected cloud transitions directly back to STATE 1.
 *
 * STATE 4 - BUILD LANDMARK SEARCH REGIONS
 *   Remove duplicate or invalid marker associations, transform each selected
 *   map landmark or camera center into the cloud frame at the cloud timestamp,
 *   and build a coarse bounding box plus a circular XY search region around
 *   every valid landmark. Missing frames or transforms return to STATE 1.
 *
 * STATE 5 - FILTER POINTS
 *   Read XYZ points, transform their height into map (landmark mode) or
 *   base_link (camera mode), reject points outside the configured Z interval,
 *   XY bounding box, then assign each point to its uniquely nearest search
 *   center. Points too close to the boundary between two IDs are discarded.
 *
 * STATE 6 - PUBLISH OR DISCARD
 *   Publish surviving XYZ points with a marker_id field so downstream cube
 *   fitting cannot mix overlapping search zones. Otherwise publish nothing.
 *   In both cases transition back to STATE 1 for the next input.
 *
 * A periodic side path publishes configured landmark zones. Camera mode also
 * publishes its active truncated-cone zones; visualization does not change state.
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <mutex>
#include <string>
#include <list>
#include <array>
#include <utility>
#include <cstdint>
#include <limits>
#include <atomic>
#include <stdexcept>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
// pas de placeholders: on utilise des lambdas pour les callbacks

namespace {

inline double wrap180(double deg) {
    double d = fmod(deg + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}

inline double angDeltaDeg(double a_deg, double b_deg) {
    return fabs(wrap180(b_deg - a_deg));
}

} // namespace



class LidarPhiFilterNode : public rclcpp::Node {
public:
    LidarPhiFilterNode() : rclcpp::Node("lidar_phi_filter_node"), tf_buffer_(this->get_clock()) {
        this->declare_parameter<double>("tolerance_deg", 15.0);
        this->declare_parameter<double>("tolerance_radius", 0.35);
        this->declare_parameter<double>("hauteur_z_min", -0.7);
        this->declare_parameter<double>("hauteur_z_max", 1.0);
        this->declare_parameter<double>("camera_detection_ttl_sec", 0.8);
        this->declare_parameter<bool>("use_camera_aruco_position", false);
        this->declare_parameter<double>("camera_cone_half_angle_deg", 5.0);
        this->declare_parameter<double>("camera_cone_depth_tolerance_m", 1.0);
        this->declare_parameter<double>("camera_cone_depth_tolerance_ratio", 0.20);
        this->declare_parameter<double>("association_ambiguity_margin_m", 0.05);
        // this->declare_parameter<double>("lidar_yaw_offset_deg", 270.0);
        this->declare_parameter<std::vector<double>>("landmark_poses", std::vector<double>{});

        // this->get_parameter("lidar_yaw_offset_deg", lidar_yaw_offset_deg_);
        this->get_parameter("tolerance_deg", tolerance_deg_);
        this->get_parameter("tolerance_radius", tolerance_radius_);
        this->get_parameter("hauteur_z_min", hauteur_z_min);
        this->get_parameter("hauteur_z_max", hauteur_z_max);
        bool initial_camera_mode = false;
        this->get_parameter("use_camera_aruco_position", initial_camera_mode);
        use_camera_aruco_position_.store(initial_camera_mode);
        this->get_parameter("camera_cone_half_angle_deg", camera_cone_half_angle_deg_);
        this->get_parameter(
            "camera_cone_depth_tolerance_m", camera_cone_depth_tolerance_m_);
        this->get_parameter(
            "camera_cone_depth_tolerance_ratio", camera_cone_depth_tolerance_ratio_);
        this->get_parameter(
            "association_ambiguity_margin_m", association_ambiguity_margin_m_);
        double camera_detection_ttl_sec = 0.8;
        this->get_parameter("camera_detection_ttl_sec", camera_detection_ttl_sec);
        this->get_parameter("landmark_poses", flat);        

        if (!(camera_cone_half_angle_deg_ > 0.0) ||
            !(camera_cone_half_angle_deg_ < 90.0) ||
            !(camera_cone_depth_tolerance_m_ > 0.0) ||
            !(camera_cone_depth_tolerance_ratio_ >= 0.0)) {
            throw std::runtime_error("invalid camera recovery cone parameters");
        }
        
        landmark_poses_.clear();
        for (size_t i = 0; i + 1 < flat.size(); i += 2) {
            landmark_poses_.emplace_back(flat[i], flat[i + 1]);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu landmark poses", landmark_poses_.size());
        

        camera_detection_ttl_ns_ = static_cast<int64_t>(camera_detection_ttl_sec * 1e9);
        input_cloud_topic_ = "/ouster/points";
        output_cloud_topic_ = "/lidar/points_near_camera_aruco_landmarks";
        aruco_topic_ = "/perception/aruco_markers_cross_id_validated";
        selected_count_ = 0;
        min_cloud_period_ns_ = static_cast<int64_t>(1e9 / 5.0); // 5 Hz max 
        last_cloud_time_ns_ = 0;
        last_camera_update_ns_ = 0;

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                filter_pcl_with_known_aruco_poses(msg);
            });
        markers_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            aruco_topic_, qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                select_camera_detections_for_filter(msg);
            });
        auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        mode_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/perception/use_camera_aruco_position", mode_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                set_camera_mode(msg->data, "recovery supervisor");
                if (get_parameter("use_camera_aruco_position").as_bool() != msg->data) {
                    const auto result = set_parameter(rclcpp::Parameter(
                        "use_camera_aruco_position", msg->data));
                    if (!result.successful) {
                        RCLCPP_ERROR(get_logger(),
                            "Failed to mirror recovery mode into parameter: %s",
                            result.reason.c_str());
                    }
                }
            });
        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &parameter : parameters) {
                    if (parameter.get_name() == "use_camera_aruco_position") {
                        set_camera_mode(parameter.as_bool(), "parameter update");
                    }
                }
                return result;
            });

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
    camera_recovery_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar/points_near_camera_aruco", qos);
    centre_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/cube_centers", qos);
    /* Separate from detect_cube (/perception/lidar_cube_markers): pose_estimator merges both. */
    cube_markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/perception/camera_forbidden_sector_cube_markers", qos);
    /* Tags usable for LiDAR crop + line RANSAC: same gate as for_filter (not in cube_markers_phi). */
    aruco_markers_lidar_assoc_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/perception/aruco_markers_for_lidar_association", qos);

    landmark_circles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/aruco_landmark_search_zones", qos);
    camera_search_zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/camera_aruco_search_zones", qos);
    landmark_timer_ = create_wall_timer(std::chrono::milliseconds(2000), [this]() {
        publish_landmark_circles();
    });
    }

private:
    vector<std::pair<double, double>> landmark_poses_;

    void set_camera_mode(bool enabled, const char *source)
    {
        const bool previous = use_camera_aruco_position_.exchange(enabled);
        if (previous == enabled) return;

        // A cloud and an association generated on opposite sides of the mode
        // switch must never be combined. Wait for a fresh camera batch.
        {
            std::lock_guard<std::mutex> lock(angles_mutex_);
            selected_angles_aruco_deg_.clear();
            selected_ranges_.clear();
            selected_marker_ids_.clear();
            selected_landmark_map_positions_.clear();
            selected_camera_positions_.clear();
            selected_camera_frame_.clear();
            selected_camera_stamp_ = builtin_interfaces::msg::Time();
            selected_count_ = 0;
            last_camera_update_ns_ = 0;
        }
        RCLCPP_WARN(get_logger(),
            "ArUco LiDAR crop mode changed to %s by %s; cached associations cleared",
            enabled ? "CAMERA_GUIDED" : "MAP_GUIDED", source);
    }


    bool lookup_cloud_to_height_frame_transform(
        const std::string &cloud_frame,
        const std::string &height_frame,
        const builtin_interfaces::msg::Time &cloud_stamp,
        geometry_msgs::msg::TransformStamped &transform)
    {
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        height_frame, cloud_frame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                    return false;
                }
                transform = tf_buffer_.lookupTransform(
                    height_frame, cloud_frame, tf2::TimePointZero);
                return true;
            }

            const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
            if (tf_buffer_.canTransform(
                    height_frame, cloud_frame, t, rclcpp::Duration::from_seconds(0.1))) {
                transform = tf_buffer_.lookupTransform(
                    height_frame, cloud_frame, t, rclcpp::Duration::from_seconds(0.1));
                return true;
            }

            if (!tf_buffer_.canTransform(
                    height_frame, cloud_frame, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                return false;
            }
            transform = tf_buffer_.lookupTransform(
                height_frame, cloud_frame, tf2::TimePointZero);
            return true;
        } catch (const tf2::TransformException &) {
            return false;
        }
    }


    vector<std::pair<double, double>> get_aruco_poses_in_frame(
        const vector<std::pair<double, double>> &aruco_map_positions,
        const std::string &target_frame,
        const builtin_interfaces::msg::Time &cloud_stamp)
    {
        // in: list of (x, y) map positions of the center of aruco boxes. Given by the ERC.
        // out: list of (x, y) positions of the center of aruco boxes in target_frame.
        // These positions are used to select points directly in the point cloud frame.
        //
        // Use the cloud header stamp for map→target_frame. Points in the cloud are expressed
        // at that time; using tf2::TimePointZero ("latest") drifts from the scan once the
        // robot moves or map→odom is updated, while /aruco_markers can still look correct.

        geometry_msgs::msg::TransformStamped transform;
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        target_frame, "map", tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                    return {};
                }
                transform = tf_buffer_.lookupTransform(target_frame, "map", tf2::TimePointZero);
            } else {
                const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
                if (tf_buffer_.canTransform(
                        target_frame, "map", t, rclcpp::Duration::from_seconds(0.1))) {
                    transform = tf_buffer_.lookupTransform(
                        target_frame, "map", t, rclcpp::Duration::from_seconds(0.1));
                } else {
                    /* Buffer not populated up to cloud time yet — latest is better than no filter. */
                    if (!tf_buffer_.canTransform(
                            target_frame, "map", tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                        return {};
                    }
                    transform = tf_buffer_.lookupTransform(target_frame, "map", tf2::TimePointZero);
                }
            }
        } catch (const tf2::TransformException &ex) {
            (void)ex;
            try {
                if (!tf_buffer_.canTransform(
                        target_frame, "map", tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                    return {};
                }
                transform = tf_buffer_.lookupTransform(target_frame, "map", tf2::TimePointZero);
            } catch (const tf2::TransformException &) {
                return {};
            }
        }

        static constexpr double MAX_VALID_COORD = 80.0;
        static constexpr double MIN_VALID_COORD = -20.0;
        static constexpr double OUT_OF_BOUNDS_COORD = 999999.0;

        auto out_of_bounds = [](double v) {
            return v >= MAX_VALID_COORD || v <= MIN_VALID_COORD;
        };
        std::vector<std::pair<double, double>> out;
        out.reserve(aruco_map_positions.size());

        geometry_msgs::msg::PointStamped in, tmp;
        in.header.frame_id = "map";
        in.point.z = 0.0;
        
        for (const auto &pos : aruco_map_positions) {
            if (out_of_bounds(pos.first) || out_of_bounds(pos.second)) {
                out.emplace_back(OUT_OF_BOUNDS_COORD, OUT_OF_BOUNDS_COORD);
                continue;
            }
            in.point.x = pos.first;
            in.point.y = pos.second;
            tf2::doTransform(in, tmp, transform);
            out.emplace_back(tmp.point.x, tmp.point.y);
        }
        return out;
    }

    vector<std::pair<double, double>> get_camera_positions_in_frame(
        const vector<geometry_msgs::msg::Point> &camera_positions,
        const std::string &camera_frame,
        const std::string &target_frame,
        const builtin_interfaces::msg::Time &cloud_stamp)
    {
        if (camera_frame.empty()) {
            return {};
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        target_frame, camera_frame, tf2::TimePointZero,
                        tf2::durationFromSec(0.1))) {
                    return {};
                }
                transform = tf_buffer_.lookupTransform(
                    target_frame, camera_frame, tf2::TimePointZero);
            } else {
                const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
                if (tf_buffer_.canTransform(
                        target_frame, camera_frame, t,
                        rclcpp::Duration::from_seconds(0.1))) {
                    transform = tf_buffer_.lookupTransform(
                        target_frame, camera_frame, t,
                        rclcpp::Duration::from_seconds(0.1));
                } else {
                    if (!tf_buffer_.canTransform(
                            target_frame, camera_frame, tf2::TimePointZero,
                            tf2::durationFromSec(0.05))) {
                        return {};
                    }
                    transform = tf_buffer_.lookupTransform(
                        target_frame, camera_frame, tf2::TimePointZero);
                }
            }
        } catch (const tf2::TransformException &) {
            return {};
        }

        vector<std::pair<double, double>> out;
        out.reserve(camera_positions.size());
        for (const auto &position : camera_positions) {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = camera_frame;
            point_in.header.stamp = cloud_stamp;
            point_in.point = position;
            tf2::doTransform(point_in, point_out, transform);
            out.emplace_back(point_out.point.x, point_out.point.y);
        }
        return out;
    }

    void publish_landmark_circles() {
        visualization_msgs::msg::MarkerArray ma;
        int id = 0;
        for (size_t i = 0; i < landmark_poses_.size(); ++i) {
            if (landmark_poses_[i].first > 90000.0) continue;
            visualization_msgs::msg::Marker m;
            m.header.stamp = this->now();
            m.header.frame_id = "map";
            m.ns = "landmark_zones";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = landmark_poses_[i].first;
            m.pose.position.y = landmark_poses_[i].second;
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = 2.0*tolerance_radius_;  // diameter = 2 * 1m radius
            m.scale.y = 2.0*tolerance_radius_;
            m.scale.z = 0.02; // flat disc
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 1.0f;
            m.color.a = 0.3f;
            m.lifetime = rclcpp::Duration(0, 0); // persistent
            ma.markers.push_back(m);
        }
        landmark_circles_pub_->publish(ma);
    }

    void publish_camera_search_cones(
        const vector<std::pair<double, double>> &centers_lidar,
        const vector<int64_t> &marker_ids,
        const std::string &lidar_frame,
        const builtin_interfaces::msg::Time &stamp)
    {
        visualization_msgs::msg::MarkerArray markers;
        const size_t count = std::min(centers_lidar.size(), marker_ids.size());
        markers.markers.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            const double center_x = centers_lidar[i].first;
            const double center_y = centers_lidar[i].second;
            const double expected_range = std::hypot(center_x, center_y);
            if (!(expected_range > 0.0)) continue;

            const double depth_tolerance = std::max(
                camera_cone_depth_tolerance_m_,
                camera_cone_depth_tolerance_ratio_ * expected_range);
            const double inner_range = std::max(0.05, expected_range - depth_tolerance);
            const double outer_range = expected_range + depth_tolerance;
            const double center_angle = std::atan2(center_y, center_x);
            const double half_angle = camera_cone_half_angle_deg_ * M_PI / 180.0;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = lidar_frame;
            marker.header.stamp = stamp;
            marker.ns = "camera_aruco_search_cones";
            marker.id = static_cast<int>(marker_ids[i]);
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 1.0f;
            marker.color.g = 0.55f;
            marker.color.b = 0.0f;
            marker.color.a = 0.25f;
            marker.lifetime = rclcpp::Duration::from_seconds(
                static_cast<double>(camera_detection_ttl_ns_) / 1e9);

            // Flat annular sector matching the horizontal recovery crop. The
            // point-cloud height gate remains visible through the cloud itself.
            constexpr int segments = 16;
            for (int segment = 0; segment < segments; ++segment) {
                const double fraction_0 =
                    static_cast<double>(segment) / static_cast<double>(segments);
                const double fraction_1 =
                    static_cast<double>(segment + 1) / static_cast<double>(segments);
                const double angle_0 = center_angle - half_angle +
                    2.0 * half_angle * fraction_0;
                const double angle_1 = center_angle - half_angle +
                    2.0 * half_angle * fraction_1;

                geometry_msgs::msg::Point inner_0;
                inner_0.x = inner_range * std::cos(angle_0);
                inner_0.y = inner_range * std::sin(angle_0);
                geometry_msgs::msg::Point inner_1;
                inner_1.x = inner_range * std::cos(angle_1);
                inner_1.y = inner_range * std::sin(angle_1);
                geometry_msgs::msg::Point outer_0;
                outer_0.x = outer_range * std::cos(angle_0);
                outer_0.y = outer_range * std::sin(angle_0);
                geometry_msgs::msg::Point outer_1;
                outer_1.x = outer_range * std::cos(angle_1);
                outer_1.y = outer_range * std::sin(angle_1);

                marker.points.push_back(inner_0);
                marker.points.push_back(outer_0);
                marker.points.push_back(outer_1);
                marker.points.push_back(inner_0);
                marker.points.push_back(outer_1);
                marker.points.push_back(inner_1);
            }
            markers.markers.push_back(marker);
        }
        camera_search_zones_pub_->publish(markers);
    }

    // Extraction des coordonnées XYZ
    // void extract_ouster_coordinates(const sensor_msgs::msg::PointCloud2 &msg,
    //                                 std::vector<std::array<float, 3>> &points) {
    //     const size_t W = msg.width;
    //     const size_t H = msg.height;
    //     points.clear();
    //     points.reserve(W * H);

    //     // Convert angular offset to column index offset
    //     // Ouster scans 360° over W columns → each column = 360/W degrees
    //     const int col_offset = static_cast<int>(
    //         std::round(lidar_yaw_offset_deg_ / 360.0 * static_cast<double>(W))
    //     ) % static_cast<int>(W);

    //     for (size_t row = 0; row < H; ++row) {
    //         for (size_t col = 0; col < W; ++col) {
    //             // Wrap the column index with the offset
    //             const size_t src_col = (col + col_offset + W) % W;
    //             const size_t idx = row * W + src_col;

    //             const uint8_t* ptr = &msg.data[idx * msg.point_step];

    //             float x, y, z;
    //             memcpy(&x, ptr + 0,  sizeof(float));  // assumes x at offset 0
    //             memcpy(&y, ptr + 4,  sizeof(float));  // assumes y at offset 4
    //             memcpy(&z, ptr + 8,  sizeof(float));  // assumes z at offset 8

    //             if (!isfinite(x) || !isfinite(y)) continue;
    //             points.push_back({x, y, z});
    //         }
    //     }
    // }

    void extract_ouster_coordinates(const sensor_msgs::msg::PointCloud2 &msg,
                                std::vector<std::array<float, 3>> &points) {
        const size_t max_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
        points.clear();
        points.reserve(max_points);
        for (sensor_msgs::PointCloud2ConstIterator<float> x_pointcloud(msg, "x"),
                                                    y_pointcloud(msg, "y"),
                                                    z_pointcloud(msg, "z");
            x_pointcloud != x_pointcloud.end(); ++x_pointcloud, ++y_pointcloud, ++z_pointcloud) {
            const float x = *x_pointcloud;
            const float y = *y_pointcloud;
            const float z = *z_pointcloud;
            if (!isfinite(x) || !isfinite(y)) continue;
            points.push_back({x, y, z});
        }
    }
    /**
     * Collects every camera detection that is allowed for LiDAR association (exluding the one blocked by the drill structure).
     * Detections are sorted by increasing range so closer visible tags are processed first.
     */
    void select_camera_detections_for_filter(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        const int64_t now_ns = this->now().nanoseconds();
        size_t n = std::min({msg->ar_angles_list.size(),
                             msg->poses.size(),
                             msg->marker_ids.size(),
                             msg->landmark_map_pos_x.size(),
                             msg->landmark_map_pos_y.size()});

        struct RangeAngleId {
            double range;
            double ang_deg;
            int64_t marker_id;
            double map_x;
            double map_y;
            geometry_msgs::msg::Point camera_position;
        };
        vector<RangeAngleId> for_filter;
        for_filter.reserve(n);

        struct AllowedRow {
            double range;
            size_t idx;
        };
        vector<AllowedRow> allowed_rows;
        allowed_rows.reserve(n);

        /* One batch per camera message so pose_estimator keeps every forbidden-sector
         * tag (last_cube_phi_ is overwritten per callback, not per row). */
        ros2_aruco_interfaces::msg::ArucoMarkers cube_phi_batch;
        cube_phi_batch.header = msg->header;
        visualization_msgs::msg::MarkerArray centre_markers_batch;

        for (size_t i = 0; i < n; ++i) {
            const double x = msg->poses[i].position.x;
            const double y = msg->poses[i].position.y;

            const double r = sqrt(x * x + y * y);
            const double a = wrap180(msg->ar_angles_list[i]);

            // Behind drill
            const bool in_forbidden_sector = (msg->ar_angles_list[i] < 160.0 && msg->ar_angles_list[i] > 110.0);

            if (!in_forbidden_sector) {
                for_filter.push_back({
                    r,
                    a,
                    msg->marker_ids[i],
                    msg->landmark_map_pos_x[i],
                    msg->landmark_map_pos_y[i],
                    msg->poses[i].position
                });
                allowed_rows.push_back({r, i});
            }
            if (in_forbidden_sector) {
                geometry_msgs::msg::Point centre_moyen;
                centre_moyen = msg->poses[i].position;

                std::string target_frame = "base_link";
                try {
                    if (tf_buffer_.canTransform(target_frame, 
                                               msg->header.frame_id,
                                               msg->header.stamp,
                                               tf2::durationFromSec(0.1))) {
                        auto transform = tf_buffer_.lookupTransform(
                            target_frame,
                            msg->header.frame_id,
                            msg->header.stamp
                        );
                        
                        geometry_msgs::msg::PointStamped point_in, point_out;
                        point_in.header = msg->header;
                        point_in.point = centre_moyen;
                        tf2::doTransform(point_in, point_out, transform);
                        centre_moyen = point_out.point;
                    } else {
                        // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        //                    "Transform not available from %s to %s",
                        //                    msg->header.frame_id.c_str(), target_frame.c_str());
                        target_frame = msg->header.frame_id;
                    }
                } catch (const tf2::TransformException &ex) {
                    // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    //                     "Transform error: %s", ex.what());
                    (void)ex;
                    target_frame = msg->header.frame_id;
                }

                cube_phi_batch.ar_angles_list.push_back(msg->ar_angles_list[i]);
                geometry_msgs::msg::Pose cube_pose;
                cube_pose.position = centre_moyen;
                cube_pose.orientation = msg->poses[i].orientation;
                cube_phi_batch.poses.push_back(cube_pose);
                cube_phi_batch.marker_ids.push_back(msg->marker_ids[i]);
                cube_phi_batch.landmark_map_pos_x.push_back(msg->landmark_map_pos_x[i]);
                cube_phi_batch.landmark_map_pos_y.push_back(msg->landmark_map_pos_y[i]);

                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->now();
                marker.header.frame_id = target_frame;
                marker.ns = "cube_centres";
                marker.id = static_cast<int>(msg->marker_ids[i]);
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position = centre_moyen;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                centre_markers_batch.markers.push_back(marker);
            }
        }

        if (!cube_phi_batch.marker_ids.empty()) {
            cube_phi_batch.header.stamp = msg->header.stamp;
            cube_phi_batch.header.frame_id = "base_link";
            cube_markers_pub_->publish(cube_phi_batch);
        }
        if (!centre_markers_batch.markers.empty()) {
            centre_pub_->publish(centre_markers_batch);
        }

        sort(allowed_rows.begin(), allowed_rows.end(),
             [](const AllowedRow &u, const AllowedRow &v) { return u.range < v.range; });
        {
            ros2_aruco_interfaces::msg::ArucoMarkers lidar_assoc_msg;
            lidar_assoc_msg.header = msg->header;
            for (const auto &ar : allowed_rows) {
                const size_t i = ar.idx;
                lidar_assoc_msg.marker_ids.push_back(msg->marker_ids[i]);
                lidar_assoc_msg.poses.push_back(msg->poses[i]);
                lidar_assoc_msg.landmark_map_pos_x.push_back(msg->landmark_map_pos_x[i]);
                lidar_assoc_msg.landmark_map_pos_y.push_back(msg->landmark_map_pos_y[i]);
                lidar_assoc_msg.ar_angles_list.push_back(msg->ar_angles_list[i]);
            }
            aruco_markers_lidar_assoc_pub_->publish(lidar_assoc_msg);
        }

        sort(for_filter.begin(), for_filter.end(),
             [](const RangeAngleId &u, const RangeAngleId &v) { return u.range < v.range; });

        {
            lock_guard<mutex> lk(angles_mutex_);
            selected_ranges_.clear();
            selected_angles_aruco_deg_.clear();
            selected_marker_ids_.clear();
            selected_landmark_map_positions_.clear();
            selected_camera_positions_.clear();
            selected_ranges_.reserve(for_filter.size());
            selected_angles_aruco_deg_.reserve(for_filter.size());
            selected_marker_ids_.reserve(for_filter.size());
            selected_landmark_map_positions_.reserve(for_filter.size());
            selected_camera_positions_.reserve(for_filter.size());
            for (const auto &e : for_filter) {
                selected_ranges_.push_back(e.range);
                selected_angles_aruco_deg_.push_back(e.ang_deg);
                selected_marker_ids_.push_back(e.marker_id);
                selected_landmark_map_positions_.push_back({e.map_x, e.map_y});
                selected_camera_positions_.push_back(e.camera_position);
            }
            selected_camera_frame_ = msg->header.frame_id;
            selected_camera_stamp_ = msg->header.stamp;
            selected_count_ = selected_ranges_.size();
            last_camera_update_ns_ = now_ns;
        }
    }

    void filter_pcl_with_known_aruco_poses(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        // This function transforms known ArUco map poses into the point cloud frame and filters
        // keeping only points that are within tolerace_radius_ meters of the supposed ArUco poses.
        // It filters by map poses among landmarks that pass the camera association gate.
        // theoretically we do not need the camera detections of aruco tags for this,
        // but we use every camera detection that associates to a known landmark within a gate distance because:
        // - the camera detecting the tag means we have lign of sight to it so it will be visible by the pointcloud
        //     (unless its hidden by the drill structure TODO: check if aruco tag is hidden)
        // - selecting the closest ones means we are more likely to be able to detected the cubes using RANSAC later on
        //   because there will be more points on the cubes.

        // Rate limit
        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns - last_cloud_time_ns_ < min_cloud_period_ns_) return;
        last_cloud_time_ns_ = now_ns;
        const bool use_camera_mode = use_camera_aruco_position_.load();

        // get the detected aruco angles, ranges and marker IDs in the camera frame.
        std::vector<double> cam_angles, cam_ranges;
        std::vector<int64_t> cam_marker_ids;
        std::vector<std::pair<double, double>> cam_map_positions;
        std::vector<geometry_msgs::msg::Point> cam_camera_positions;
        std::string camera_frame;
        builtin_interfaces::msg::Time camera_stamp;
        int64_t last_camera_update_ns = 0;
        {
            std::lock_guard<std::mutex> lk(angles_mutex_);
            cam_angles = selected_angles_aruco_deg_;
            cam_ranges = selected_ranges_;
            cam_marker_ids = selected_marker_ids_;
            cam_map_positions = selected_landmark_map_positions_;
            cam_camera_positions = selected_camera_positions_;
            camera_frame = selected_camera_frame_;
            camera_stamp = selected_camera_stamp_;
            last_camera_update_ns = last_camera_update_ns_;
        }
        // if there are no camera detections, we dont even attempt to try to find the aruco boxes
        // For now this is done to ensure maximum reliability of the detection because if the camera
        // sees it then it means we should have LoS and it should be visible in the pcl.
        // if the detection code is robust enough then one could just remove this dependance on the cameras.
        if (cam_marker_ids.empty()) {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            //     "[phi_filter] EXIT: cam_marker_ids empty (no camera detections stored)");
            return;
        }

        if (last_camera_update_ns <= 0 || now_ns - last_camera_update_ns > camera_detection_ttl_ns_) {
            {
                std::lock_guard<std::mutex> lk(angles_mutex_);
                selected_angles_aruco_deg_.clear();
                selected_ranges_.clear();
                selected_marker_ids_.clear();
                selected_landmark_map_positions_.clear();
                selected_camera_positions_.clear();
                selected_camera_frame_.clear();
                selected_camera_stamp_ = builtin_interfaces::msg::Time();
                selected_count_ = 0;
            }
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            //     "[phi_filter] EXIT: stale camera detections (age %.3fs > ttl %.3fs)",
            //     (now_ns - last_camera_update_ns) / 1e9,
            //     camera_detection_ttl_ns_ / 1e9);
            return;
        }

        // Direct detection-based association: each camera detection carries both the marker ID
        // and that marker's known map position from multiview_aruco_node. Keep the ID to
        // reject duplicates, but use the per-detection map position so this node cannot drop
        // extra detections because of a stale/mismatched local landmark table.
        static constexpr double OUT_OF_BOUNDS_COORD = 999999.0;

        std::vector<std::pair<double, double>> selected_map_positions;
        std::vector<geometry_msgs::msg::Point> selected_camera_positions;
        std::vector<int64_t> selected_ids;
        std::vector<double> selected_angles;
        std::vector<double> selected_ranges;
        selected_map_positions.reserve(cam_marker_ids.size());
        selected_camera_positions.reserve(cam_marker_ids.size());
        selected_ids.reserve(cam_marker_ids.size());
        selected_angles.reserve(cam_marker_ids.size());
        selected_ranges.reserve(cam_marker_ids.size());

        for (size_t i = 0; i < cam_marker_ids.size(); ++i) {
            const int64_t id = cam_marker_ids[i];
            if ((!use_camera_mode && i >= cam_map_positions.size()) ||
                (use_camera_mode && i >= cam_camera_positions.size())) {
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                //     "[phi_filter] cam[%zu] marker_id=%ld has no map position, skipping",
                //     i, static_cast<long>(id));
                continue;
            }
            if (std::find(selected_ids.begin(), selected_ids.end(), id) != selected_ids.end()) {
                continue; // duplicate detection for the same marker
            }

            if (use_camera_mode) {
                const auto &position = cam_camera_positions[i];
                if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                    !std::isfinite(position.z)) {
                    continue;
                }
                selected_camera_positions.push_back(position);
            } else {
                const double map_x = cam_map_positions[i].first;
                const double map_y = cam_map_positions[i].second;
                if (!std::isfinite(map_x) || !std::isfinite(map_y) ||
                    map_x == OUT_OF_BOUNDS_COORD || map_y == OUT_OF_BOUNDS_COORD) {
                    // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                    //     "[phi_filter] cam[%zu] marker_id=%ld map position invalid, skipping",
                    //     i, static_cast<long>(id));
                    continue;
                }
                selected_map_positions.push_back({map_x, map_y});
            }

            selected_ids.push_back(id);
            selected_angles.push_back(cam_angles[i]);
            selected_ranges.push_back(cam_ranges[i]);
        }

        // Transform selected landmarks directly from map into the point cloud frame
        // so the spatial filter operates in the exact coordinate system of the LiDAR points.
        const std::string &cloud_frame = in->header.frame_id;
        if (cloud_frame.empty()) {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            //     "[phi_filter] EXIT: input cloud has empty frame_id");
            return;
        }
        std::vector<std::pair<double, double>> selected_landmarks;
        if (use_camera_mode) {
            selected_landmarks = get_camera_positions_in_frame(
                selected_camera_positions, camera_frame, cloud_frame, camera_stamp);
        } else {
            selected_landmarks = get_aruco_poses_in_frame(
                selected_map_positions, cloud_frame, in->header.stamp);
        }

        std::vector<std::pair<double, double>> valid_selected_landmarks;
        std::vector<int64_t> valid_selected_ids;
        valid_selected_landmarks.reserve(selected_landmarks.size());
        valid_selected_ids.reserve(selected_landmarks.size());
        const size_t transformed_count = std::min(
            selected_landmarks.size(), selected_ids.size());
        for (size_t i = 0; i < transformed_count; ++i) {
            const double lx = selected_landmarks[i].first;
            const double ly = selected_landmarks[i].second;
            if (!std::isfinite(lx) || !std::isfinite(ly) ||
                lx == OUT_OF_BOUNDS_COORD || ly == OUT_OF_BOUNDS_COORD) {
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                //     "[phi_filter] cam id=%ld map=(%.2f,%.2f) transformed invalid, skipping",
                //     static_cast<long>(selected_ids[i]),
                //     selected_map_positions[i].first, selected_map_positions[i].second);
                continue;
            }
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            //     "[phi_filter] cam id=%ld ang=%.1f° range=%.2fm map=(%.2f,%.2f) -> %s=(%.2f,%.2f)",
            //     static_cast<long>(selected_ids[i]), selected_angles[i], selected_ranges[i],
            //     selected_map_positions[i].first, selected_map_positions[i].second,
            //     cloud_frame.c_str(), lx, ly);
            valid_selected_landmarks.push_back({lx, ly});
            valid_selected_ids.push_back(selected_ids[i]);
        }
        selected_landmarks = std::move(valid_selected_landmarks);
        selected_ids = std::move(valid_selected_ids);

        if (selected_landmarks.empty()) {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            //     "[phi_filter] EXIT: no valid camera landmark map positions "
            //     "(cam_marker_ids=%zu)",
            //     cam_marker_ids.size());
            return;
        }

        if (use_camera_mode) {
            publish_camera_search_cones(
                selected_landmarks, selected_ids, cloud_frame, in->header.stamp);
        }

        // Debug: print landmark positions in cloud frame and tolerance
        // std::string lm_str;
        // for (size_t i = 0; i < selected_landmarks.size(); ++i) {
        //     char buf[64];
        //     snprintf(buf, sizeof(buf), "(%.2f,%.2f) ", selected_landmarks[i].first, selected_landmarks[i].second);
        //     lm_str += buf;
        // }
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
        //     "[phi_filter] landmarks in '%s' frame: %s| tol=%.2fm",
        //     cloud_frame.c_str(), lm_str.c_str(), tolerance_radius_);
        // RCLCPP_INFO(this->get_logger(), "lidar frame Landmarks in %s", lm_str.c_str());

        // bounding box filter over all selected landmark neighborhoods (now in cloud frame)
        const double bounding_box_tol = tolerance_radius_;
        const double bounding_box_tol_squared = bounding_box_tol * bounding_box_tol;

        double xmin = std::numeric_limits<double>::infinity();
        double xmax = -std::numeric_limits<double>::infinity();
        double ymin = std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();

        for(const auto &landmark : selected_landmarks) {
            if (use_camera_mode) {
                const double range = std::hypot(landmark.first, landmark.second);
                const double depth_tolerance = std::max(
                    camera_cone_depth_tolerance_m_,
                    camera_cone_depth_tolerance_ratio_ * range);
                const double lateral_tolerance =
                    (range + depth_tolerance) *
                    std::tan(camera_cone_half_angle_deg_ * M_PI / 180.0);
                // Conservative coarse box; the exact cone test is below.
                const double coarse_tolerance = depth_tolerance + lateral_tolerance;
                xmin = std::min(xmin, landmark.first - coarse_tolerance);
                xmax = std::max(xmax, landmark.first + coarse_tolerance);
                ymin = std::min(ymin, landmark.second - coarse_tolerance);
                ymax = std::max(ymax, landmark.second + coarse_tolerance);
            } else {
                xmin = std::min(xmin, landmark.first - bounding_box_tol);
                xmax = std::max(xmax, landmark.first + bounding_box_tol);
                ymin = std::min(ymin, landmark.second - bounding_box_tol);
                ymax = std::max(ymax, landmark.second + bounding_box_tol);
            }
        }

        extract_ouster_coordinates(*in, points_buf_);
        if (points_buf_.empty()) return;

        const std::string height_frame = use_camera_mode ? "base_link" : "map";
        geometry_msgs::msg::TransformStamped cloud_to_height_tf;
        if (!lookup_cloud_to_height_frame_transform(
                cloud_frame, height_frame, in->header.stamp, cloud_to_height_tf)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *get_clock(), 3000,
                "[phi_filter] EXIT: transform from cloud frame '%s' to height frame '%s' "
                "unavailable; cannot apply z filter",
                cloud_frame.c_str(), height_frame.c_str());
            return;
        }
        tf2::Quaternion cloud_to_height_q;
        tf2::fromMsg(cloud_to_height_tf.transform.rotation, cloud_to_height_q);
        const tf2::Matrix3x3 cloud_to_height_rot(cloud_to_height_q);
        const double cloud_to_height_z = cloud_to_height_tf.transform.translation.z;

        sensor_msgs::msg::PointCloud2 out;
        out.header         = in->header;
        out.height         = 1;
        out.is_bigendian   = false;
        out.is_dense       = false;
        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "marker_id", 1, sensor_msgs::msg::PointField::INT32);
        mod.resize(points_buf_.size());

        sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
        sensor_msgs::PointCloud2Iterator<int32_t> oid(out, "marker_id");
        size_t kept = 0;
        size_t rej_z = 0;       // rejected by z filter
        size_t rej_bbox = 0;    // rejected by bounding box filter
        size_t rej_radius = 0;  // inside bbox but outside tolerance_radius of every landmark
        size_t rej_ambiguous = 0;

        for (const auto &p : points_buf_) {
            const double height_z =
                cloud_to_height_z +
                cloud_to_height_rot[2][0] * static_cast<double>(p[0]) +
                cloud_to_height_rot[2][1] * static_cast<double>(p[1]) +
                cloud_to_height_rot[2][2] * static_cast<double>(p[2]);
            if (height_z < hauteur_z_min || height_z > hauteur_z_max) { ++rej_z; continue; }

            const double px = p[0];
            const double py = p[1];

            if (px < xmin || px > xmax || py < ymin || py > ymax) { ++rej_bbox; continue; }

            size_t nearest_index = selected_landmarks.size();
            double nearest_squared_dist = std::numeric_limits<double>::infinity();
            double second_squared_dist = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < selected_landmarks.size(); ++i) {
                double squared_dist = std::numeric_limits<double>::infinity();
                if (use_camera_mode) {
                    const double point_range = std::hypot(px, py);
                    const double expected_range = std::hypot(
                        selected_landmarks[i].first, selected_landmarks[i].second);
                    if (!(point_range > 0.0) || !(expected_range > 0.0)) continue;

                    const double depth_tolerance = std::max(
                        camera_cone_depth_tolerance_m_,
                        camera_cone_depth_tolerance_ratio_ * expected_range);
                    const double radial_error = std::fabs(point_range - expected_range);
                    const double point_angle = std::atan2(py, px) * 180.0 / M_PI;
                    const double expected_angle = std::atan2(
                        selected_landmarks[i].second,
                        selected_landmarks[i].first) * 180.0 / M_PI;
                    const double angular_error = angDeltaDeg(point_angle, expected_angle);
                    if (radial_error > depth_tolerance ||
                        angular_error > camera_cone_half_angle_deg_) {
                        continue;
                    }

                    const double lateral_error = point_range *
                        std::sin(angular_error * M_PI / 180.0);
                    squared_dist = radial_error * radial_error +
                        lateral_error * lateral_error;
                } else {
                    const double dx = px - selected_landmarks[i].first;
                    const double dy = py - selected_landmarks[i].second;
                    squared_dist = dx * dx + dy * dy;
                }
                if (squared_dist < nearest_squared_dist) {
                    second_squared_dist = nearest_squared_dist;
                    nearest_squared_dist = squared_dist;
                    nearest_index = i;
                } else if (squared_dist < second_squared_dist) {
                    second_squared_dist = squared_dist;
                }
            }

            if (nearest_index >= selected_landmarks.size() ||
                (!use_camera_mode && nearest_squared_dist > bounding_box_tol_squared)) {
                ++rej_radius;
                continue;
            }
            if (std::isfinite(second_squared_dist) &&
                std::sqrt(second_squared_dist) - std::sqrt(nearest_squared_dist) <
                    association_ambiguity_margin_m_) {
                ++rej_ambiguous;
                continue;
            }

            *ox = px;  ++ox;
            *oy = py;  ++oy;
            *oz = p[2]; ++oz;
            *oid = static_cast<int32_t>(selected_ids[nearest_index]); ++oid;
            ++kept;
        }

        // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
        //     "[phi_filter] stats: in=%zu kept=%zu rej_z=%zu rej_bbox=%zu rej_radius=%zu "
        //     "| z_seen=[%.2f..%.2f] z_filt=[%.2f..%.2f] | bbox=[%.2f..%.2f, %.2f..%.2f] frame='%s'",
        //     points_buf_.size(), kept, rej_z, rej_bbox, rej_radius,
        //     z_seen_min, z_seen_max, hauteur_z_min, hauteur_z_max,
        //     xmin, xmax, ymin, ymax, in->header.frame_id.c_str());

        mod.resize(kept);
        out.width = static_cast<uint32_t>(kept);

        // Recovery-only diagnostic output. It deliberately includes an empty
        // cloud when the camera crop contains no usable LiDAR returns, making
        // a failed crop visible to rosbag/RViz instead of leaving stale data.
        if (use_camera_mode) {
            camera_recovery_cloud_pub_->publish(out);
        }
        if (kept == 0) return;

        cloud_pub_->publish(out);
    }


private:
    double tolerance_deg_;
    double tolerance_radius_;
    double hauteur_z_min;
    double hauteur_z_max;
    std::atomic_bool use_camera_aruco_position_{false};
    double camera_cone_half_angle_deg_{5.0};
    double camera_cone_depth_tolerance_m_{1.0};
    double camera_cone_depth_tolerance_ratio_{0.20};
    double association_ambiguity_margin_m_;
    int64_t min_cloud_period_ns_;
    int64_t last_cloud_time_ns_;
    int64_t camera_detection_ttl_ns_;
    int64_t last_camera_update_ns_;
    std::vector<std::array<float, 3>> points_buf_;
    std::vector<double> flat;

    string input_cloud_topic_;
    string output_cloud_topic_;
    string aruco_topic_;
    // subs/pubs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_recovery_cloud_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_lidar_assoc_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_circles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_search_zones_pub_;
    rclcpp::TimerBase::SharedPtr landmark_timer_;
    // état angles
    mutex angles_mutex_;
    // Toutes les détections caméra utilisables pour l’association LiDAR–landmark (triées par portée croissante).
    size_t selected_count_;
    vector<double> selected_angles_aruco_deg_;
    vector<double> selected_ranges_;
    // marker IDs aligned 1:1 with selected_ranges_ / selected_angles_aruco_deg_,
    // used for direct ID-based association to known landmarks.
    vector<int64_t> selected_marker_ids_;
    vector<std::pair<double, double>> selected_landmark_map_positions_;
    vector<geometry_msgs::msg::Point> selected_camera_positions_;
    string selected_camera_frame_;
    builtin_interfaces::msg::Time selected_camera_stamp_;
    // transformation 
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<LidarPhiFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
