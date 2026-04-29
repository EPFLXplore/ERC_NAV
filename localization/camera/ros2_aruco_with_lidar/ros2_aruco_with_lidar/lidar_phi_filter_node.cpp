#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <mutex>
#include <string>
#include <list>
#include <array>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
        this->declare_parameter<double>("hauteur_z_min", -0.5);
        this->declare_parameter<double>("hauteur_z_max", 0.5);

        this->get_parameter("tolerance_deg", tolerance_deg_);
        this->get_parameter("tolerance_radius", tolerance_radius_);
        this->get_parameter("hauteur_z_min", hauteur_z_min);
        this->get_parameter("hauteur_z_max", hauteur_z_max);
        input_cloud_topic_ = "/ouster/points";
        output_cloud_topic_ = "/ouster_points_aruco";
        aruco_topic_ = "aruco_markers";
        selected_count_ = 0;
        min_cloud_period_ns_ = static_cast<int64_t>(1e9 / 5.0); // 5 Hz max 
        last_cloud_time_ns_ = 0;

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

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
    centre_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("centre_cube_beleck", qos);
    cube_markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/cube_markers", qos);

    landmark_circles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "aruco_cube_circles", qos);
    landmark_timer_ = create_wall_timer(std::chrono::milliseconds(2000), [this]() {
        publish_landmark_circles();
    });
    }

private:
    const vector<std::pair<double, double>> landmark_poses_ = {
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


    vector<std::pair<double, double>> get_aruco_poses_in_base_link(const vector<std::pair<double, double>> &aruco_map_positions) {
        // in: list of (x, y) map positions of the center of aruco boxes. Given by the ERC.
        // out: list of (x, y) positions of the center of aruco boxes in base_link. Those will be used to select points in the pointcloud
        // around those positions to make the detection more efficient and robust.

        // check if the transform map->base_link is available
        string target_frame = "base_link";
        geometry_msgs::msg::TransformStamped transform;
        try {
            if (tf_buffer_.canTransform(target_frame, "map", tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                // gets the transform from map to base_link at the latest available time (TimePointZero) with a timeout of 0.1s
                transform = tf_buffer_.lookupTransform(target_frame, "map", tf2::TimePointZero);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Transform not available from map to base_link");
                return {};
            }        
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "Transform error: %s", ex.what());
            return {};
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
            m.scale.x = 0.6;  // diameter = 2 * 1m radius
            m.scale.y = 0.6;
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

    // Extraction des coordonnées XYZ
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
     * Collects every camera detection that is allowed for LiDAR association (exluding the one blocked by the drill structure),
     * Detections are sorted by increasing range so downstream greedy landmark association still prefers closer tags first.
     */
    void select_camera_detections_for_filter(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        size_t n = msg->ar_angles_list.size();
        if (msg->poses.size() < n) {
            n = msg->poses.size();
        }

        struct RangeAngle {
            double range;
            double ang_deg;
        };
        vector<RangeAngle> for_filter;
        for_filter.reserve(n);

        for (size_t i = 0; i < n; ++i) {
            const double x = msg->poses[i].position.x;
            const double y = msg->poses[i].position.y;

            const double r = sqrt(x * x + y * y);
            const double a = wrap180(msg->ar_angles_list[i]);

            const bool in_forbidden_sector =
                (msg->ar_angles_list[i] < 180.0 && msg->ar_angles_list[i] > 90.0);

            if (!in_forbidden_sector) {
                for_filter.push_back({r, a});
            }
            if (in_forbidden_sector) {
                ros2_aruco_interfaces::msg::ArucoMarkers cube_msg;
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
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                                           "Transform not available from %s to %s",
                                           msg->header.frame_id.c_str(), target_frame.c_str());
                        target_frame = msg->header.frame_id;
                    }
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                                        "Transform error: %s", ex.what());
                    target_frame = msg->header.frame_id;
                }
                
                cube_msg.header = msg->header;
                cube_msg.header.frame_id = target_frame;
                cube_msg.ar_angles_list.push_back(msg->ar_angles_list[i]);
                geometry_msgs::msg::Pose cube_pose;
                cube_pose.position = centre_moyen;
                cube_pose.orientation = msg->poses[i].orientation;
                cube_msg.poses.push_back(cube_pose);
                cube_msg.marker_ids.push_back(msg->marker_ids[i]);
                cube_markers_pub_->publish(cube_msg);

                // Publish a MarkerArray with one marker at the detected ArUco pose
                visualization_msgs::msg::MarkerArray markers;
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->now();
                marker.header.frame_id = target_frame;
                marker.ns = "cube_centres";
                marker.id = static_cast<int>(i);
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
                markers.markers.push_back(marker);
                centre_pub_->publish(markers);
            }
        }

        sort(for_filter.begin(), for_filter.end(),
             [](const RangeAngle &u, const RangeAngle &v) { return u.range < v.range; });

        {
            lock_guard<mutex> lk(angles_mutex_);
            selected_ranges_.clear();
            selected_angles_aruco_deg_.clear();
            selected_ranges_.reserve(for_filter.size());
            selected_angles_aruco_deg_.reserve(for_filter.size());
            for (const auto &e : for_filter) {
                selected_ranges_.push_back(e.range);
                selected_angles_aruco_deg_.push_back(e.ang_deg);
            }
            selected_count_ = selected_ranges_.size();
        }
    }

    void filter_pcl_with_known_aruco_poses(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        // This functions uses the known ArUco poses in the base_link frame to filter the point cloud
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

        const auto landmarks_in_base_link = get_aruco_poses_in_base_link(landmark_poses_);
        if (landmarks_in_base_link.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                "[phi_filter] EXIT: landmarks_in_base_link empty (TF failed)");
            return;
        }

        // get the detected aruco angles and ranges in the camera frame.
        std::vector<double> cam_angles, cam_ranges;
        {
            std::lock_guard<std::mutex> lk(angles_mutex_);
            cam_angles = selected_angles_aruco_deg_;
            cam_ranges = selected_ranges_;
        }
        // if there are no camera detections, we dont even attempt to try to find the aruco boxes
        // For now this is done to ensure maximum reliability of the detection because if the camera
        // sees it then it means we should have LoS and it should be visible in the pcl.
        // if the detection code is robust enough then one could just remove this dependance on the cameras.
        if (cam_angles.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                "[phi_filter] EXIT: cam_angles empty (no camera detections stored)");
            return;
        }

        //associate each camera deteciton with the closest known landmark in base_link.
        static constexpr double ANGLE_OFFSET_DEG   = 0.0; // camera bearings are already in base_link-compatible frame (x-forward)
        static constexpr double DEG2RAD            = M_PI / 180.0;
        static constexpr double OUT_OF_BOUNDS_COORD = 999999.0;
        const double ASSOCIATION_GATE_DIST = 1.3; // in meters, max distance to associate a camera detection to a landmark

        std::vector<std::pair<double, double>> selected_landmarks;
        // avoid associating two camera detections to the same landmark
        // by keeping track of which landmarks have already been associated to a camera detection.
        selected_landmarks.reserve(cam_angles.size());
        std::vector<char> landmark_used(landmarks_in_base_link.size(), 0);

        // WARNING: we currently assume that the camera origin is the same as the base_link origin, which isn't exactly true
        // but the error is comprised in the tolerance_radius_ param

        //TODO : use URDF Transform Lookup from camera frame to base_link frame to convert the camera detections to base_link frame.
        for(size_t i=0; i<cam_angles.size(); ++i) {
            const double theta = (cam_angles[i] + ANGLE_OFFSET_DEG) * DEG2RAD;

            // convert to cartesian coords in cam frame
            const double cx = cam_ranges[i] * std::cos(theta);
            const double cy = cam_ranges[i] * std::sin(theta);

            // nearest neighbor search among the known landmarks in base_link
            double best_squared_dist = std::numeric_limits<double>::infinity();
            size_t best_idx = landmarks_in_base_link.size();


            for(size_t j=0; j<landmarks_in_base_link.size(); ++j) {
                if (landmark_used[j]) continue; // already associated to another cam detection
                const double lx = landmarks_in_base_link[j].first;
                const double ly = landmarks_in_base_link[j].second;
                if (lx == OUT_OF_BOUNDS_COORD || ly == OUT_OF_BOUNDS_COORD) continue; // invalid landmark position
                const double squared_dist = (cx - lx) * (cx - lx) + (cy - ly) * (cy - ly);
                if (squared_dist < best_squared_dist) {
                    best_squared_dist = squared_dist;
                    best_idx = j;
                }
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                "[phi_filter] cam[%zu] ang=%.1f° range=%.2fm -> cart(%.2f,%.2f) | "
                "nearest lm[%zu]=(%.2f,%.2f) dist=%.2fm gate=%.1fm",
                i, cam_angles[i], cam_ranges[i], cx, cy,
                best_idx,
                best_idx < landmarks_in_base_link.size() ? landmarks_in_base_link[best_idx].first : 0.0,
                best_idx < landmarks_in_base_link.size() ? landmarks_in_base_link[best_idx].second : 0.0,
                std::sqrt(best_squared_dist), ASSOCIATION_GATE_DIST);

            if (best_idx < landmarks_in_base_link.size() && best_squared_dist <= ASSOCIATION_GATE_DIST * ASSOCIATION_GATE_DIST) {
                selected_landmarks.push_back(landmarks_in_base_link[best_idx]);
                landmark_used[best_idx] = 1;
            }
        }
        if (selected_landmarks.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                "[phi_filter] EXIT: no cam-landmark associations passed %.1fm gate "
                "(cam_angles=%zu, landmarks=%zu)",
                ASSOCIATION_GATE_DIST, cam_angles.size(), landmarks_in_base_link.size());
            return;
        }

        // Transform selected landmarks from base_link into the point cloud frame
        // so the spatial filter operates in the same coordinate system as the lidar points.
        const std::string &cloud_frame = in->header.frame_id;
        if (cloud_frame != "base_link") {
            try {
                auto tf_bl_to_cloud = tf_buffer_.lookupTransform(
                    cloud_frame, "base_link", tf2::TimePointZero);
                geometry_msgs::msg::PointStamped pt_in, pt_out;
                pt_in.header.frame_id = "base_link";
                pt_in.point.z = 0.0;
                for (auto &lm : selected_landmarks) {
                    pt_in.point.x = lm.first;
                    pt_in.point.y = lm.second;
                    tf2::doTransform(pt_in, pt_out, tf_bl_to_cloud);
                    lm.first  = pt_out.point.x;
                    lm.second = pt_out.point.y;
                }
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                    "[phi_filter] Cannot transform landmarks to cloud frame '%s': %s",
                    cloud_frame.c_str(), ex.what());
                return;
            }
        }

        // Debug: print landmark positions in cloud frame and tolerance
        {
            std::string lm_str;
            for (size_t i = 0; i < selected_landmarks.size(); ++i) {
                char buf[64];
                snprintf(buf, sizeof(buf), "(%.2f,%.2f) ", selected_landmarks[i].first, selected_landmarks[i].second);
                lm_str += buf;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
                "[phi_filter] landmarks in '%s' frame: %s| tol=%.2fm",
                cloud_frame.c_str(), lm_str.c_str(), tolerance_radius_);
        }

        // bounding box filter over all selected landmark neighborhoods (now in cloud frame)
        const double bounding_box_tol = tolerance_radius_;
        const double bounding_box_tol_squared = bounding_box_tol * bounding_box_tol;

        double xmin = std::numeric_limits<double>::infinity();
        double xmax = -std::numeric_limits<double>::infinity();
        double ymin = std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();

        for(const auto &landmark : selected_landmarks) {
            xmin = std::min(xmin, landmark.first - bounding_box_tol);
            xmax = std::max(xmax, landmark.first + bounding_box_tol);
            ymin = std::min(ymin, landmark.second - bounding_box_tol);
            ymax = std::max(ymax, landmark.second + bounding_box_tol);
        }

        extract_ouster_coordinates(*in, points_buf_);
        if (points_buf_.empty()) return;

        sensor_msgs::msg::PointCloud2 out;
        out.header         = in->header;
        out.height         = 1;
        out.is_bigendian   = false;
        out.is_dense       = false;
        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(points_buf_.size());

        sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
        size_t kept = 0;
        size_t rej_z = 0;       // rejected by z filter
        size_t rej_bbox = 0;    // rejected by bounding box filter
        size_t rej_radius = 0;  // inside bbox but outside tolerance_radius of every landmark
        float z_seen_min =  std::numeric_limits<float>::infinity();
        float z_seen_max = -std::numeric_limits<float>::infinity();

        for (const auto &p : points_buf_) {
            if (p[2] < z_seen_min) z_seen_min = p[2];
            if (p[2] > z_seen_max) z_seen_max = p[2];

            if (p[2] < hauteur_z_min || p[2] > hauteur_z_max) { ++rej_z; continue; }

            const double px = p[0];
            const double py = p[1];

            if (px < xmin || px > xmax || py < ymin || py > ymax) { ++rej_bbox; continue; }

            bool in_radius = false;
            for (const auto &landmark: selected_landmarks) {
                const double dx = px - landmark.first;
                const double dy = py - landmark.second;
                const double squared_dist = dx * dx + dy * dy;
                if (squared_dist <= bounding_box_tol_squared) {
                    *ox = px;  ++ox;
                    *oy = py;  ++oy;
                    *oz = p[2]; ++oz;
                    ++kept;
                    in_radius = true;
                    break;
                }
            }
            if (!in_radius) ++rej_radius;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 3000,
            "[phi_filter] stats: in=%zu kept=%zu rej_z=%zu rej_bbox=%zu rej_radius=%zu "
            "| z_seen=[%.2f..%.2f] z_filt=[%.2f..%.2f] | bbox=[%.2f..%.2f, %.2f..%.2f] frame='%s'",
            points_buf_.size(), kept, rej_z, rej_bbox, rej_radius,
            z_seen_min, z_seen_max, hauteur_z_min, hauteur_z_max,
            xmin, xmax, ymin, ymax, in->header.frame_id.c_str());

        if (kept == 0) {
            return;
        }
        mod.resize(kept);
        out.width = static_cast<uint32_t>(kept);
        cloud_pub_->publish(out);
    }


private:
    double tolerance_deg_;
    double tolerance_radius_;
    double hauteur_z_min;
    double hauteur_z_max;
    int64_t min_cloud_period_ns_;
    int64_t last_cloud_time_ns_;
    std::vector<std::array<float, 3>> points_buf_;

    string input_cloud_topic_;
    string output_cloud_topic_;
    string aruco_topic_;
    // subs/pubs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_circles_pub_;
    rclcpp::TimerBase::SharedPtr landmark_timer_;
    // état angles
    mutex angles_mutex_;
    // Toutes les détections caméra utilisables pour l’association LiDAR–landmark (triées par portée croissante).
    size_t selected_count_;
    vector<double> selected_angles_aruco_deg_;
    vector<double> selected_ranges_;
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


