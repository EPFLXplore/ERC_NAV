#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
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
        // Valeurs par défaut (sans déclaration de paramètres)
        tolerance_deg_ = 15;
        tolerance_radius_ = 1.5; // 0.2 is the limit 
        hauteur_z_min= -0.7;
        hauteur_z_max= 1;
        input_cloud_topic_ = "/ouster_points";
        output_cloud_topic_ = "/ouster_points_aruco";
        aruco_topic_ = "aruco_markers";
        selected_count_ = 0;

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // subscriber de POINTCLOUD2

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                onCloud(msg);
            });
            // subscriber de ArucoMarkers
        markers_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            aruco_topic_, rclcpp::SensorDataQoS(),
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                detect_3_best_camera(msg);
            });

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
    centre_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("centre_cube_beleck", qos);
    cube_markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/cube_markers", qos);
    }

private:
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
    void detect_3_best_camera(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        size_t n = msg->ar_angles_list.size();
        //if (msg->poses.size() < n) n = msg->poses.size();
        // RCLCPP_INFO(this->get_logger(), "n : %f", n);

        const double INF = 1e10;
        double best_ang[3] = {0.0, 0.0, 0.0};
        double z_aruco[3] = {0.0, 0.0, 0.0};

        double best_rng[3] = {INF, INF, INF};

        for (size_t i = 0; i < n; ++i) {
            const double x = msg->poses[i].position.x;
            const double y = msg->poses[i].position.y;
            const double z = msg->poses[i].position.z;

            //RCLCPP_INFO(this->get_logger(), "x : %f", x);
            //RCLCPP_INFO(this->get_logger(), "y : %f", y);

            const double r = sqrt(x * x + y * y);
            const double a = wrap180(msg->ar_angles_list[i]);

            if (r < best_rng[0] && !(msg->ar_angles_list[i] <180 && msg->ar_angles_list[i] >90)) {
                best_rng[2] = best_rng[1]; best_ang[2] = best_ang[1]; z_aruco[2] = z_aruco[1];
                best_rng[1] = best_rng[0]; best_ang[1] = best_ang[0]; z_aruco[1] = z_aruco[0];
                best_rng[0] = r;           best_ang[0] = a; z_aruco[0] = z;
                // RCLCPP_INFO(this->get_logger(), "premierif : %f", i);

            } else if (r < best_rng[1] && !(msg->ar_angles_list[i] <180 && msg->ar_angles_list[i] >90)) {
                best_rng[2] = best_rng[1]; best_ang[2] = best_ang[1]; z_aruco[2] = z_aruco[1];
                best_rng[1] = r;           best_ang[1] = a; z_aruco[1] = z;
                // RCLCPP_INFO(this->get_logger(), "deuxieme : %f", i);

            } else if (r < best_rng[2] && !(msg->ar_angles_list[i] <180 && msg->ar_angles_list[i] >90)) {
                best_rng[2] = r;           best_ang[2] = a; z_aruco[2] = z;
            }
            if (msg->ar_angles_list[i] <180 && msg->ar_angles_list[i] >90) {
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

        size_t count = 0;
        if (best_rng[0] < INF) ++count;
        if (best_rng[1] < INF) ++count;
        if (best_rng[2] < INF) ++count;
        //RCLCPP_INFO(this->get_logger(), "nbr aruco: %f", count);

        selected_count_ = count;
        for (size_t i = 0; i < count && i < 3; ++i) {
            selected_angles_aruco_deg_[i] = best_ang[i];

            selected_ranges_[i] = best_rng[i];
            // RCLCPP_INFO(this->get_logger(), "nbr aruco: %f", i);
        }
    }

    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        // copie locale des angles sélectionnés (max 3)
        double angles_local[3];
        double ranges_local[3];
        size_t k = selected_count_;
        for (size_t i = 0; i < k && i < 3; ++i) {
            angles_local[i] = selected_angles_aruco_deg_[i];
            ranges_local[i] = selected_ranges_[i];
        }        // if (k == 0) {
        //     return; // pas d'angles -> pas de filtrage
        // }

        // Extraire XYZ dans un tableau simple
        std::vector<std::array<float, 3>> points;
        extract_ouster_coordinates(*in, points);
        if (points.empty()) {
            return;
        }

        // Préparer sortie (taille max = nb points d'entrée, on réduira après)
        sensor_msgs::msg::PointCloud2 out;
        out.header = in->header;
        out.height = 1;
        out.width = static_cast<uint32_t>(points.size());
        out.is_bigendian = false;
        out.is_dense = false;
        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
        sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
        sensor_msgs::PointCloud2Iterator<float> oz(out, "z");

        size_t kept = 0;
        for (const auto &p : points) {
            const double angle_pointcloud_deg = wrap180(atan2(static_cast<double>(p[1]), static_cast<double>(p[0])) * 180.0 / M_PI);
            const double r_point = hypot(static_cast<double>(p[0]), static_cast<double>(p[1]));
            bool match = false;
            for (size_t i = 0; i < k; ++i) {
                // RCLCPP_INFO(this->get_logger(), "position z pt cloud lidar: %.3f", p[2])
                if (angDeltaDeg(angle_pointcloud_deg , angles_local[i]+90) <= tolerance_deg_ &&
                    fabs(r_point - ranges_local[i]) <= tolerance_radius_  && p[2]> hauteur_z_min && p[2]< hauteur_z_max) 
                    { 
                        match = true;
                        // RCLCPP_INFO(this->get_logger(), "Distance lidar - arucotag : %f", fabs(r_point - ranges_local[i])); 
                        //RCLCPP_INFO(this->get_logger(), "Distance lidar à aruco tag R point : %f", fabs(r_point - ranges_local[i]));
                        //RCLCPP_INFO(this->get_logger(), "z filtre: %f", p[2]);

                        break;
                    }
                // if (angDeltaDeg(angle_pointcloud_deg , angles_local[i]+90) <= tolerance_deg_ ) { match = true; break; }
                         
            }
            if (!match) continue;
            

            *ox = p[0]; *oy = p[1]; *oz = p[2];
            ++ox; ++oy; ++oz;
            ++kept;
        }

        if (kept == 0) {
            return; // rien à publier
        }

        mod.resize(kept);
        out.width = static_cast<uint32_t>(kept);
        cloud_pub_->publish(out);
    }

private:
    // paramètres
    double tolerance_deg_;
    double tolerance_radius_;
    double hauteur_z_min;
    double hauteur_z_max;

    string input_cloud_topic_;
    string output_cloud_topic_;
    string aruco_topic_;
    // subs/pubs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    // état angles
    mutex angles_mutex_;
    // angles sélectionnés (max 3)
    size_t selected_count_;
    double selected_angles_aruco_deg_[3];
    double selected_ranges_[3];
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


