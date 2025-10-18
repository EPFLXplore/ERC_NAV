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
    LidarPhiFilterNode() : rclcpp::Node("lidar_phi_filter_node") {
        // Valeurs par défaut (sans déclaration de paramètres)
        tolerance_deg_ = 7.0;
        input_cloud_topic_ = "/ouster/points";
        output_cloud_topic_ = "/ouster/points_aruco";
        aruco_topic_ = "aruco_markers";
        selected_count_ = 0;
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

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
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
        if (msg->landmark_map_pos_x.size() < n) n = msg->landmark_map_pos_x.size();
        if (msg->landmark_map_pos_y.size() < n) n = msg->landmark_map_pos_y.size();

        const double INF = 1e10;
        double best_ang[3] = {0.0, 0.0, 0.0};
        double best_rng[3] = {INF, INF, INF};

        for (size_t i = 0; i < n; ++i) {
            const double x = msg->landmark_map_pos_x[i];
            const double y = msg->landmark_map_pos_y[i];
            const double r = sqrt(x * x + y * y);
            const double a = wrap180(msg->ar_angles_list[i]);

            if (r < best_rng[0]) {
                best_rng[2] = best_rng[1]; best_ang[2] = best_ang[1];
                best_rng[1] = best_rng[0]; best_ang[1] = best_ang[0];
                best_rng[0] = r;           best_ang[0] = a;
            } else if (r < best_rng[1]) {
                best_rng[2] = best_rng[1]; best_ang[2] = best_ang[1];
                best_rng[1] = r;           best_ang[1] = a;
            } else if (r < best_rng[2]) {
                best_rng[2] = r;           best_ang[2] = a;
            }
        }

        size_t count = 0;
        if (best_rng[0] < INF) ++count;
        if (best_rng[1] < INF) ++count;
        if (best_rng[2] < INF) ++count;

        selected_count_ = count;
        for (size_t i = 0; i < count && i < 3; ++i) {
            selected_angles_aruco_deg_[i] = best_ang[i];
        }
    }

    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        // copie locale des angles sélectionnés (max 3)
        double angles_local[3];
        size_t k = selected_count_;
        for (size_t i = 0; i < k && i < 3; ++i) angles_local[i] = selected_angles_aruco_deg_[i];
        if (k == 0) {
            return; // pas d'angles -> pas de filtrage
        }

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
            bool match = false;
            for (size_t i = 0; i < k; ++i) {
                if (angDeltaDeg(angle_pointcloud_deg, angles_local[i]) <= tolerance_deg_) { match = true; break; }
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
    string input_cloud_topic_;
    string output_cloud_topic_;
    string aruco_topic_;
    // subs/pubs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    // état angles
    mutex angles_mutex_;
    // angles sélectionnés (max 3)
    size_t selected_count_;
    double selected_angles_aruco_deg_[3];
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<LidarPhiFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


