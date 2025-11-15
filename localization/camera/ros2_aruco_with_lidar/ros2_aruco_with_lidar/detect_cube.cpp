#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <vector>


class DetectCubeNode : public rclcpp::Node {
public:
    DetectCubeNode()
        : rclcpp::Node("detect_cube_node") {
        input_cloud_topic_ = "/ouster/points_aruco";
        distance_threshold_inliers = 0.05; 
        max_iterations_ = 100;
        min_inliers_ = 5;
        max_lines_ = 3;

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                detect_lignes(*message);
            }
        );
    }

private:

    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ou_on_va_supprimer_point_petit(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *pointcloud_ou_on_va_supprimer_point_petit);
        
        struct LineDetection {
            float x0, y0, z0;
            float dx, dy, dz; 
        };
        std::vector<LineDetection> lines;
        lines.reserve(static_cast<size_t>(max_lines_));

        for (int line_idx = 0; line_idx < max_lines_; ++line_idx) {

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(distance_threshold_inliers);
            seg.setMaxIterations(max_iterations_);
            seg.setInputCloud(pointcloud_ou_on_va_supprimer_point_petit);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
                break;
            }

            const float x0 = coefficients->values[0];
            const float y0 = coefficients->values[1];
            const float z0 = coefficients->values[2];
            float dx = coefficients->values[3];
            float dy = coefficients->values[4];
            float dz = coefficients->values[5];
            
            LineDetection det;
            det.x0 = x0; det.y0 = y0; det.z0 = z0;
            det.dx = dx; det.dy = dy; det.dz = dz;
            lines.push_back(det);

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(pointcloud_ou_on_va_supprimer_point_petit);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr remainder(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*remainder);
            pointcloud_ou_on_va_supprimer_point_petit.swap(remainder);
        }

        if (!lines.empty()) {
            for (size_t i = 0; i < lines.size(); ++i) {
                const auto &L = lines[i];
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Ligne %zu | Eq: (x,y,z)=(%.3f,%.3f,%.3f)+t*(%.3f,%.3f,%.3f)",
                    i,
                    L.x0, L.y0, L.z0, L.dx, L.dy, L.dz
                );
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "problemeeeetechnique");

        }

    }

private:
    std::string input_cloud_topic_;
    double distance_threshold_inliers;
    int max_iterations_;
    int min_inliers_;
    int max_lines_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}