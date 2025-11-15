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
        input_cloud_topic_ = "/ouster_points_aruco";
        output_cloud_topic_ = "/cloud_without_lines";
        distance_threshold_inliers = 0.01; 
        max_iterations_ = 300;
        min_inliers_ = 30; // at least 20 points to define a line
        max_lines_ = 5; // at least 2 lines to form a corner
        
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                detect_lignes(*message);
            }
        );
        lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
        RCLCPP_INFO(this->get_logger(), "initialized detect_cube_node");

    }

private:

    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_minus_lines(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *pointcloud_minus_lines);
        sensor_msgs::msg::PointCloud2 cloud_without_lines;
        
        struct LineDetection {
            float x0, y0, z0;
            float dx, dy, dz; 
        };
        std::vector<LineDetection> lines;
        lines.reserve(static_cast<size_t>(max_lines_));
        RCLCPP_INFO(this->get_logger(), "avant le for");


        for (int line_idx = 0; line_idx < max_lines_; ++line_idx) {

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(distance_threshold_inliers);
            seg.setMaxIterations(max_iterations_);
            seg.setInputCloud(pointcloud_minus_lines);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
                RCLCPP_INFO(this->get_logger(), "detect pas suffisamment de points pour une ligne");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "detecte une ligne avec %zu inliers", inliers->indices.size());

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
            extract.setInputCloud(pointcloud_minus_lines);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr remainder(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*remainder);
            pointcloud_minus_lines.swap(remainder);
          

            pcl::ExtractIndices<pcl::PointXYZ> extract_only_lines;
            extract_only_lines.setInputCloud(pointcloud_minus_lines);
            extract_only_lines.setIndices(inliers);
            extract_only_lines.setNegative(false);
            pcl::PointCloud<pcl::PointXYZ>::Ptr detected_lines(new pcl::PointCloud<pcl::PointXYZ>());
            extract_only_lines.filter(*detected_lines);
            toROSMsg(*(detected_lines), cloud_without_lines);
            lines_pub_->publish(cloud_without_lines);


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
    std::string output_cloud_topic_;
    double distance_threshold_inliers;
    int max_iterations_;
    int min_inliers_;
    int max_lines_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lines_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}