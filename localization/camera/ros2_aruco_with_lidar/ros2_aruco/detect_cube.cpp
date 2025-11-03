#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

class PlaneDetectorNode : public rclcpp::Node {
public:
    PlaneDetectorNode() : rclcpp::Node("plane_detector_node") {
        input_topic_ = "/ouster_pointv2";
        max_planes_ = 3;
        distance_threshold_ = 0.02; 
        min_inliers_ = 5000;

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(msg); }
        );

        RCLCPP_INFO(get_logger(), "Plane detector: input='%s' max_planes=%d dist=%.3f min_inliers=%d",
                    input_topic_.c_str(), max_planes_, distance_threshold_, min_inliers_);
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*in, *cloud);
        if (cloud->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr work(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold_);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        int planes_found = 0;

        while (planes_found < max_planes_ && !work->empty()) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            seg.setInputCloud(work);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
                break; // plus de plan significatif
            }

            // calcul centre des inliers
            double cx = 0.0, cy = 0.0, cz = 0.0;
            for (int idx : inliers->indices) {
                const auto &p = work->points[idx];
                cx += p.x; cy += p.y; cz += p.z;
            }
            const double invn = 1.0 / static_cast<double>(inliers->indices.size());
            cx *= invn; cy *= invn; cz *= invn;

            // coeffs plan: ax + by + cz + d = 0
            const double a = coefficients->values[0];
            const double b = coefficients->values[1];
            const double c = coefficients->values[2];
            const double d = coefficients->values[3];

            RCLCPP_INFO(get_logger(), "Plan %d: coeff[a=%.3f b=%.3f c=%.3f d=%.3f], inliers=%zu, centroid=(%.3f, %.3f, %.3f)",
                        planes_found + 1, a, b, c, d, inliers->indices.size(), cx, cy, cz);

            // retirer les inliers et continuer
            extract.setInputCloud(work);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*remaining);
            work.swap(remaining);
            planes_found++;
        }
    }

private:
    string input_topic_;
    int max_planes_;
    double distance_threshold_;
    int min_inliers_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<PlaneDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


