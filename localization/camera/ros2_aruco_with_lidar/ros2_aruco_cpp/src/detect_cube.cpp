// inclut la source existante pour éviter la duplication
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
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

void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
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
}
    



