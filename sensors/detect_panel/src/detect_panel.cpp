// ROS2 files for the lidar_filter node
// using pcl library
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/filter.h>

#include <pcl/common/common.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/shadowpoints.h>

rmw_qos_profile_t profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
        profile.history,
        profile.depth
    ),
    profile);

class DetectPanelNode : public rclcpp::Node
{
    private:
    bool reached = false;

    protected:

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;

    public:
    DetectPanelNode() : Node("NAV_detect_panel_node")
    {
        RCLCPP_INFO(this->get_logger(), "Detect panel Node Started");

        // Create a publisher
        pub_ = this->create_publisher<std_msgs::msg::String>("/detected_panel", qos);

        // // Set up QoS settings for the subscriber
        // rclcpp::QoS qos(rclcpp::KeepLast(5));  // Keep the last 10 messages
        // qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // Best-effort reliability

        // Create a subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/stereo/points", qos, std::bind(&DetectPanelNode::callback, this, std::placeholders::_1));
        destroy_sub_ = this->create_subscription<std_msgs::msg::String>("ROVER/NAV_status", qos, std::bind(&DetectPanelNode::destroy_callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud); // source on the left, target on the right

        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned2(new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> indices;

        // remove NaN points
        pcl::removeNaNFromPointCloud(*cloud, *cleaned, indices);


        // compute the median of the pointcloud

        std::vector<float> z_values;


        for (auto point : *cleaned)
        {
            if (point.z < 2.0 && point.z > 0.0)
            {
                z_values.push_back(point.z);
            }
            
        }

        std::sort(z_values.begin(), z_values.end());

        float z_median = 0;

        if (z_values.size() % 2 == 0)
        {
            z_median = (z_values[z_values.size() / 2 - 1] + z_values[z_values.size() / 2]) / 2;
        }
        else
        {
            z_median = z_values[z_values.size()+1 / 2];
        }

        z_median *= cos(17/180 * M_PI);

        RCLCPP_INFO(this->get_logger(), "z_median: '%f'", z_median);
        // std::cout<<"z_median: "<<z_median<<std::endl;

        if (z_median < 0.75 && !reached) {
            std_msgs::msg::String msg;
            msg.data = "reached";
            pub_->publish(msg);
            reached = true;
        } 

    }

    void destroy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "abort") rclcpp::shutdown();
    }
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectPanelNode>());
    rclcpp::shutdown();
    return 0;
}