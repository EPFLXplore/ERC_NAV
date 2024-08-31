// ROS2 files for the lidar_filter node
// using pcl library
#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/shadowpoints.h>
#include <iostream>
#include <string>

rmw_qos_profile_t real_profile = {
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
rmw_qos_profile_t sim_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};



class LidarFilterNode : public rclcpp::Node
{
    private:

    double min_dist; // minimum distance of points to keep (in meters)
    double slope_threshold; // slope threshold for the floor (in percentage)
    bool filter_bool; // whether to filter or not
    bool sim;
    float voxel_leaf_size;
    int n_clusters;

    protected:

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_floor;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;


    public:
    LidarFilterNode() : Node("NAV_lidar_filter_node")
    {
        RCLCPP_INFO(this->get_logger(), "Lidar Filter Node Started");

        this->declare_parameter("min_distance", 0.8);
        this->declare_parameter("slope_threshold", 0.4);    //100% is 45 degrees to horizontal
        this->declare_parameter("filter", true);
        this->declare_parameter("voxel_leaf_size", 0.05);
        this->declare_parameter("n_clusters", 50);
        this->declare_parameter("sim", false);

        
        sim = this->get_parameter("sim").as_bool();

        auto profile = sim ? sim_profile : real_profile;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                profile.history,
                profile.depth
            ),
            profile
        );

        std::string topic_name = sim ? "/ouster/points" : "/points";
        
        // Create a publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_filter/filtered_points", qos);
        pub_floor = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_filter/floor_points", qos);

        // Create subscribers lidar points and NAV_status (to destroy node)
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, qos, std::bind(&LidarFilterNode::callback, this, std::placeholders::_1));
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        filter_bool = this->get_parameter("filter").as_bool();
        

        if (!filter_bool) {
            RCLCPP_INFO(this->get_logger(), "Not filtering lidar.");
            pub_floor->publish(*msg);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Filtering lidar.");

        min_dist = this->get_parameter("min_distance").as_double();
        slope_threshold = this->get_parameter("slope_threshold").as_double();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        n_clusters = this->get_parameter("n_clusters").as_int();



        
        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud); // source on the left, target on the right

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr floor_points(new pcl::PointCloud<pcl::PointXYZ>());

        // voxel grid filter to downsample
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(voxel_leaf_size,voxel_leaf_size,voxel_leaf_size);
        filter.filter(*filtered_points); // output dataset is the filtered_points

        // compute kmeans and compute normals
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setInputCloud(filtered_points);
        normal_estimator.setKSearch(n_clusters);
        normal_estimator.useSensorOriginAsViewPoint();

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimator.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr points_normal(new pcl::PointCloud<pcl::Normal>());
        normal_estimator.compute(*points_normal);   

        // iterate through the points and normals
        auto normal_iterator = points_normal->begin();

        for(auto point : *filtered_points) {
            pcl::Normal normal = *normal_iterator++;

            float normal_h = sqrt(normal.normal_x*normal.normal_x + normal.normal_y*normal.normal_y);   // 
            float slope_percentage = abs(normal_h / normal.normal_z);

            if(point.x*point.x + point.y*point.y > min_dist*min_dist) { 
                if(slope_percentage > slope_threshold) {
                    obstacle_points->push_back(point);
                } else {
                    floor_points->push_back(point);
                }
            }
        }
        
        // Filter
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(0.0, 1.0);
        // pass.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);

        // Obstacles
        pcl::toROSMsg(*obstacle_points, *output);
        output->header.frame_id = msg->header.frame_id;
        pub_->publish(*output);

        // Floor
        pcl::toROSMsg(*floor_points, *output);
        output->header.frame_id = msg->header.frame_id;
        pub_floor->publish(*output);

        // Publish
        // sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
        // pcl::toROSMsg(*cloud_filtered, *output);
        // output->header.frame_id = "base_link";
        // pub_->publish(*output);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilterNode>());
    rclcpp::shutdown();
    return 0;
}
