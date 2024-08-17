#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <builtin_interfaces/msg/duration.h>
#include <limits>
#include <cmath>

float min_ang_, max_ang_, range_min_, range_max_;
std::string frame_id_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
sensor_msgs::msg::PointCloud2 cloud_1;
sensor_msgs::msg::PointCloud2 cloud_2;
sensor_msgs::msg::PointCloud2 concatenated_cloud;


class ConcatPointCloudsNode : public rclcpp::Node
{
public:
    ConcatPointCloudsNode() : Node("concat_with_pc_node"),
                             tf_buffer_(rclcpp::Duration::Duration(30.0))

    {
        this->declare_parameter("min_ang", min_ang_);
        this->declare_parameter("max_ang", max_ang_);
        this->declare_parameter("range_min", range_min_);
        this->declare_parameter("range_max", range_max_);
        this->declare_parameter("frame_id", frame_id_);

        this->get_parameter("min_ang", min_ang_);
        this->get_parameter("max_ang", max_ang_);
        this->get_parameter("range_min", range_min_);
        this->get_parameter("range_max", range_max_);
        this->get_parameter("frame_id", frame_id_);

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_cloud", 1);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        ouster_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", qos, std::bind(&ConcatPointCloudsNode::ouster_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/stereo/points", qos, std::bind(&ConcatPointCloudsNode::depth_callback, this, std::placeholders::_1));
        // auto clock = this->get_clock();
        // tf2_ros::Buffer tf_buffer_;
        // tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(30.0)));

    }


private:


    //lidar pointcloud
    void ouster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ouster callback");
        // cloud_1 = *msg;
        //remove unecessary fields in ouster msg, not present in depth msg
        cloud_1 = clean_msg(msg);

        if (!cloud_2.data.empty())
        {
            concatenate_clouds();
        }

    }

    // depth camera pointcloud
    void depth_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "stereo callback");

        // change the reference frame to match the lidar

        tf2_ros::Buffer tf_buffer_(tf2::Duration(1));// Not sure what duration to put
        geometry_msgs::msg::TransformStamped transform;
        
        try
        {
            transform = tf_buffer_.lookupTransform("lidar_link", msg->header.frame_id,
            msg->header.stamp, tf2::Duration(30)); // Not sure what duration to put
            //lookupTransform(const std::string& target_frame, const std::string& source_frame,
                  //  const tf2::TimePoint& time, const tf2::Duration timeout) 
            sensor_msgs::msg::PointCloud2 cloud_out;
            tf2::doTransform(*msg, cloud_out, transform);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // cloud_2 = *msg;
        cloud_2 = cloud_out;

        // Publish only trandformed ptcl for now
        //cloud_pub_->publish(cloud_2);

        //( cloud_2.is_dense=true;)
        
        if (!cloud_1.data.empty())
        {
            concatenate_clouds();
        }

    }

  
    sensor_msgs::msg::PointCloud2 clean_msg(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Define a new PoitnCloud msg 
        sensor_msgs::msg::PointCloud2 output;

        // Define the fields of the PointCloud msg
        sensor_msgs::PointCloud2Modifier modifier(output);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);


        // Parameters
        output.header = msg->header;
        output.height = msg->height;
        output.width = msg->width;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = 16; 
        output.row_step = output.point_step * msg->width;
        output.is_dense = msg->is_dense;
        // output.data.resize(output.width * output.height);
        output.data.resize(msg->width * msg->height);


        // Iteration over msg and populate the fields with lidar data
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_in(*msg, "intensity");

        sensor_msgs::PointCloud2Iterator<float> iter_output_x(output, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_output_y(output, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_output_z(output, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_output_in(output, "intensity");

        {
        *iter_output_x = *iter_x;
        *iter_output_y = *iter_y; 
        *iter_output_z = *iter_z;
        *iter_output_in = *iter_in;

        // Increment the iterators
        ++iter_output_x;
        ++iter_output_y;
        ++iter_output_z;
        ++iter_output_in;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_in;
        }
        return output;
    }


    void concatenate_clouds()
    {
        pcl::concatenatePointCloud(cloud_1, cloud_2, concatenated_cloud);
        concatenated_cloud.fields[3].name = "intensity";
        concatenated_cloud.header.frame_id = "lidar_link";
        concatenated_cloud.data.resize(concatenated_cloud.width * concatenated_cloud.height * concatenated_cloud.point_step);
        cloud_pub_->publish(concatenated_cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ouster_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConcatPointCloudsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
