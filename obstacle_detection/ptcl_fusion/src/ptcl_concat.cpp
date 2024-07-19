#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_field.hpp"
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
    ConcatPointCloudsNode() : Node("concat_with_pc_node")
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
        
        ouster_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", 10, std::bind(&ConcatPointCloudsNode::ouster_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth/points", 10, std::bind(&ConcatPointCloudsNode::depth_callback, this, std::placeholders::_1));
    }


private:
    void ouster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud_1 = *msg;
        if (!cloud_2.data.empty())
        {
            concatenate_clouds();
        }
    }

    void depth_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud_2 = *msg;
        // cloud_2= convert_rgb_to_intensity(msg);
       
        if (!cloud_1.data.empty())
        {
            concatenate_clouds();
        }
    }

  
    
    sensor_msgs::msg::PointCloud2 convert_rgb_to_intensity(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 output;

        // auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
         //Modifier to describe what the fields are.
        sensor_msgs::PointCloud2Modifier modifier(output);


        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);


        output.header = msg->header;
        output.height = msg->height;
        output.width = msg->width;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = 16; 
        output.row_step = output.point_step * msg->width;
        output.is_dense = msg->is_dense;
        // output.data.resize(output.width * output.height);
        output.data.resize(msg->width * msg->height);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_rgb(*msg, "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_output_x(output, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_output_y(output, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_output_z(output, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_output_in(output, "intensity");

        //iterate over the message and populate the fields.      
        {
        uint8_t r = iter_rgb[0];
        uint8_t g = iter_rgb[1];
        uint8_t b = iter_rgb[2];
        float intensity = 0.299f * r + 0.587f * g + 0.114f * b;
        *iter_output_x = *iter_x;
        *iter_output_y = *iter_y; 
        *iter_output_z = *iter_z;
        *iter_output_in = intensity;

        // Increment the iterators
        ++iter_output_x;
        ++iter_output_y;
        ++iter_output_z;
        ++iter_output_in;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_rgb;
        }
        return output;
    }


    void concatenate_clouds()
    {
        pcl::concatenatePointCloud(cloud_1, cloud_2, concatenated_cloud);
        concatenated_cloud.fields[3].name = "intensity";
        concatenated_cloud.header.frame_id = "base_link";
        cloud_pub_->publish(cloud_2);
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


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/point_cloud2_iterator.hpp"
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/io.h>
// #include <limits>
// #include <cmath>

// float min_ang_ = 0.0f, max_ang_ = 0.0f, range_min_ = 0.0f, range_max_ = 0.0f;
// std::string frame_id_;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
// sensor_msgs::msg::PointCloud2 cloud_1;
// sensor_msgs::msg::PointCloud2 cloud_2;
// sensor_msgs::msg::PointCloud2 concatenated_cloud;

// class ConcatPointCloudsNode : public rclcpp::Node
// {
// public:
//     ConcatPointCloudsNode() : Node("concat_with_pc_node")
//     {
//         this->declare_parameter("min_ang", min_ang_);
//         this->declare_parameter("max_ang", max_ang_);
//         this->declare_parameter("range_min", range_min_);
//         this->declare_parameter("range_max", range_max_);
//         this->declare_parameter("frame_id", frame_id_);

//         this->get_parameter("min_ang", min_ang_);
//         this->get_parameter("max_ang", max_ang_);
//         this->get_parameter("range_min", range_min_);
//         this->get_parameter("range_max", range_max_);
//         this->get_parameter("frame_id", frame_id_);

//         cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_cloud", 1);
        
//         ouster_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/ouster/points", 10, std::bind(&ConcatPointCloudsNode::ouster_callback, this, std::placeholders::_1));
//         depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/depth/points", 10, std::bind(&ConcatPointCloudsNode::depth_callback, this, std::placeholders::_1));
//     }

// private:
//     void ouster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         cloud_1 = *msg;
//         if (!cloud_2.data.empty())
//         {
//             concatenate_clouds();
//         }
//     }

//     void depth_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         RCLCPP_INFO(get_logger(), "intensity");

//         cloud_2 = *msg;//*convert_rgb_to_intensity(msg);
//         if (!cloud_1.data.empty())
//         {
//             concatenate_clouds();
//         }
//     }

//     sensor_msgs::msg::PointCloud2::SharedPtr convert_rgb_to_intensity(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         RCLCPP_INFO(get_logger(), "intensity");

//         auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
//         output->header = msg->header;
//         output->height = msg->height;
//         output->width = msg->width;
//         output->is_bigendian = msg->is_bigendian;
//         output->point_step = 16;  // x, y, z, intensity
//         output->row_step = output->point_step * msg->width;
//         output->is_dense = msg->is_dense;

//         // Initialize the point cloud data with XYZ and intensity fields
//         sensor_msgs::PointCloud2Modifier pcd_modifier(*output);
//         pcd_modifier.setPointCloud2FieldsByString(2, "xyz");
//         pcd_modifier.resize(msg->width * msg->height);

//         sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
//         sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
//         sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
//         sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
//         sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
//         sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");

//         sensor_msgs::PointCloud2Iterator<float> iter_out_x(*output, "x");
//         sensor_msgs::PointCloud2Iterator<float> iter_out_y(*output, "y");
//         sensor_msgs::PointCloud2Iterator<float> iter_out_z(*output, "z");
//         sensor_msgs::PointCloud2Iterator<float> iter_out_intensity(*output, "intensity");

//         for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b,
//             ++iter_out_x, ++iter_out_y, ++iter_out_z, ++iter_out_intensity)
//         {
//             *iter_out_x = *iter_x;
//             *iter_out_y = *iter_y;
//             *iter_out_z = *iter_z;

//             uint8_t r = *iter_r;
//             uint8_t g = *iter_g;
//             uint8_t b = *iter_b;
//             float intensity = 0.299f * r + 0.587f * g + 0.114f * b;
//             RCLCPP_INFO(get_logger(), "intensity '%f'",  intensity);

//             *iter_out_intensity = intensity;
//         }

//         return output;
//     }
    
//     void concatenate_clouds()
//     {
//         // pcl::concatenatePointCloud(cloud_1, cloud_2, concatenated_cloud);
//         // concatenated_cloud.fields[3].name = "intensity";
//         // concatenated_cloud.header.frame_id = "base_link";
//         cloud_pub_->publish(cloud_2);
//     }
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ouster_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ConcatPointCloudsNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
