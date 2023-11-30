#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <string>
#include <stdint.h>
#include <chrono>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"

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


class DepthMapFilter : public rclcpp::Node
{
    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->height); // 400
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->width); // 640
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", (msg->encoding));
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", (msg->step)); // nombre de bytes per row 2560
        std::cout<<msg->encoding<<std::endl;

        // 2560 / 640 = 4 bytes par pixels

        uint32_t width = msg->width;
        uint32_t height = msg->height;

        auto start = std::chrono::high_resolution_clock::now();

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;

        ///////////////////////////////////////
        // average image out
        // float* beginning = image.ptr<float>();

        // float* avg = new float[(width-2) * (height-2)];

        // cv::Mat float_image(height-2, width-2, CV_32FC1);
        // float* data = float_image.ptr<float>();

        // uint32_t index_orig(0), index(0), index_1(0), index1(0), index_2(0), index2(0);

        // for(int i = 1; i < height-1; i++)
        // {
        //     for(int j = 1; j < width-1; j++)
        //     {
        //         index_orig = i * width + j;

        //         avg[index] += beginning[index_orig]; // add value of pixel
        //         avg[index] += beginning[index_orig+1] + beginning[index_orig-1]; // add pixel left and right
        //         avg[index] += beginning[index_orig + width] + beginning[index_orig - width]; // add pixel top and bottom
        //         avg[index] += beginning[index_orig + width + 1] + beginning[index_orig + width - 1];
        //         avg[index] += beginning[index_orig];

        //     }

        // }


        /////////////////////////////////////////

        // compute delta of depth between each pixel and use threshold.

        float* beginning = image.ptr<float>();
        float* grad_mat_h = new float[(width-2)*(height-2)];
        float* grad_mat_w = new float[(width-2)*(height-2)];
        cv::Mat float_image(height-2, width-2, CV_32FC1);
        //float_image.header = msg->header;
        float* data = float_image.ptr<float>();

        uint32_t index_orig(0), index(0), index_1(0), index1(0), index_2(0), index2(0);

        float sum_grad(0.0);


        
        for(int i = 1; i < height-1; i++)
        {
            for(int j = 1; j < width-1; j++)
            {
                index_orig = i * width + j;
                
                // row wise gradient
                index_1 = (i-1) * width + j;
                index1 = (i+1) * width + j;
                grad_mat_h[index] = beginning[index1] - beginning[index_1];

                // column wise gradient
                index_2 = i * width + j - 1;
                index2 = i * width + j + 1;
                grad_mat_w[index] = beginning[index2] - beginning[index_2];

                sum_grad = (std::abs(grad_mat_h[index]) + std::abs(grad_mat_w[index])) / beginning[index_orig];

                // threshold
                if (sum_grad < 0.0005 && beginning[index_orig]<5.00) // filter out if more than 5m0 meters away
                {
                    data[index] = beginning[index_orig]; // set the filtered value to be equal to orignal value
                    //std::cout<<sum_grad<<"_";
                    
                } else
                {
                    data[index] = 0.0; // filtered value = 0 so it doesn't appear on the map
                }

                index += 1;
                
                
            }
            //std::cout<<std::endl;
        }

        cv_bridge::CvImage cv_image;
        cv_image.image = float_image;
        cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Set the image encoding to float
        //cv_image.header = std_msgs::msg::Header();
        cv_image.header = msg->header;
        sensor_msgs::msg::Image::SharedPtr msg_new = cv_image.toImageMsg();

        auto end = std::chrono::high_resolution_clock::now();

        double duration = std::chrono::duration<double, std::milli>(end-start).count();
        std::cout<<"Execution time: " << duration << "ms\n";

        publisher_->publish(*msg_new.get());

        delete grad_mat_h;
        delete grad_mat_w;
        

        RCLCPP_INFO(this->get_logger(), "I published");

        // canny and sobel for edge detection
    }

    public:
    DepthMapFilter()
    : Node("pc_front_camera_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/stereo/converted_depth", qos, std::bind(&DepthMapFilter::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_depth", qos);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // create subscriber for the 
    rclcpp::spin(std::make_shared<DepthMapFilter>());
    rclcpp::shutdown();


    return 0;
}