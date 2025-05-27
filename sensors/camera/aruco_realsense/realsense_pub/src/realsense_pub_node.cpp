#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

class RealSensePub : public rclcpp::Node {
public:
  RealSensePub() : Node("realsense_pub") {
    // Parameters
    this->declare_parameter<std::string>("topic", "/camera/color/image/compressed");
    this->declare_parameter<std::string>("serial_number", "102122061110");  // Optional

    std::string topic = this->get_parameter("topic").as_string();
    std::string serial = this->get_parameter("serial_number").as_string();

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 10);

    // RealSense config
    if (!serial.empty()) {
      cfg_.enable_device(serial);
      RCLCPP_INFO(this->get_logger(), "Using RealSense device with serial: %s", serial.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "No serial number specified. Using default RealSense device.");
    }

    cfg_.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_Y8, 15);
    pipe_.start(cfg_);

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(66), // ~15 fps
      std::bind(&RealSensePub::captureAndPublish, this));
  }

private:
  void captureAndPublish() {
    rs2::frameset frameset;
    if (!pipe_.poll_for_frames(&frameset)) return;

    rs2::frame color_frame = frameset.get_color_frame();
    if (!color_frame) return;

    cv::Mat image(cv::Size(1280, 720), CV_8UC1, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf, {cv::IMWRITE_JPEG_QUALITY, 70});


    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = this->get_clock()->now();
    msg.format = "jpeg";
    msg.data = std::move(buf);
    pub_->publish(msg);
  }

  rs2::pipeline pipe_;
  rs2::config cfg_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSensePub>());
  rclcpp::shutdown();
  return 0;
}
