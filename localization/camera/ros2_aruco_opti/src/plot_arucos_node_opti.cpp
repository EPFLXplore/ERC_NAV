#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

class ArucoMarkersViz : public rclcpp::Node
{
public:
  ArucoMarkersViz()
      : Node("aruco_markers_viz")
  {
    declare_parameter<std::string>("input_topic", "aruco_markers");
    declare_parameter<std::string>("output_topic", "aruco_markers_vis");
    declare_parameter<double>("marker_size", 0.144);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    marker_size_ = get_parameter("marker_size").as_double();

    sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        input_topic,
        10,
        std::bind(&ArucoMarkersViz::callback, this, std::placeholders::_1));

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);

    RCLCPP_INFO(
        get_logger(),
        "Listening on %s, publishing MarkerArray on %s",
        input_topic.c_str(),
        output_topic.c_str());
  }

private:
  void callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const size_t n = std::min(msg->marker_ids.size(), msg->poses.size());

    for (size_t i = 0; i < n; ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "aruco";
      marker.id = static_cast<int32_t>(msg->marker_ids[i]);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = msg->poses[i];

      marker.scale.x = marker_size_;
      marker.scale.y = marker_size_;
      marker.scale.z = marker_size_;

      marker.color.r = 0.0F; 
      marker.color.g = 1.0F;
      marker.color.b = 0.0F;
      marker.color.a = 0.6F;

      marker_array.markers.push_back(marker);
    }

    pub_->publish(marker_array);
  }

  double marker_size_{0.144};
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMarkersViz>());
  rclcpp::shutdown();
  return 0;
}
