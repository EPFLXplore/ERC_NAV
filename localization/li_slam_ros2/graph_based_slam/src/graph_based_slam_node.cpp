#include "graph_based_slam/graph_based_slam_component.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<graphslam::GraphBasedSlamComponent>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
