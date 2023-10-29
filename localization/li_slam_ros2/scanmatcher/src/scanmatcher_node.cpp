#include "scanmatcher/scanmatcher_component.h"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<graphslam::ScanMatcherComponent>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
