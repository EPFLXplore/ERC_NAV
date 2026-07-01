#ifndef PATH_PLANNING__RESTAMP_GOAL_HPP_
#define PATH_PLANNING__RESTAMP_GOAL_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_planning
{

class RestampGoal : public BT::SyncActionNode
{
public:
  RestampGoal(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("RestampGoal")};
};

}  // namespace path_planning

#endif