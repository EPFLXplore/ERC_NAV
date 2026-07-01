#include "path_planning/restamp_goal.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace path_planning
{

RestampGoal::RestampGoal(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList RestampGoal::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal")
  };
}

BT::NodeStatus RestampGoal::tick()
{
  geometry_msgs::msg::PoseStamped goal;

  if (!getInput("input_goal", goal)) {
    RCLCPP_WARN(logger_, "RestampGoal: missing input_goal");
    return BT::NodeStatus::FAILURE;
  }

  goal.header.stamp = node_->now();

  setOutput("output_goal", goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace path_planning