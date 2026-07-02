#ifndef NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_
#define NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_planning
{

class AcceptCandidatePath : public BT::SyncActionNode
{
public:
  AcceptCandidatePath(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  double pathLength(
    const nav_msgs::msg::Path & path,
    double max_distance) const;

  double initialHeading(
    const nav_msgs::msg::Path & path,
    double lookahead_distance) const;

  double normalizeAngle(double angle) const;
  double angleDiff(double a, double b) const;

  bool isCandidateAcceptable(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path);

  bool goalChanged(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AcceptCandidatePath")};
};

}  // namespace path_planning

#endif