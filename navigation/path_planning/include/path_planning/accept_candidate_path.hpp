#ifndef NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_
#define NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_

#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
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
  void declareParameters();

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
    const nav_msgs::msg::Path & candidate_path) const;

  bool beginningMatches(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path) const;

  bool goalChanged(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path) const;

  bool currentPathInvalid(const nav_msgs::msg::Path & path);

  BT::NodeStatus rejectCandidate(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AcceptCandidatePath")};

  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // Set while the current path is continuously invalid; used to force-accept
  // a divergent candidate once force_accept_timeout_ has elapsed.
  std::optional<rclcpp::Time> invalid_since_;

  // Tunables, loaded from ROS parameters on the bt_navigator node
  // (accept_candidate_path.*).
  double check_distance_{1.0};
  double max_initial_angle_deg_{100.0};
  double max_length_ratio_{1.5};
  double max_start_deviation_{0.3};
  double force_accept_timeout_{3.0};
  double is_path_valid_timeout_{0.2};
};

}  // namespace path_planning

#endif
