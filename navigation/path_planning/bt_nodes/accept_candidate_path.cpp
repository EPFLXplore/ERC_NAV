#include "path_planning/accept_candidate_path.hpp"
#include "path_planning/restamp_goal.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace path_planning
{

AcceptCandidatePath::AcceptCandidatePath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList AcceptCandidatePath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("current_path"),
    BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path"),

    BT::InputPort<double>(
      "check_distance", 1.0,
      "Distance in meters used to compare the beginning of paths"),

    BT::InputPort<double>(
      "max_initial_angle_deg", 100.0,
      "Reject candidate if its initial heading differs more than this"),

    BT::InputPort<double>(
      "max_length_ratio", 1.5,
      "Reject candidate if local beginning is much longer than current path")
  };
}

BT::NodeStatus AcceptCandidatePath::tick()
{
  nav_msgs::msg::Path candidate_path;
  nav_msgs::msg::Path current_path;

  if (!getInput("candidate_path", candidate_path)) {
    RCLCPP_WARN(logger_, "Reject candidate: missing candidate_path");
    return BT::NodeStatus::FAILURE;
  }

  const auto current_path_result = getInput("current_path", current_path);
  const bool has_current_path = static_cast<bool>(current_path_result);

  if (candidate_path.poses.size() < 2) {
    RCLCPP_WARN(logger_, "Reject candidate: path has fewer than 2 poses");
    return BT::NodeStatus::FAILURE;
  }

  if (!has_current_path || current_path.poses.size() < 2) {
    setOutput("output_path", candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: no usable current path");
    return BT::NodeStatus::SUCCESS;
  }

  if (isCandidateAcceptable(current_path, candidate_path)) {
    setOutput("output_path", candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate path");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(logger_, "Reject candidate path");
  return BT::NodeStatus::FAILURE;
}

bool AcceptCandidatePath::isCandidateAcceptable(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path)
{
  double check_distance = 1.0;
  double max_initial_angle_deg = 100.0;
  double max_length_ratio = 1.5;

  getInput("check_distance", check_distance);
  getInput("max_initial_angle_deg", max_initial_angle_deg);
  getInput("max_length_ratio", max_length_ratio);

  const double current_heading = initialHeading(current_path, check_distance);
  const double candidate_heading = initialHeading(candidate_path, check_distance);

  const double heading_change_deg =
    std::abs(angleDiff(candidate_heading, current_heading)) * 180.0 / M_PI;

  if (heading_change_deg > max_initial_angle_deg) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: heading change %.1f deg > %.1f deg",
      heading_change_deg,
      max_initial_angle_deg);
    return false;
  }

  const double current_len = pathLength(current_path, check_distance);
  const double candidate_len = pathLength(candidate_path, check_distance);

  if (candidate_len > current_len * max_length_ratio) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: local length %.2f > %.2f * %.2f",
      candidate_len,
      current_len,
      max_length_ratio);
    return false;
  }

  return true;
}

double AcceptCandidatePath::pathLength(
  const nav_msgs::msg::Path & path,
  double max_distance) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total = 0.0;

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;

    const double ds = std::hypot(b.x - a.x, b.y - a.y);
    total += ds;

    if (total >= max_distance) {
      return max_distance;
    }
  }

  return total;
}

double AcceptCandidatePath::initialHeading(
  const nav_msgs::msg::Path & path,
  double lookahead_distance) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  const auto & start = path.poses.front().pose.position;

  double accumulated = 0.0;

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & p = path.poses[i].pose.position;

    accumulated += std::hypot(p.x - prev.x, p.y - prev.y);

    if (accumulated >= lookahead_distance || i == path.poses.size() - 1) {
      return std::atan2(p.y - start.y, p.x - start.x);
    }
  }

  const auto & p = path.poses.back().pose.position;
  return std::atan2(p.y - start.y, p.x - start.x);
}

double AcceptCandidatePath::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double AcceptCandidatePath::angleDiff(double a, double b) const
{
  return normalizeAngle(a - b);
}

}  // namespace path_planning

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<path_planning::AcceptCandidatePath>(
    "AcceptCandidatePath");

  factory.registerNodeType<path_planning::RestampGoal>(
    "RestampGoal");

}