#include "path_planning/accept_candidate_path_stupid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"

namespace path_planning
{

AcceptCandidatePathStupid::AcceptCandidatePathStupid(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  robot_base_frame_ = node_->get_parameter("robot_base_frame").as_string();

  const std::string ns = "accept_candidate_path_stupid.";
  if (!node_->has_parameter(ns + "check_distance")) {
    node_->declare_parameter(ns + "check_distance", check_distance_);
  }
  if (!node_->has_parameter(ns + "minimum_heading_change_deg")) {
    node_->declare_parameter(
      ns + "minimum_heading_change_deg", minimum_heading_change_deg_);
  }

  check_distance_ = std::max(
    0.0, node_->get_parameter(ns + "check_distance").as_double());
  minimum_heading_change_deg_ = std::max(
    0.0, node_->get_parameter(ns + "minimum_heading_change_deg").as_double());
}

BT::PortsList AcceptCandidatePathStupid::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("current_path"),
    BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path")
  };
}

BT::NodeStatus AcceptCandidatePathStupid::tick()
{
  nav_msgs::msg::Path current_path;
  nav_msgs::msg::Path candidate_path;

  if (!getInput("candidate_path", candidate_path) || candidate_path.poses.size() < 2) {
    RCLCPP_ERROR(logger_, "FAILURE: missing or unusable candidate path");
    return BT::NodeStatus::FAILURE;
  }

  const auto current_result = getInput("current_path", current_path);
  if (!current_result || current_path.poses.size() < 2) {
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: no usable current path");
    return BT::NodeStatus::SUCCESS;
  }

  if (pathFrame(current_path) != pathFrame(candidate_path)) {
    outputPath(current_path);
    RCLCPP_WARN(logger_, "Keep current path: path frames differ");
    return BT::NodeStatus::SUCCESS;
  }

  const auto current_heading = prefixHeading(current_path);
  const auto candidate_heading = prefixHeading(candidate_path);
  if (!current_heading || !candidate_heading) {
    outputPath(current_path);
    RCLCPP_WARN(logger_, "Keep current path: prefix heading unavailable");
    return BT::NodeStatus::SUCCESS;
  }

  const double heading_change_deg =
    std::abs(normalizeAngle(*candidate_heading - *current_heading)) * 180.0 / M_PI;
  if (heading_change_deg >= minimum_heading_change_deg_) {
    outputPath(candidate_path);
    RCLCPP_WARN(
      logger_, "Accept candidate: heading changes %.1f deg >= %.1f deg",
      heading_change_deg, minimum_heading_change_deg_);
  } else {
    outputPath(current_path);
    RCLCPP_INFO(
      logger_, "Keep current path: heading changes %.1f deg < %.1f deg",
      heading_change_deg, minimum_heading_change_deg_);
  }

  return BT::NodeStatus::SUCCESS;
}

std::optional<std::size_t> AcceptCandidatePathStupid::nearestPathIndex(
  const nav_msgs::msg::Path & path) const
{
  const std::string frame = pathFrame(path);
  if (path.poses.empty() || frame.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped path_from_base;
  try {
    path_from_base = tf_buffer_->lookupTransform(
      frame, robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "Cannot locate rover in path frame: %s", ex.what());
    return std::nullopt;
  }

  const double robot_x = path_from_base.transform.translation.x;
  const double robot_y = path_from_base.transform.translation.y;
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & point = path.poses[i].pose.position;
    const double distance = std::hypot(point.x - robot_x, point.y - robot_y);
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }
  return nearest;
}

std::optional<double> AcceptCandidatePathStupid::prefixHeading(
  const nav_msgs::msg::Path & path) const
{
  const auto start = nearestPathIndex(path);
  if (!start || *start >= path.poses.size() - 1) {
    return std::nullopt;
  }

  const auto & origin = path.poses[*start].pose.position;
  double travelled = 0.0;
  for (std::size_t i = *start + 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & point = path.poses[i].pose.position;
    travelled += std::hypot(point.x - previous.x, point.y - previous.y);
    if (travelled >= check_distance_ || i == path.poses.size() - 1) {
      return std::atan2(point.y - origin.y, point.x - origin.x);
    }
  }
  return std::nullopt;
}

std::string AcceptCandidatePathStupid::pathFrame(const nav_msgs::msg::Path & path) const
{
  if (!path.header.frame_id.empty()) {
    return path.header.frame_id;
  }
  return path.poses.empty() ? std::string{} : path.poses.front().header.frame_id;
}

double AcceptCandidatePathStupid::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void AcceptCandidatePathStupid::outputPath(const nav_msgs::msg::Path & path)
{
  setOutput("output_path", path);
}

}  // namespace path_planning
