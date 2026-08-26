#include "path_planning/accept_candidate_path.hpp"
#include "path_planning/restamp_goal.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace path_planning
{

AcceptCandidatePath::AcceptCandidatePath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  robot_base_frame_ = node_->get_parameter("robot_base_frame").as_string();

  declareParameters();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  is_path_valid_client_ = node_->create_client<nav2_msgs::srv::IsPathValid>(
    "is_path_valid", rmw_qos_profile_services_default, callback_group_);
}

void AcceptCandidatePath::declareParameters()
{
  const std::string ns = "accept_candidate_path.";

  const auto declare = [this, &ns](const std::string & param, double default_value) {
      if (!node_->has_parameter(ns + param)) {
        node_->declare_parameter(ns + param, default_value);
      }
      return node_->get_parameter(ns + param).as_double();
    };

  check_distance_ = declare("check_distance", check_distance_);
  max_initial_angle_deg_ = declare("max_initial_angle_deg", max_initial_angle_deg_);
  max_length_ratio_ = declare("max_length_ratio", max_length_ratio_);
  max_start_deviation_ = declare("max_start_deviation", max_start_deviation_);
  force_accept_timeout_ = declare("force_accept_timeout", force_accept_timeout_);
  is_path_valid_timeout_ = declare("is_path_valid_timeout", is_path_valid_timeout_);

  RCLCPP_INFO(
    logger_,
    "Params: check_distance=%.2f max_initial_angle_deg=%.1f max_length_ratio=%.2f "
    "max_start_deviation=%.2f force_accept_timeout=%.1f is_path_valid_timeout=%.2f",
    check_distance_, max_initial_angle_deg_, max_length_ratio_,
    max_start_deviation_, force_accept_timeout_, is_path_valid_timeout_);
}

BT::PortsList AcceptCandidatePath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("current_path"),
    BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path")
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
    invalid_since_.reset();
    setOutput("output_path", candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: no usable current path");
    return BT::NodeStatus::SUCCESS;
  }

  if (goalChanged(current_path, candidate_path)) {
    invalid_since_.reset();
    setOutput("output_path", candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: goal changed");
    return BT::NodeStatus::SUCCESS;
  }

  if (isCandidateAcceptable(current_path, candidate_path)) {
    invalid_since_.reset();
    setOutput("output_path", candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate path");
    return BT::NodeStatus::SUCCESS;
  }

  return rejectCandidate(current_path, candidate_path);
}

BT::NodeStatus AcceptCandidatePath::rejectCandidate(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path)
{
  if (currentPathInvalid(current_path)) {
    if (!invalid_since_) {
      invalid_since_ = node_->now();
    }

    const double invalid_for = (node_->now() - *invalid_since_).seconds();

    if (invalid_for > force_accept_timeout_) {
      invalid_since_.reset();
      setOutput("output_path", candidate_path);
      RCLCPP_WARN(
        logger_,
        "Force-accept divergent candidate: current path invalid for %.1f s > %.1f s",
        invalid_for, force_accept_timeout_);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(
      logger_,
      "Reject candidate, current path invalid for %.1f s (force-accept at %.1f s)",
      invalid_for, force_accept_timeout_);
  } else {
    invalid_since_.reset();
    RCLCPP_WARN(logger_, "Reject candidate path, keeping current path");
  }

  // Keep following the current path; rejection is normal operation and must
  // not fail the tree (that would halt FollowPath and trigger recovery).
  setOutput("output_path", current_path);
  return BT::NodeStatus::SUCCESS;
}

bool AcceptCandidatePath::currentPathInvalid(const nav_msgs::msg::Path & path)
{
  if (!is_path_valid_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 5000,
      "is_path_valid service not available, assuming current path is valid");
    return false;
  }

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request->path = path;

  auto future = is_path_valid_client_->async_send_request(request);

  const auto timeout = std::chrono::duration<double>(is_path_valid_timeout_);
  if (callback_group_executor_.spin_until_future_complete(future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "is_path_valid did not answer in time, assuming valid");
    return false;
  }

  return !future.get()->is_valid;
}

bool AcceptCandidatePath::isCandidateAcceptable(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path) const
{
  const double candidate_heading = initialHeading(candidate_path, check_distance_);

  std::string candidate_frame = candidate_path.header.frame_id;
  if (candidate_frame.empty() && !candidate_path.poses.empty()) {
    candidate_frame = candidate_path.poses.front().header.frame_id;
  }

  if (candidate_frame.empty()) {
    RCLCPP_WARN(logger_, "Reject candidate: path frame is empty");
    return false;
  }

  double robot_heading;
  try {
    // The transform rotation is the current base_link heading expressed in the
    // candidate path frame. Subtracting it from the path direction is
    // equivalent to expressing that direction in base_link and comparing it
    // with base_link's +X axis.
    const auto path_from_base = tf_buffer_->lookupTransform(
      candidate_frame, robot_base_frame_, tf2::TimePointZero);
    robot_heading = tf2::getYaw(path_from_base.transform.rotation);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: cannot get current %s heading in %s: %s",
      robot_base_frame_.c_str(), candidate_frame.c_str(), ex.what());
    return false;
  }

  const double heading_change_deg =
    std::abs(angleDiff(candidate_heading, robot_heading)) * 180.0 / M_PI;

  if (heading_change_deg > max_initial_angle_deg_) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: initial direction is %.1f deg from current %s heading > %.1f deg",
      heading_change_deg,
      robot_base_frame_.c_str(),
      max_initial_angle_deg_);
    return false;
  }

  const double current_len = pathLength(current_path, check_distance_);
  const double candidate_len = pathLength(candidate_path, check_distance_);

  if (candidate_len > current_len * max_length_ratio_) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: local length %.2f > %.2f * %.2f",
      candidate_len,
      current_len,
      max_length_ratio_);
    return false;
  }

  if (!beginningMatches(current_path, candidate_path)) {
    return false;
  }

  return true;
}

bool AcceptCandidatePath::beginningMatches(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path) const
{
  // Every candidate pose within check_distance_ of its start must stay within
  // max_start_deviation_ of the beginning of the current path, so an accepted
  // update never moves the section the controller is currently tracking.
  double candidate_travelled = 0.0;

  for (size_t i = 1; i < candidate_path.poses.size() &&
    candidate_travelled < check_distance_; ++i)
  {
    const auto & prev = candidate_path.poses[i - 1].pose.position;
    const auto & p = candidate_path.poses[i].pose.position;
    candidate_travelled += std::hypot(p.x - prev.x, p.y - prev.y);

    double min_dist = std::numeric_limits<double>::max();
    double current_travelled = 0.0;

    for (size_t j = 0; j < current_path.poses.size() &&
      current_travelled < 2.0 * check_distance_; ++j)
    {
      if (j > 0) {
        const auto & a = current_path.poses[j - 1].pose.position;
        const auto & b = current_path.poses[j].pose.position;
        current_travelled += std::hypot(b.x - a.x, b.y - a.y);
      }

      const auto & q = current_path.poses[j].pose.position;
      min_dist = std::min(min_dist, std::hypot(p.x - q.x, p.y - q.y));
    }

    if (min_dist > max_start_deviation_) {
      RCLCPP_WARN(
        logger_,
        "Reject candidate: start deviates %.2f m > %.2f m from current path",
        min_dist,
        max_start_deviation_);
      return false;
    }
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

bool AcceptCandidatePath::goalChanged(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path) const
{
  if (current_path.poses.empty() || candidate_path.poses.empty()) {
    return true;
  }

  const auto & a = current_path.poses.back().pose.position;
  const auto & b = candidate_path.poses.back().pose.position;

  const double dist = std::hypot(b.x - a.x, b.y - a.y);
  return dist > 0.25;
}

}  // namespace path_planning



BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<path_planning::AcceptCandidatePath>(
    "AcceptCandidatePath");

  factory.registerNodeType<path_planning::RestampGoal>(
    "RestampGoal");

}
