#include "path_planning/accept_candidate_path.hpp"
#include "path_planning/restamp_goal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  costmap_qos.transient_local().reliable();
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_, costmap_qos,
    [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap_ = msg;
    });

  auto accepted_path_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  accepted_path_qos.transient_local().reliable();
  accepted_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    accepted_path_topic_, accepted_path_qos);
}

void AcceptCandidatePath::declareParameters()
{
  const std::string ns = "accept_candidate_path.";

  const auto declareDouble = [this, &ns](const std::string & param, double default_value) {
      if (!node_->has_parameter(ns + param)) {
        node_->declare_parameter(ns + param, default_value);
      }
      return node_->get_parameter(ns + param).as_double();
    };

  const auto declareBool = [this, &ns](const std::string & param, bool default_value) {
      if (!node_->has_parameter(ns + param)) {
        node_->declare_parameter(ns + param, default_value);
      }
      return node_->get_parameter(ns + param).as_bool();
    };

  const auto declareString = [this, &ns](
    const std::string & param, const std::string & default_value)
    {
      if (!node_->has_parameter(ns + param)) {
        node_->declare_parameter(ns + param, default_value);
      }
      return node_->get_parameter(ns + param).as_string();
    };

  check_distance_ = declareDouble("check_distance", check_distance_);
  max_initial_angle_deg_ = declareDouble("max_initial_angle_deg", max_initial_angle_deg_);
  max_length_ratio_ = declareDouble("max_length_ratio", max_length_ratio_);
  max_start_deviation_ = declareDouble("max_start_deviation", max_start_deviation_);
  obstacle_check_distance_ = declareDouble(
    "obstacle_check_distance", obstacle_check_distance_);
  obstacle_switch_cost_ = declareDouble("obstacle_switch_cost", obstacle_switch_cost_);
  obstacle_confirmation_time_ = declareDouble(
    "obstacle_confirmation_time", obstacle_confirmation_time_);
  minimum_cost_improvement_ = declareDouble(
    "minimum_cost_improvement", minimum_cost_improvement_);
  treat_unknown_as_obstacle_ = declareBool(
    "treat_unknown_as_obstacle", treat_unknown_as_obstacle_);
  costmap_topic_ = declareString("costmap_topic", costmap_topic_);
  accepted_path_topic_ = declareString("accepted_path_topic", accepted_path_topic_);

  RCLCPP_INFO(
    logger_,
    "Params: check_distance=%.2f max_initial_angle_deg=%.1f max_length_ratio=%.2f "
    "max_start_deviation=%.2f obstacle_check_distance=%.2f obstacle_switch_cost=%.1f "
    "obstacle_confirmation_time=%.2f minimum_cost_improvement=%.1f "
    "costmap_topic=%s accepted_path_topic=%s",
    check_distance_, max_initial_angle_deg_, max_length_ratio_, max_start_deviation_,
    obstacle_check_distance_, obstacle_switch_cost_, obstacle_confirmation_time_,
    minimum_cost_improvement_, costmap_topic_.c_str(), accepted_path_topic_.c_str());
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
    obstacle_since_.reset();
    RCLCPP_WARN(logger_, "Reject candidate: missing candidate_path");
    return BT::NodeStatus::FAILURE;
  }

  const auto current_path_result = getInput("current_path", current_path);
  const bool has_current_path = static_cast<bool>(current_path_result);

  if (candidate_path.poses.size() < 2) {
    obstacle_since_.reset();
    RCLCPP_WARN(logger_, "Reject candidate: path has fewer than 2 poses");
    return BT::NodeStatus::FAILURE;
  }

  if (!has_current_path || current_path.poses.size() < 2) {
    obstacle_since_.reset();
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: no usable current path");
    return BT::NodeStatus::SUCCESS;
  }

  if (goalChanged(current_path, candidate_path)) {
    obstacle_since_.reset();
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: goal changed");
    return BT::NodeStatus::SUCCESS;
  }

  PathCost current_cost;
  PathCost candidate_cost;
  if (shouldSwitchForObstacle(current_path, candidate_path, current_cost, candidate_cost)) {
    const auto now = node_->now();
    if (!obstacle_since_) {
      obstacle_since_ = now;
    }

    const double blocked_for = std::max(0.0, (now - *obstacle_since_).seconds());
    if (blocked_for >= obstacle_confirmation_time_) {
      obstacle_since_.reset();
      outputPath(candidate_path);
      RCLCPP_WARN(
        logger_,
        "Accept obstacle detour: current max/mean %.0f/%.1f, candidate %.0f/%.1f",
        current_cost.maximum, current_cost.mean,
        candidate_cost.maximum, candidate_cost.mean);
      return BT::NodeStatus::SUCCESS;
    }

    outputPath(current_path);
    RCLCPP_WARN(
      logger_,
      "Obstacle detour waiting for confirmation: %.2f / %.2f s",
      blocked_for, obstacle_confirmation_time_);
    return BT::NodeStatus::SUCCESS;
  }

  obstacle_since_.reset();

  if (current_cost.samples > 0 && current_cost.maximum >= obstacle_switch_cost_) {
    outputPath(current_path);
    RCLCPP_WARN(
      logger_,
      "Keep blocked current path: candidate is not yet clear/better enough "
      "(current max/mean %.0f/%.1f, candidate %.0f/%.1f)",
      current_cost.maximum, current_cost.mean,
      candidate_cost.maximum, candidate_cost.mean);
    return BT::NodeStatus::SUCCESS;
  }

  if (isCandidateAcceptable(current_path, candidate_path)) {
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept stable candidate path");
    return BT::NodeStatus::SUCCESS;
  }

  // A geometrically divergent replan is ignored while the followed path is
  // below the obstacle threshold. FollowPath therefore keeps the same prefix.
  outputPath(current_path);
  RCLCPP_WARN(logger_, "Reject divergent candidate: no obstacle-triggered switch");
  return BT::NodeStatus::SUCCESS;
}

bool AcceptCandidatePath::shouldSwitchForObstacle(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path,
  PathCost & current_cost,
  PathCost & candidate_cost) const
{
  const auto current_score = scorePathAhead(current_path);
  const auto candidate_score = scorePathAhead(candidate_path);
  if (!current_score || !candidate_score) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 5000,
      "Cannot compare path costs yet; keeping the current path stable");
    return false;
  }

  current_cost = *current_score;
  candidate_cost = *candidate_score;

  const bool current_blocked = current_cost.maximum >= obstacle_switch_cost_;
  const bool candidate_clear = candidate_cost.maximum < obstacle_switch_cost_;
  const bool maximum_improved =
    candidate_cost.maximum + minimum_cost_improvement_ <= current_cost.maximum;
  const bool mean_improved =
    candidate_cost.mean + minimum_cost_improvement_ <= current_cost.mean;

  return current_blocked && candidate_clear && (maximum_improved || mean_improved);
}

std::optional<AcceptCandidatePath::PathCost> AcceptCandidatePath::scorePathAhead(
  const nav_msgs::msg::Path & path) const
{
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = costmap_;
  }

  if (!costmap || costmap->metadata.resolution <= 0.0 ||
    costmap->metadata.size_x == 0 || costmap->metadata.size_y == 0 ||
    costmap->data.empty())
  {
    return std::nullopt;
  }

  const auto start = nearestPathIndex(path);
  const std::string path_frame = pathFrame(path);
  const std::string costmap_frame = costmap->header.frame_id;
  if (!start || path_frame.empty() || costmap_frame.empty()) {
    return std::nullopt;
  }

  std::optional<geometry_msgs::msg::TransformStamped> costmap_from_path;
  if (path_frame != costmap_frame) {
    try {
      costmap_from_path = tf_buffer_->lookupTransform(
        costmap_frame, path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 5000,
        "Cannot transform path costs from %s to %s: %s",
        path_frame.c_str(), costmap_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  const double origin_yaw = tf2::getYaw(costmap->metadata.origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double resolution = static_cast<double>(costmap->metadata.resolution);
  const double sample_spacing = std::max(0.02, 0.5 * resolution);

  PathCost score;
  double cost_sum = 0.0;

  const auto addPointCost = [&](const geometry_msgs::msg::Point & path_point) {
      geometry_msgs::msg::Point point = path_point;
      if (costmap_from_path) {
        geometry_msgs::msg::PointStamped input;
        geometry_msgs::msg::PointStamped output;
        input.header.frame_id = path_frame;
        input.point = path_point;
        tf2::doTransform(input, output, *costmap_from_path);
        point = output.point;
      }

      const double dx = point.x - costmap->metadata.origin.position.x;
      const double dy = point.y - costmap->metadata.origin.position.y;
      const double local_x = cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;
      const int mx = static_cast<int>(std::floor(local_x / resolution));
      const int my = static_cast<int>(std::floor(local_y / resolution));

      if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(costmap->metadata.size_x) ||
        my >= static_cast<int>(costmap->metadata.size_y))
      {
        return;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * costmap->metadata.size_x +
        static_cast<std::size_t>(mx);
      if (index >= costmap->data.size()) {
        return;
      }

      const std::uint8_t raw_cost = costmap->data[index];
      if (raw_cost == 255u && !treat_unknown_as_obstacle_) {
        return;
      }

      const double cost = static_cast<double>(raw_cost);
      score.maximum = std::max(score.maximum, cost);
      cost_sum += cost;
      ++score.samples;
    };

  addPointCost(path.poses[*start].pose.position);
  double travelled = 0.0;

  for (std::size_t i = *start + 1;
    i < path.poses.size() && travelled < obstacle_check_distance_; ++i)
  {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const double distance_to_sample = std::min(
      segment_length, obstacle_check_distance_ - travelled);
    const std::size_t sample_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(distance_to_sample / sample_spacing)));

    for (std::size_t sample = 1; sample <= sample_count; ++sample) {
      const double along = distance_to_sample *
        static_cast<double>(sample) / static_cast<double>(sample_count);
      const double ratio = along / segment_length;
      geometry_msgs::msg::Point point;
      point.x = a.x + ratio * dx;
      point.y = a.y + ratio * dy;
      point.z = a.z + ratio * (b.z - a.z);
      addPointCost(point);
    }

    travelled += distance_to_sample;
  }

  if (score.samples == 0) {
    return std::nullopt;
  }

  score.mean = cost_sum / static_cast<double>(score.samples);
  return score;
}

std::optional<std::size_t> AcceptCandidatePath::nearestPathIndex(
  const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  const std::string frame = pathFrame(path);
  if (frame.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped path_from_base;
  try {
    path_from_base = tf_buffer_->lookupTransform(
      frame, robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 5000,
      "Cannot locate %s in path frame %s: %s",
      robot_base_frame_.c_str(), frame.c_str(), ex.what());
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

bool AcceptCandidatePath::isCandidateAcceptable(
  const nav_msgs::msg::Path & current_path,
  const nav_msgs::msg::Path & candidate_path) const
{
  const std::string current_frame = pathFrame(current_path);
  const std::string candidate_frame = pathFrame(candidate_path);
  if (current_frame.empty() || current_frame != candidate_frame) {
    RCLCPP_WARN(logger_, "Reject candidate: current and candidate path frames differ");
    return false;
  }

  const auto current_start = nearestPathIndex(current_path);
  const auto candidate_start = nearestPathIndex(candidate_path);
  if (!current_start || !candidate_start) {
    return false;
  }

  const double current_heading = pathHeading(
    current_path, *current_start, check_distance_);
  const double candidate_heading = pathHeading(
    candidate_path, *candidate_start, check_distance_);
  const double heading_change_deg =
    std::abs(angleDiff(candidate_heading, current_heading)) * 180.0 / M_PI;

  if (heading_change_deg > max_initial_angle_deg_) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: prefix heading changes %.1f deg > %.1f deg",
      heading_change_deg, max_initial_angle_deg_);
    return false;
  }

  const double current_len = pathLength(current_path, *current_start, check_distance_);
  const double candidate_len = pathLength(candidate_path, *candidate_start, check_distance_);
  if (candidate_len > current_len * max_length_ratio_) {
    RCLCPP_WARN(
      logger_,
      "Reject candidate: local length %.2f > %.2f * %.2f",
      candidate_len, current_len, max_length_ratio_);
    return false;
  }

  return beginningMatches(
    current_path, *current_start, candidate_path, *candidate_start);
}

bool AcceptCandidatePath::beginningMatches(
  const nav_msgs::msg::Path & current_path,
  std::size_t current_start,
  const nav_msgs::msg::Path & candidate_path,
  std::size_t candidate_start) const
{
  double travelled = 0.0;
  for (std::size_t i = candidate_start;
    i < candidate_path.poses.size() && travelled <= check_distance_; ++i)
  {
    if (i > candidate_start) {
      const auto & previous = candidate_path.poses[i - 1].pose.position;
      const auto & point = candidate_path.poses[i].pose.position;
      travelled += std::hypot(point.x - previous.x, point.y - previous.y);
    }

    const double deviation = distanceToPathSection(
      candidate_path.poses[i].pose.position,
      current_path, current_start, 2.0 * check_distance_);
    if (deviation > max_start_deviation_) {
      RCLCPP_WARN(
        logger_,
        "Reject candidate: prefix deviates %.2f m > %.2f m from current path",
        deviation, max_start_deviation_);
      return false;
    }
  }

  return true;
}

double AcceptCandidatePath::distanceToPathSection(
  const geometry_msgs::msg::Point & point,
  const nav_msgs::msg::Path & path,
  std::size_t start,
  double max_distance) const
{
  if (start >= path.poses.size()) {
    return std::numeric_limits<double>::max();
  }

  double min_distance = std::hypot(
    point.x - path.poses[start].pose.position.x,
    point.y - path.poses[start].pose.position.y);
  double travelled = 0.0;

  for (std::size_t i = start + 1;
    i < path.poses.size() && travelled <= max_distance; ++i)
  {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    travelled += std::sqrt(length_squared);

    double projection = 0.0;
    if (length_squared > std::numeric_limits<double>::epsilon()) {
      projection = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared;
      projection = std::clamp(projection, 0.0, 1.0);
    }

    const double closest_x = a.x + projection * dx;
    const double closest_y = a.y + projection * dy;
    min_distance = std::min(
      min_distance, std::hypot(point.x - closest_x, point.y - closest_y));
  }

  return min_distance;
}

double AcceptCandidatePath::pathLength(
  const nav_msgs::msg::Path & path,
  std::size_t start,
  double max_distance) const
{
  double total = 0.0;
  for (std::size_t i = start + 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
    if (total >= max_distance) {
      return max_distance;
    }
  }
  return total;
}

double AcceptCandidatePath::pathHeading(
  const nav_msgs::msg::Path & path,
  std::size_t start,
  double lookahead_distance) const
{
  if (start >= path.poses.size() - 1) {
    return 0.0;
  }

  const auto & origin = path.poses[start].pose.position;
  double travelled = 0.0;
  for (std::size_t i = start + 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & point = path.poses[i].pose.position;
    travelled += std::hypot(point.x - previous.x, point.y - previous.y);
    if (travelled >= lookahead_distance || i == path.poses.size() - 1) {
      return std::atan2(point.y - origin.y, point.x - origin.x);
    }
  }
  return 0.0;
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

  const auto & current_goal = current_path.poses.back().pose.position;
  const auto & candidate_goal = candidate_path.poses.back().pose.position;
  return std::hypot(
    candidate_goal.x - current_goal.x,
    candidate_goal.y - current_goal.y) > 0.25;
}

std::string AcceptCandidatePath::pathFrame(const nav_msgs::msg::Path & path) const
{
  if (!path.header.frame_id.empty()) {
    return path.header.frame_id;
  }
  return path.poses.empty() ? std::string{} : path.poses.front().header.frame_id;
}

void AcceptCandidatePath::outputPath(const nav_msgs::msg::Path & path)
{
  setOutput("output_path", path);
  accepted_path_pub_->publish(path);
}

}  // namespace path_planning

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<path_planning::AcceptCandidatePath>(
    "AcceptCandidatePath");

  factory.registerNodeType<path_planning::RestampGoal>(
    "RestampGoal");
}
