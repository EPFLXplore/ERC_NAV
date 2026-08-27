#include "path_planning/accept_candidate_path.hpp"
#include "path_planning/accept_candidate_path_stupid.hpp"
#include "path_planning/restamp_goal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>

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

  // The node supplied on Nav2's BT blackboard is an auxiliary rclcpp node.
  // Arbitrary callbacks in its default callback group are not spun by the BT
  // action nodes, so give this subscription a group that this node services
  // explicitly at the beginning of every tick.
  costmap_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions costmap_options;
  costmap_options.callback_group = costmap_callback_group_;

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  costmap_qos.transient_local().reliable();
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_, costmap_qos,
    [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      const bool first_costmap = !costmap_;
      costmap_ = msg;
      if (first_costmap) {
        RCLCPP_INFO(
          logger_, "Received first Costmap on '%s' (%ux%u at %.3f m/cell)",
          costmap_topic_.c_str(), msg->metadata.size_x, msg->metadata.size_y,
          static_cast<double>(msg->metadata.resolution));
      }
    }, costmap_options);
  costmap_executor_.add_callback_group(
    costmap_callback_group_, node_->get_node_base_interface());

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
  check_cost_distance_ = declareDouble("check_cost_distance", check_cost_distance_);
  obstacle_cost_threshold_ = declareDouble(
    "obstacle_cost_threshold", obstacle_cost_threshold_);
  blocked_window_length_ = declareDouble(
    "blocked_window_length", blocked_window_length_);
  minimum_blocked_length_ = declareDouble(
    "minimum_blocked_length", minimum_blocked_length_);
  minimum_mean_cost_improvement_ = declareDouble(
    "minimum_mean_cost_improvement", minimum_mean_cost_improvement_);
  minimum_heading_improvement_deg_ = declareDouble(
    "minimum_heading_improvement_deg", minimum_heading_improvement_deg_);
  treat_unknown_as_obstacle_ = declareBool(
    "treat_unknown_as_obstacle", treat_unknown_as_obstacle_);
  costmap_topic_ = declareString("costmap_topic", costmap_topic_);
  accepted_path_topic_ = declareString("accepted_path_topic", accepted_path_topic_);

  check_distance_ = std::max(0.0, check_distance_);
  check_cost_distance_ = std::max(0.0, check_cost_distance_);
  obstacle_cost_threshold_ = std::clamp(obstacle_cost_threshold_, 0.0, 255.0);
  blocked_window_length_ = std::max(0.0, blocked_window_length_);
  minimum_blocked_length_ = std::max(0.0, minimum_blocked_length_);
  minimum_mean_cost_improvement_ = std::max(0.0, minimum_mean_cost_improvement_);
  minimum_heading_improvement_deg_ = std::max(0.0, minimum_heading_improvement_deg_);

  RCLCPP_INFO(
    logger_,
    "Params: check_distance=%.2f check_cost_distance=%.2f obstacle_cost_threshold=%.1f "
    "blocked_window_length=%.2f minimum_blocked_length=%.2f "
    "minimum_mean_cost_improvement=%.1f "
    "minimum_heading_improvement_deg=%.1f "
    "costmap_topic=%s accepted_path_topic=%s",
    check_distance_, check_cost_distance_, obstacle_cost_threshold_, blocked_window_length_,
    minimum_blocked_length_, minimum_mean_cost_improvement_, minimum_heading_improvement_deg_,
    costmap_topic_.c_str(), accepted_path_topic_.c_str());
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
  // Process the latest transient-local Costmap before evaluating either path.
  // KeepLast(1) bounds this work even though the global costmap publishes more
  // frequently than this BT node is ticked.
  costmap_executor_.spin_some();

  nav_msgs::msg::Path candidate_path;
  nav_msgs::msg::Path current_path;

  if (!getInput("candidate_path", candidate_path)) {
    RCLCPP_ERROR(logger_, "AcceptCandidatePath FAILURE: missing candidate_path input");
    return BT::NodeStatus::FAILURE;
  }

  const auto current_path_result = getInput("current_path", current_path);
  const bool has_current_path = static_cast<bool>(current_path_result);

  if (candidate_path.poses.size() < 2) {
    RCLCPP_ERROR(
      logger_, "AcceptCandidatePath FAILURE: candidate path has fewer than 2 poses");
    return BT::NodeStatus::FAILURE;
  }

  if (!has_current_path || current_path.poses.size() < 2) {
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: no usable current path");
    return BT::NodeStatus::SUCCESS;
  }

  if (goalChanged(current_path, candidate_path)) {
    outputPath(candidate_path);
    RCLCPP_INFO(logger_, "Accept candidate: goal changed");
    return BT::NodeStatus::SUCCESS;
  }

  const auto current_cost = scorePathAhead(current_path, "current");
  const auto candidate_cost = scorePathAhead(candidate_path, "candidate");
  if (!current_cost || !candidate_cost) {
    outputPath(current_path);
    RCLCPP_WARN(
      logger_,
      "No switch before Rule 1: cost score unavailable (current=%s, candidate=%s); "
      "keeping current path",
      current_cost ? "available" : "FAILED",
      candidate_cost ? "available" : "FAILED");
    return BT::NodeStatus::SUCCESS;
  }

  const bool current_blocked =
    current_cost->maximum_blocked_length >= minimum_blocked_length_;
  const bool candidate_blocked =
    candidate_cost->maximum_blocked_length >= minimum_blocked_length_;

  // One comparison line per replan. These are the filter's distance-weighted
  // mean costs over check_cost_distance, not Smac's internal search score.
  RCLCPP_INFO(
    logger_,
    "Path costs | current: mean=%.1f high_coverage=%.2f m blocked=%s | "
    "candidate: mean=%.1f high_coverage=%.2f m blocked=%s",
    current_cost->mean, current_cost->maximum_blocked_length,
    current_blocked ? "yes" : "no",
    candidate_cost->mean, candidate_cost->maximum_blocked_length,
    candidate_blocked ? "yes" : "no");

  // Rule 1: never replace the followed path with a blocked candidate.
  if (candidate_blocked) {
    outputPath(current_path);
    RCLCPP_WARN(
      logger_,
      "Rule 1 - reject blocked candidate: %.2f m high-cost coverage in %.2f m window "
      ">= %.0f (current %.2f m)",
      candidate_cost->maximum_blocked_length, blocked_window_length_, obstacle_cost_threshold_,
      current_cost->maximum_blocked_length);
    return BT::NodeStatus::SUCCESS;
  }

  // Rule 2: a clear detour immediately replaces a blocked current path.
  if (current_blocked) {
    outputPath(candidate_path);
    RCLCPP_WARN(
      logger_,
      "Rule 2 - accept clear obstacle detour: current high-cost coverage %.2f m, "
      "candidate mean %.1f",
      current_cost->maximum_blocked_length, candidate_cost->mean);
    return BT::NodeStatus::SUCCESS;
  }

  const double mean_improvement = current_cost->mean - candidate_cost->mean;

  // Rule 3: both paths are clear, so switch only for a meaningful cost gain.
  if (mean_improvement >= minimum_mean_cost_improvement_) {
    outputPath(candidate_path);
    RCLCPP_INFO(
      logger_, "Rule 3 - accept lower-cost candidate: mean improves %.1f (%.1f -> %.1f)",
      mean_improvement, current_cost->mean, candidate_cost->mean);
    return BT::NodeStatus::SUCCESS;
  }

  // Rule 4: heading decides only while the mean-cost difference is insignificant.
  const bool cost_difference_is_insignificant =
    std::abs(mean_improvement) < minimum_mean_cost_improvement_;
  std::optional<double> heading_improvement_deg;
  if (cost_difference_is_insignificant) {
    const auto current_heading_error = headingError(current_path);
    const auto candidate_heading_error = headingError(candidate_path);
    if (current_heading_error && candidate_heading_error) {
      heading_improvement_deg =
        (*current_heading_error - *candidate_heading_error) * 180.0 / M_PI;
      if (*heading_improvement_deg >= minimum_heading_improvement_deg_) {
        outputPath(candidate_path);
        RCLCPP_INFO(
          logger_,
          "Rule 4 - accept better-aligned candidate: heading error improves %.1f deg "
          "(%.1f -> %.1f deg)",
          *heading_improvement_deg, *current_heading_error * 180.0 / M_PI,
          *candidate_heading_error * 180.0 / M_PI);
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  // Rule 5: no meaningful improvement; preserve the controller's current path.
  outputPath(current_path);
  if (!cost_difference_is_insignificant) {
    RCLCPP_INFO(
      logger_,
      "Rule 5 - keep current path: candidate mean cost is significantly worse "
      "(%.1f -> %.1f)",
      current_cost->mean, candidate_cost->mean);
  } else if (!heading_improvement_deg) {
    RCLCPP_WARN(
      logger_,
      "Rule 5 - keep current path: cost difference %.1f is insignificant, but heading "
      "error could not be calculated",
      mean_improvement);
  } else {
    RCLCPP_INFO(
      logger_,
      "Rule 5 - keep current path: cost difference %.1f is insignificant and heading "
      "improves only %.1f deg (< %.1f deg)",
      mean_improvement, *heading_improvement_deg, minimum_heading_improvement_deg_);
  }
  return BT::NodeStatus::SUCCESS;
}

std::optional<AcceptCandidatePath::PathCost> AcceptCandidatePath::scorePathAhead(
  const nav_msgs::msg::Path & path,
  const char * path_label) const
{
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = costmap_;
  }

  if (!costmap) {
    RCLCPP_WARN(
      logger_,
      "Cannot score %s path: no Costmap received (configured='%s', resolved='%s', "
      "publishers=%zu)",
      path_label, costmap_topic_.c_str(), costmap_sub_->get_topic_name(),
      costmap_sub_->get_publisher_count());
    return std::nullopt;
  }

  if (costmap->metadata.resolution <= 0.0 || costmap->metadata.size_x == 0 ||
    costmap->metadata.size_y == 0 || costmap->data.empty())
  {
    RCLCPP_WARN(
      logger_,
      "Cannot score %s path: invalid costmap (resolution=%.3f, size=%ux%u, data=%zu)",
      path_label, static_cast<double>(costmap->metadata.resolution),
      costmap->metadata.size_x, costmap->metadata.size_y, costmap->data.size());
    return std::nullopt;
  }

  const auto start = nearestPathIndex(path);
  const std::string path_frame = pathFrame(path);
  const std::string costmap_frame = costmap->header.frame_id;
  if (!start) {
    RCLCPP_WARN(
      logger_, "Cannot score %s path: rover-to-path transform or nearest point unavailable",
      path_label);
    return std::nullopt;
  }
  if (*start >= path.poses.size() - 1) {
    RCLCPP_WARN(
      logger_,
      "Cannot score %s path: nearest pose is the final pose (%zu of %zu), "
      "so there is no path ahead",
      path_label, *start, path.poses.size());
    return std::nullopt;
  }
  if (path_frame.empty() || costmap_frame.empty()) {
    RCLCPP_WARN(
      logger_, "Cannot score %s path: empty frame (path='%s', costmap='%s')",
      path_label, path_frame.c_str(), costmap_frame.c_str());
    return std::nullopt;
  }

  std::optional<geometry_msgs::msg::TransformStamped> costmap_from_path;
  if (path_frame != costmap_frame) {
    try {
      costmap_from_path = tf_buffer_->lookupTransform(
        costmap_frame, path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_, "Cannot score %s path: transform from %s to %s failed: %s",
        path_label, path_frame.c_str(), costmap_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  const double origin_yaw = tf2::getYaw(costmap->metadata.origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double resolution = static_cast<double>(costmap->metadata.resolution);
  const double sample_spacing = std::max(0.02, 0.5 * resolution);

  PathCost score;
  double weighted_cost_sum = 0.0;
  std::deque<std::pair<double, double>> blocked_window;
  double window_distance = 0.0;
  double window_blocked_length = 0.0;
  std::size_t attempted_samples = 0;
  std::size_t unknown_samples = 0;
  std::size_t outside_samples = 0;
  std::size_t invalid_index_samples = 0;

  const auto pointCost = [&](const geometry_msgs::msg::Point & path_point)
    -> std::optional<double>
    {
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
        ++outside_samples;
        return std::nullopt;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * costmap->metadata.size_x +
        static_cast<std::size_t>(mx);
      if (index >= costmap->data.size()) {
        ++invalid_index_samples;
        return std::nullopt;
      }

      const std::uint8_t raw_cost = costmap->data[index];
      if (raw_cost == 255u && !treat_unknown_as_obstacle_) {
        ++unknown_samples;
        return std::nullopt;
      }

      return static_cast<double>(raw_cost);
    };

  double travelled = 0.0;

  for (std::size_t i = *start + 1;
    i < path.poses.size() && travelled < check_cost_distance_; ++i)
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
      segment_length, check_cost_distance_ - travelled);
    const std::size_t sample_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(distance_to_sample / sample_spacing)));
    const double sample_length = distance_to_sample / static_cast<double>(sample_count);

    for (std::size_t sample = 1; sample <= sample_count; ++sample) {
      const double along = distance_to_sample *
        static_cast<double>(sample) / static_cast<double>(sample_count);
      const double ratio = along / segment_length;
      geometry_msgs::msg::Point point;
      point.x = a.x + ratio * dx;
      point.y = a.y + ratio * dy;
      point.z = a.z + ratio * (b.z - a.z);
      ++attempted_samples;
      const auto cost = pointCost(point);
      if (!cost) {
        blocked_window.clear();
        window_distance = 0.0;
        window_blocked_length = 0.0;
        continue;
      }

      weighted_cost_sum += *cost * sample_length;
      score.sampled_distance += sample_length;
      ++score.samples;

      const double blocked_length =
        *cost >= obstacle_cost_threshold_ ? sample_length : 0.0;
      blocked_window.emplace_back(sample_length, blocked_length);
      window_distance += sample_length;
      window_blocked_length += blocked_length;

      // Measure high-cost centerline coverage in a local sliding window. This
      // catches dense clusters without letting one isolated cell reject a path.
      while (!blocked_window.empty() && window_distance > blocked_window_length_) {
        auto & oldest = blocked_window.front();
        const double removed_distance = std::min(
          oldest.first, window_distance - blocked_window_length_);
        const double removed_blocked = oldest.first > 0.0 ?
          oldest.second * removed_distance / oldest.first : 0.0;
        oldest.first -= removed_distance;
        oldest.second -= removed_blocked;
        window_distance -= removed_distance;
        window_blocked_length -= removed_blocked;
        if (oldest.first <= std::numeric_limits<double>::epsilon()) {
          blocked_window.pop_front();
        }
      }

      score.maximum_blocked_length = std::max(
        score.maximum_blocked_length, window_blocked_length);
    }

    travelled += distance_to_sample;
  }

  if (score.samples == 0 || score.sampled_distance <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Cannot score %s path: 0 valid samples out of %zu "
      "(unknown=%zu, outside_costmap=%zu, invalid_index=%zu, start=%zu/%zu, "
      "path_frame='%s', costmap_frame='%s')",
      path_label, attempted_samples, unknown_samples, outside_samples,
      invalid_index_samples, *start, path.poses.size(), path_frame.c_str(),
      costmap_frame.c_str());
    return std::nullopt;
  }

  score.mean = weighted_cost_sum / score.sampled_distance;
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

std::optional<double> AcceptCandidatePath::headingError(
  const nav_msgs::msg::Path & path) const
{
  const std::string frame = pathFrame(path);
  const auto start = nearestPathIndex(path);
  if (frame.empty() || !start || *start >= path.poses.size() - 1) {
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped path_from_base;
  try {
    path_from_base = tf_buffer_->lookupTransform(
      frame, robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 5000,
      "Cannot calculate heading error in path frame %s: %s",
      frame.c_str(), ex.what());
    return std::nullopt;
  }

  const double robot_heading = tf2::getYaw(path_from_base.transform.rotation);
  const double prefix_heading = pathHeading(path, *start, check_distance_);
  return std::abs(angleDiff(prefix_heading, robot_heading));
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

  factory.registerNodeType<path_planning::AcceptCandidatePathStupid>(
    "AcceptCandidatePathStupid");

  factory.registerNodeType<path_planning::RestampGoal>(
    "RestampGoal");
}
