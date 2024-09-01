#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "rotation_controller/rotation_controller.hpp"
#include "rotation_controller/tools/utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace rotation_controller
{

RotationController::RotationController()
: lp_loader_("nav2_core", "nav2_core::Controller"),
  primary_controller_(nullptr),
  path_updated_(false),
  rotation_active_(false)
{
}

void RotationController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::string primary_controller;
  double control_frequency;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_dist_threshold", rclcpp::ParameterValue(0.785));  // 45 deg
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_sampling_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".primary_controller", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_goal_heading", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".angular_dist_threshold", angular_dist_threshold_);
  node->get_parameter(plugin_name_ + ".forward_sampling_distance", forward_sampling_distance_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".simulate_ahead_time", simulate_ahead_time_);

  primary_controller = node->get_parameter(plugin_name_ + ".primary_controller").as_string();
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  node->get_parameter(plugin_name_ + ".rotate_to_goal_heading", rotate_to_goal_heading_);

  try {
    primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
    RCLCPP_INFO(
      logger_, "Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    return;
  }

  primary_controller_->configure(parent, name, tf, costmap_ros);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());
}

void RotationController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "rotation_controller::RotationController",
    plugin_name_.c_str());

  primary_controller_->activate();

  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RotationController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RotationController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "rotation_controller::RotationController",
    plugin_name_.c_str());

  primary_controller_->deactivate();

  dyn_params_handler_.reset();
}

void RotationController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "rotation_controller::RotationController",
    plugin_name_.c_str());

  primary_controller_->cleanup();
  primary_controller_.reset();
}

geometry_msgs::msg::TwistStamped RotationController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Rotate to goal heading when in goal xy tolerance
  if (rotate_to_goal_heading_) {
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    try {
      geometry_msgs::msg::PoseStamped sampled_pt_goal = getSampledPathGoal();

      if (!nav2_util::transformPoseInTargetFrame(
          sampled_pt_goal, sampled_pt_goal, *tf_,
          pose.header.frame_id))
      {
        throw std::runtime_error("Failed to transform pose to base frame!");
      }

      if (utils::withinPositionGoalTolerance(
          goal_checker,
          pose.pose,
          sampled_pt_goal.pose))
      {
        double pose_yaw = tf2::getYaw(pose.pose.orientation);
        double goal_yaw = tf2::getYaw(sampled_pt_goal.pose.orientation);

        double angular_distance_to_heading = angles::shortest_angular_distance(pose_yaw, goal_yaw);

        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_INFO(
        logger_,
        "Rotation Controller was unable to find a goal point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
    }
  }

  if (rotation_active_) {
    double pose_yaw = tf2::getYaw(pose.pose.orientation);
    double angular_distance_to_heading = angles::shortest_angular_distance(pose_yaw, initial_rotation_goal_yaw_);

    if (fabs(angular_distance_to_heading) > angular_dist_threshold_) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation in progress, ignoring path updates until rotation is complete...");
      return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
    } else {
      RCLCPP_DEBUG(
        logger_,
        "Rotation complete, resuming path tracking...");
      rotation_active_ = false;
      path_updated_ = false;
    }
  }

  if (path_updated_) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    std::lock_guard<std::mutex> lock_reinit(mutex_);
    try {
      geometry_msgs::msg::Pose sampled_pt_base = transformPoseToBaseFrame(getSampledPathPt());

      double angular_distance_to_heading =
        std::atan2(sampled_pt_base.position.y, sampled_pt_base.position.x);
      if (fabs(angular_distance_to_heading) > angular_dist_threshold_) {
        RCLCPP_DEBUG(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        initial_rotation_goal_yaw_ = tf2::getYaw(sampled_pt_base.orientation);
        rotation_active_ = true;
        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      } else {
        RCLCPP_DEBUG(
          logger_,
          "Robot is at the new path's rough heading, passing to controller");
        path_updated_ = false;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Controller was unable to find a sampling point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
      path_updated_ = false;
    }
  }

  // If at this point, use the primary controller to path track
  return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
}

geometry_msgs::msg::PoseStamped RotationController::getSampledPathPt()
{
  if (current_path_.poses.size() < 2) {
    throw nav2_core::PlannerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  double dx, dy;

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.poses.size(); i++) {
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;

    if (hypot(dx, dy) > forward_sampling_distance_) {
      return current_path_.poses[i];
    }
  }

  // If all points are closer than required sampling distance,
  // use last point
  return current_path_.poses.back();
}

geometry_msgs::msg::PoseStamped RotationController::getSampledPathGoal()
{
  if (current_path_.poses.empty()) {
    throw nav2_core::PlannerException("Path is too short to find a valid goal point.");
  }

  return current_path_.poses.back();
}

geometry_msgs::msg::Pose RotationController::transformPoseToBaseFrame(
  const geometry_msgs::msg::PoseStamped & pt)
{
  geometry_msgs::msg::PoseStamped pt_tf;
  nav2_util::transformPoseInTargetFrame(pt, pt_tf, *tf_, costmap_ros_->getBaseFrameID());

  return pt_tf.pose;
}

geometry_msgs::msg::TwistStamped RotationController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  double desired_ang_velocity = sqrt(2 * max_angular_accel_ * fabs(angular_distance_to_heading));
  desired_ang_velocity = std::min(rotate_to_heading_angular_vel_, desired_ang_velocity);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.angular.z = desired_ang_velocity * (angular_distance_to_heading > 0 ? 1.0 : -1.0);

  return cmd_vel;
}

void RotationController::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  path_updated_ = true;
}

void RotationController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

rcl_interfaces::msg::SetParametersResult
RotationController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto node = node_.lock();
  rcl_interfaces::msg::SetParametersResult result;

  if (!node) {
    result.successful = false;
    return result;
  }

  result.successful = true;
  for (auto & param : parameters) {
    RCLCPP_INFO(
      logger_,
      "Dynamic reconfigure: Setting %s to %s",
      param.get_name().c_str(), param.value_to_string().c_str());

    if (param.get_name() == plugin_name_ + ".angular_dist_threshold" &&
      param.get_type() == ParameterType::PARAMETER_DOUBLE)
    {
      angular_dist_threshold_ = param.as_double();
    }

    if (param.get_name() == plugin_name_ + ".forward_sampling_distance" &&
      param.get_type() == ParameterType::PARAMETER_DOUBLE)
    {
      forward_sampling_distance_ = param.as_double();
    }

    if (param.get_name() == plugin_name_ + ".rotate_to_heading_angular_vel" &&
      param.get_type() == ParameterType::PARAMETER_DOUBLE)
    {
      rotate_to_heading_angular_vel_ = param.as_double();
    }

    if (param.get_name() == plugin_name_ + ".max_angular_accel" &&
      param.get_type() == ParameterType::PARAMETER_DOUBLE)
    {
      max_angular_accel_ = param.as_double();
    }

    if (param.get_name() == plugin_name_ + ".simulate_ahead_time" &&
      param.get_type() == ParameterType::PARAMETER_DOUBLE)
    {
      simulate_ahead_time_ = param.as_double();
    }

    if (param.get_name() == plugin_name_ + ".rotate_to_goal_heading" &&
      param.get_type() == ParameterType::PARAMETER_BOOL)
    {
      rotate_to_goal_heading_ = param.as_bool();
    }
  }

  return result;
}

}  // namespace rotation_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rotation_controller::RotationController, nav2_core::Controller)
