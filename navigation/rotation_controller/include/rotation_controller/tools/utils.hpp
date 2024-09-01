

#ifndef ROTATION_CONTROLLER__TOOLS__UTILS_HPP_
#define ROTATION_CONTROLLER__TOOLS__UTILS_HPP_

#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rotation_controller::utils
{

/**
* @brief get the current pose of the robot
* @param goal_checker goal checker to get tolerances
* @param robot robot pose
* @param goal goal pose
* @return bool Whether the robot is in the distance tolerance ignoring rotation and speed
*/
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & goal)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal.position.x;
    auto dy = robot.position.y - goal.position.y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

}  // namespace rotation_controller::utils

#endif  // ROTATION_CONTROLLER__TOOLS__UTILS_HPP_