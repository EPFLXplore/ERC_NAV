#ifndef PATH_PLANNING__ACCEPT_CANDIDATE_PATH_STUPID_HPP_
#define PATH_PLANNING__ACCEPT_CANDIDATE_PATH_STUPID_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace path_planning
{

// Deliberately simple alternative filter kept for experimentation. It ignores
// all costmap data and accepts a candidate only when its prefix heading differs
// from the current prefix heading by at least the configured angle.
class AcceptCandidatePathStupid : public BT::SyncActionNode
{
public:
  AcceptCandidatePathStupid(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::optional<std::size_t> nearestPathIndex(const nav_msgs::msg::Path & path) const;
  std::optional<double> prefixHeading(const nav_msgs::msg::Path & path) const;
  std::string pathFrame(const nav_msgs::msg::Path & path) const;
  double normalizeAngle(double angle) const;
  void outputPath(const nav_msgs::msg::Path & path);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string robot_base_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("AcceptCandidatePathStupid")};

  double check_distance_{1.5};
  double minimum_heading_change_deg_{30.0};
};

}  // namespace path_planning

#endif  // PATH_PLANNING__ACCEPT_CANDIDATE_PATH_STUPID_HPP_
