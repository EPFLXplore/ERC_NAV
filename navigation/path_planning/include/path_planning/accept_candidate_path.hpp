#ifndef NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_
#define NAV2_PATH_STABILITY_BT__ACCEPT_CANDIDATE_PATH_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2_ros/buffer.h"

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
  struct PathCost
  {
    double mean{0.0};
    double maximum_blocked_length{0.0};
    double sampled_distance{0.0};
    std::size_t samples{0};
  };

  void declareParameters();

  std::optional<PathCost> scorePathAhead(
    const nav_msgs::msg::Path & path,
    const char * path_label) const;
  std::optional<std::size_t> nearestPathIndex(const nav_msgs::msg::Path & path) const;

  std::optional<double> headingError(const nav_msgs::msg::Path & path) const;

  double pathHeading(
    const nav_msgs::msg::Path & path,
    std::size_t start,
    double lookahead_distance) const;

  double normalizeAngle(double angle) const;
  double angleDiff(double a, double b) const;

  bool goalChanged(
    const nav_msgs::msg::Path & current_path,
    const nav_msgs::msg::Path & candidate_path) const;

  std::string pathFrame(const nav_msgs::msg::Path & path) const;
  void outputPath(const nav_msgs::msg::Path & path);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string robot_base_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("AcceptCandidatePath")};

  rclcpp::CallbackGroup::SharedPtr costmap_callback_group_;
  rclcpp::executors::SingleThreadedExecutor costmap_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr accepted_path_pub_;
  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap_;

  // Tunables, loaded from ROS parameters on the bt_navigator node
  // (accept_candidate_path.*).
  double check_distance_{1.5};
  double check_cost_distance_{3.0};
  double obstacle_cost_threshold_{220.0};
  double blocked_window_length_{0.25};
  double minimum_blocked_length_{0.15};
  double minimum_mean_cost_improvement_{10.0};
  double minimum_heading_improvement_deg_{5.0};
  bool treat_unknown_as_obstacle_{false};
  std::string costmap_topic_{"/global_costmap/costmap_raw"};
  std::string accepted_path_topic_{"accepted_plan"};
};

}  // namespace path_planning

#endif
