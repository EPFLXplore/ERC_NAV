#include "gradient_layer_plugin.hpp"

namespace gradient_layer
{

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("/occupancy_map_local"));
  
  std::string topic;
  node->get_parameter(name_ + ".topic", topic);

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(10),
    std::bind(&GradientLayer::mapCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(node->get_logger(), "GradientLayer initialized, subscribing to %s", topic.c_str());
}

void GradientLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!latest_map_) return;

  // Expand bounds to cover the entire map
  *min_x = std::min(*min_x, latest_map_->info.origin.position.x);
  *min_y = std::min(*min_y, latest_map_->info.origin.position.y);
  *max_x = std::max(*max_x, latest_map_->info.origin.position.x + 
           latest_map_->info.width * latest_map_->info.resolution);
  *max_y = std::max(*max_y, latest_map_->info.origin.position.y + 
           latest_map_->info.height * latest_map_->info.resolution);
}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!latest_map_) return;

  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      int map_x = (int)((wx - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
      int map_y = (int)((wy - latest_map_->info.origin.position.y) / latest_map_->info.resolution);

      if (map_x < 0 || map_y < 0 || 
          map_x >= (int)latest_map_->info.width || 
          map_y >= (int)latest_map_->info.height)
        continue;

      int index = map_y * latest_map_->info.width + map_x;
      int8_t cost = latest_map_->data[index];

      if (cost < 0) continue; // unknown

      // Scale 0-100 occupancy to 0-254 costmap range
      unsigned char scaled_cost = (unsigned char)((cost * 254) / 100);
      master_grid.setCost(i, j, scaled_cost);
    }
  }
}

}  // namespace gradient_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_layer::GradientLayer, nav2_costmap_2d::Layer)