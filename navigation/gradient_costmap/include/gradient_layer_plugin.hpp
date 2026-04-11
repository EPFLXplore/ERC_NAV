#ifndef GRADIENT_LAYER__GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER__GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <string>
#include <vector>

namespace gradient_layer
{

class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  GradientLayer() = default;
  virtual ~GradientLayer() = default;

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override { return; }

  virtual void onFootprintChanged() override { return; }

  virtual bool isClearable() {return false;}

private:
  bool loadMapFromYaml(const std::string & yaml_path);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  bool use_map_file_{false};
  bool expand_update_bounds_{true};
  bool file_map_loaded_{false};
  double file_resolution_{0.0};
  double file_origin_x_{0.0};
  double file_origin_y_{0.0};
  int file_width_{0};
  int file_height_{0};
  bool file_flip_x_{false};
  bool file_flip_y_{false};
  std::vector<unsigned char> file_data_;
};

}  // namespace gradient_layer

#endif  // GRADIENT_LAYER__GRADIENT_LAYER_HPP_