#ifndef GRADIENT_LAYER__GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER__GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace gradient_layer
{

// Lightweight 2D rigid transform cache: p_dst = R(yaw) * p_src + t
// Used per update cycle so the per-cell updateCosts loop does not call TF.
struct PlanarTransform2D
{
  bool   valid{false};
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  double tx{0.0};
  double ty{0.0};

  inline void apply(double x_src, double y_src, double & x_dst, double & y_dst) const
  {
    x_dst = cos_yaw * x_src - sin_yaw * y_src + tx;
    y_dst = sin_yaw * x_src + cos_yaw * y_src + ty;
  }
};

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

  virtual void reset() override
  {
    has_previous_bounds_ = false;
    current_ = true;
  }

  virtual void onFootprintChanged() override { return; }

  virtual bool isClearable() {return false;}

private:
  bool loadMapFromYaml(const std::string & yaml_path);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // Refresh the cached master<->grid planar transforms from the TF buffer.
  // Returns true if both transforms are valid and usable this cycle.
  bool refreshGridTransforms(const std::string & master_frame_id);

  void resizePersistenceIfNeeded(const nav2_costmap_2d::Costmap2D & master_grid);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr active_map_;  // snapshot used within a single update cycle
  std::mutex map_mutex_;  // guards latest_map_ against concurrent cb/updateBounds

  bool use_map_file_{false};
  bool expand_update_bounds_{true};
  bool footprint_clearing_enabled_{false};
  // When true, a zero from the incoming grid forces FREE_SPACE even on cells
  // this layer never wrote, so live free space overrides earlier layers
  // (notably a stale file-backed map) inside the incoming grid's extent.
  bool authoritative_clear_{false};
  bool persistent_patch_{false};
  bool file_map_loaded_{false};
  double file_resolution_{0.0};
  double file_origin_x_{0.0};
  double file_origin_y_{0.0};
  int file_width_{0};
  int file_height_{0};
  bool file_flip_x_{false};
  bool file_flip_y_{false};
  std::vector<unsigned char> file_data_;

  bool has_previous_bounds_{false};
  double previous_min_x_{0.0};
  double previous_min_y_{0.0};
  double previous_max_x_{0.0};
  double previous_max_y_{0.0};

  std::vector<unsigned char> persistent_costs_;
  std::vector<int64_t> persistent_stamps_ms_;
  std::vector<unsigned char> gradient_owned_cells_;
  std::vector<unsigned char> gradient_touched_cells_;
  unsigned int persistent_size_x_{0};
  unsigned int persistent_size_y_{0};
  double persistence_timeout_{8.0};
  unsigned char persistence_min_cost_{1};

  // Cached per-cycle planar transforms between the incoming grid frame and
  // the master costmap's global frame.  master_from_grid_ is used to expand
  // updateBounds; grid_from_master_ is used in the updateCosts inner loop.
  PlanarTransform2D master_from_grid_;
  PlanarTransform2D grid_from_master_;
  double transform_tolerance_{0.2};

  // Robot footprint transformed into the master costmap frame for this
  // update cycle. Cells inside it are forced to FREE_SPACE when footprint
  // clearing is enabled.
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  rclcpp::Logger tf_logger_{rclcpp::get_logger("gradient_layer")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace gradient_layer

#endif  // GRADIENT_LAYER__GRADIENT_LAYER_HPP_
