#include "gradient_layer_plugin.hpp"

#include "nav2_costmap_2d/cost_values.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string stripQuotes(const std::string & s)
{
  if (s.size() >= 2 &&
      ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

std::string dirname(const std::string & path)
{
  const auto pos = path.find_last_of('/');
  if (pos == std::string::npos) {
    return ".";
  }
  if (pos == 0) {
    return "/";
  }
  return path.substr(0, pos);
}

std::string joinPath(const std::string & base, const std::string & rel)
{
  if (!rel.empty() && rel[0] == '/') {
    return rel;
  }
  if (base.empty() || base == ".") {
    return rel;
  }
  if (base.back() == '/') {
    return base + rel;
  }
  return base + "/" + rel;
}

bool readTokenSkippingComments(std::istream & in, std::string & token)
{
  token.clear();
  while (in >> token) {
    if (!token.empty() && token[0] == '#') {
      in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    return true;
  }
  return false;
}

}  // namespace

namespace gradient_layer
{

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  tf_logger_ = node->get_logger();
  clock_     = node->get_clock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("/occupancy_map_local_inflated"));
  declareParameter("use_map_file", rclcpp::ParameterValue(false));
  declareParameter("map_yaml_filename", rclcpp::ParameterValue(""));
  declareParameter("map_flip_x", rclcpp::ParameterValue(false));
  declareParameter("map_flip_y", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("map_subscribe_reliable", rclcpp::ParameterValue(true));
  declareParameter("expand_update_bounds", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("persistent_patch", rclcpp::ParameterValue(false));
  declareParameter("persistence_timeout", rclcpp::ParameterValue(8.0));
  declareParameter("persistence_min_cost", rclcpp::ParameterValue(1));

  std::string topic;
  std::string map_yaml_filename;
  bool map_subscribe_transient_local{true};
  bool map_subscribe_reliable{true};
  node->get_parameter(name_ + ".topic", topic);
  node->get_parameter(name_ + ".use_map_file", use_map_file_);
  node->get_parameter(name_ + ".map_yaml_filename", map_yaml_filename);
  node->get_parameter(name_ + ".map_flip_x", file_flip_x_);
  node->get_parameter(name_ + ".map_flip_y", file_flip_y_);
  node->get_parameter(name_ + ".map_subscribe_transient_local", map_subscribe_transient_local);
  node->get_parameter(name_ + ".map_subscribe_reliable", map_subscribe_reliable);
  node->get_parameter(name_ + ".expand_update_bounds", expand_update_bounds_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".persistent_patch", persistent_patch_);
  node->get_parameter(name_ + ".persistence_timeout", persistence_timeout_);
  int persistence_min_cost_param{1};
  node->get_parameter(name_ + ".persistence_min_cost", persistence_min_cost_param);
  persistence_min_cost_ = static_cast<unsigned char>(std::clamp(persistence_min_cost_param, 0, 252));

  if (use_map_file_) {
    file_map_loaded_ = loadMapFromYaml(map_yaml_filename);
    if (!file_map_loaded_) {
      RCLCPP_ERROR(node->get_logger(), "GradientLayer failed to load file map from %s", map_yaml_filename.c_str());
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "GradientLayer loaded map file %s (%dx%d @ %.3f m/px, flip_x=%s, flip_y=%s)",
        map_yaml_filename.c_str(),
        file_width_,
        file_height_,
        file_resolution_,
        file_flip_x_ ? "true" : "false",
        file_flip_y_ ? "true" : "false");
    }
    current_ = true;
    return;
  }

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  if (map_subscribe_reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  if (map_subscribe_transient_local) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, qos,
    std::bind(&GradientLayer::mapCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "GradientLayer initialized, subscribing to %s (transient_local=%s, reliable=%s)",
    topic.c_str(),
    map_subscribe_transient_local ? "true" : "false",
    map_subscribe_reliable ? "true" : "false");
}

bool GradientLayer::loadMapFromYaml(const std::string & yaml_path)
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }

  if (yaml_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "map_yaml_filename is empty while use_map_file=true");
    return false;
  }

  std::ifstream yaml_in(yaml_path);
  if (!yaml_in.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Could not open YAML map file: %s", yaml_path.c_str());
    return false;
  }

  std::string image_name;
  std::string line;
  while (std::getline(yaml_in, line)) {
    auto cleaned = trim(line);
    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }
    const auto colon = cleaned.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    auto key = trim(cleaned.substr(0, colon));
    auto val = trim(cleaned.substr(colon + 1));

    if (key == "image") {
      image_name = stripQuotes(val);
    } else if (key == "resolution") {
      file_resolution_ = std::stod(val);
    } else if (key == "origin") {
      // origin: [x, y, yaw]
      const auto lb = val.find('[');
      const auto rb = val.find(']');
      if (lb != std::string::npos && rb != std::string::npos && rb > lb + 1) {
        std::string inside = val.substr(lb + 1, rb - lb - 1);
        std::replace(inside.begin(), inside.end(), ',', ' ');
        std::istringstream iss(inside);
        iss >> file_origin_x_ >> file_origin_y_;
      }
    }
  }

  if (image_name.empty() || file_resolution_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid map YAML contents in: %s", yaml_path.c_str());
    return false;
  }

  const std::string pgm_path = joinPath(dirname(yaml_path), image_name);
  std::ifstream pgm_in(pgm_path, std::ios::binary);
  if (!pgm_in.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Could not open PGM map image: %s", pgm_path.c_str());
    return false;
  }

  std::string magic;
  if (!readTokenSkippingComments(pgm_in, magic) || magic != "P5") {
    RCLCPP_ERROR(node->get_logger(), "Unsupported PGM format in %s (expected P5)", pgm_path.c_str());
    return false;
  }

  std::string tok;
  if (!readTokenSkippingComments(pgm_in, tok)) {
    return false;
  }
  file_width_ = std::stoi(tok);

  if (!readTokenSkippingComments(pgm_in, tok)) {
    return false;
  }
  file_height_ = std::stoi(tok);

  if (!readTokenSkippingComments(pgm_in, tok)) {
    return false;
  }
  const int maxval = std::stoi(tok);
  if (maxval <= 0 || maxval > 255) {
    RCLCPP_ERROR(node->get_logger(), "Unsupported PGM maxval %d in %s", maxval, pgm_path.c_str());
    return false;
  }

  pgm_in.get();

  const size_t n = static_cast<size_t>(file_width_) * static_cast<size_t>(file_height_);
  file_data_.assign(n, 0);
  pgm_in.read(reinterpret_cast<char *>(file_data_.data()), static_cast<std::streamsize>(n));
  if (pgm_in.gcount() != static_cast<std::streamsize>(n)) {
    RCLCPP_ERROR(node->get_logger(), "Unexpected EOF while reading PGM data from %s", pgm_path.c_str());
    return false;
  }

  return true;
}

void GradientLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  latest_map_ = msg;
}

bool GradientLayer::refreshGridTransforms(const std::string & master_frame_id)
{
  master_from_grid_.valid = false;
  grid_from_master_.valid = false;

  if (!active_map_ || !tf_) {
    return false;
  }

  const std::string & grid_frame_id = active_map_->header.frame_id;
  if (grid_frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      tf_logger_, *clock_, 5000,
      "GradientLayer: incoming OccupancyGrid has empty frame_id, skipping update.");
    return false;
  }

  // Fast-path: same frame -> identity transform (avoids a TF lookup).
  if (grid_frame_id == master_frame_id) {
    master_from_grid_ = PlanarTransform2D{true, 1.0, 0.0, 0.0, 0.0};
    grid_from_master_ = PlanarTransform2D{true, 1.0, 0.0, 0.0, 0.0};
    return true;
  }

  geometry_msgs::msg::TransformStamped t_master_from_grid;
  geometry_msgs::msg::TransformStamped t_grid_from_master;
  try {
    // Use the latest available transform (TimePointZero) to avoid waiting and
    // to be robust to small timestamp mismatches between the grid and TF.
    t_master_from_grid = tf_->lookupTransform(
      master_frame_id, grid_frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    t_grid_from_master = tf_->lookupTransform(
      grid_frame_id, master_frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      tf_logger_, *clock_, 2000,
      "GradientLayer: TF lookup '%s' <- '%s' failed: %s",
      master_frame_id.c_str(), grid_frame_id.c_str(), ex.what());
    return false;
  }

  auto to_planar = [](const geometry_msgs::msg::TransformStamped & t) {
    tf2::Quaternion q(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    PlanarTransform2D p;
    p.valid   = true;
    p.cos_yaw = std::cos(yaw);
    p.sin_yaw = std::sin(yaw);
    p.tx      = t.transform.translation.x;
    p.ty      = t.transform.translation.y;
    return p;
  };

  master_from_grid_ = to_planar(t_master_from_grid);
  grid_from_master_ = to_planar(t_grid_from_master);
  return true;
}

void GradientLayer::resizePersistenceIfNeeded(const nav2_costmap_2d::Costmap2D & master_grid)
{
  const auto size_x = master_grid.getSizeInCellsX();
  const auto size_y = master_grid.getSizeInCellsY();
  if (size_x == persistent_size_x_ && size_y == persistent_size_y_) {
    return;
  }

  persistent_size_x_ = size_x;
  persistent_size_y_ = size_y;
  const size_t n = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  persistent_costs_.assign(n, nav2_costmap_2d::FREE_SPACE);
  persistent_stamps_ms_.assign(n, 0);
  gradient_owned_cells_.assign(n, 0);
  gradient_touched_cells_.assign(n, 0);
}

void GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (use_map_file_) {
    if (!file_map_loaded_) {
      return;
    }

    *min_x = std::min(*min_x, file_origin_x_);
    *min_y = std::min(*min_y, file_origin_y_);
    *max_x = std::max(*max_x, file_origin_x_ + file_width_ * file_resolution_);
    *max_y = std::max(*max_y, file_origin_y_ + file_height_ * file_resolution_);
    return;
  }

  // Snapshot the latest map so updateBounds and updateCosts see the same grid
  // even if a new message arrives between the two calls.
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    active_map_ = latest_map_;
  }

  if (!active_map_) return;

  // Resolve the master costmap's global frame so we can transform the grid
  // into it.  Without layered_costmap_ we cannot know the target frame.
  std::string master_frame_id;
  if (layered_costmap_) {
    master_frame_id = layered_costmap_->getGlobalFrameID();
  }
  if (master_frame_id.empty()) {
    return;
  }

  if (!refreshGridTransforms(master_frame_id)) {
    // Leave active_map_ populated so updateCosts can no-op consistently.
    return;
  }

  if (!expand_update_bounds_) return;

  // Expand bounds by transforming the 4 corners of the incoming grid into the
  // master frame.  This correctly handles arbitrary yaw between frames.
  const double res = active_map_->info.resolution;
  const double ox  = active_map_->info.origin.position.x;
  const double oy  = active_map_->info.origin.position.y;
  const double w_m = active_map_->info.width  * res;
  const double h_m = active_map_->info.height * res;

  const double corners_grid[4][2] = {
    {ox,       oy      },
    {ox + w_m, oy      },
    {ox,       oy + h_m},
    {ox + w_m, oy + h_m}
  };

  double current_min_x = std::numeric_limits<double>::max();
  double current_min_y = std::numeric_limits<double>::max();
  double current_max_x = std::numeric_limits<double>::lowest();
  double current_max_y = std::numeric_limits<double>::lowest();

  for (const auto & c : corners_grid) {
    double wx, wy;
    master_from_grid_.apply(c[0], c[1], wx, wy);
    current_min_x = std::min(current_min_x, wx);
    current_min_y = std::min(current_min_y, wy);
    current_max_x = std::max(current_max_x, wx);
    current_max_y = std::max(current_max_y, wy);
  }

  *min_x = std::min(*min_x, current_min_x);
  *min_y = std::min(*min_y, current_min_y);
  *max_x = std::max(*max_x, current_max_x);
  *max_y = std::max(*max_y, current_max_y);

  if (has_previous_bounds_) {
    *min_x = std::min(*min_x, previous_min_x_);
    *min_y = std::min(*min_y, previous_min_y_);
    *max_x = std::max(*max_x, previous_max_x_);
    *max_y = std::max(*max_y, previous_max_y_);
  }

  previous_min_x_ = current_min_x;
  previous_min_y_ = current_min_y;
  previous_max_x_ = current_max_x;
  previous_max_y_ = current_max_y;
  has_previous_bounds_ = true;
}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (use_map_file_) {
    if (!file_map_loaded_) {
      return;
    }

    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        const int map_x = static_cast<int>(std::floor((wx - file_origin_x_) / file_resolution_));
        const int map_y = static_cast<int>(std::floor((wy - file_origin_y_) / file_resolution_));

        if (map_x < 0 || map_y < 0 || map_x >= file_width_ || map_y >= file_height_) {
          continue;
        }

        const int sample_x = file_flip_x_ ? (file_width_ - 1 - map_x) : map_x;
        const int sample_y = file_flip_y_ ? (file_height_ - 1 - map_y) : map_y;
        const int index = sample_y * file_width_ + sample_x;
        // const unsigned char pixel = file_data_[index];

        // // PGM ROS convention: white (254-255) = free, black (0) = occupied,
        // // gray (205) = unknown. Convert to costmap values.
        // if (pixel == 205) {
        //   master_grid.setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
        // } else {
        //   const double occ = (255.0 - pixel) / 255.0;
        //   if (occ < 0.25) {
        //     master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
        //   } else if (occ > 0.65) {
        //     master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        //   } else {
        //     auto scaled = static_cast<unsigned char>(
        //       1 + (occ - 0.25) / (0.65 - 0.25) * 251);
        //     master_grid.setCost(i, j, scaled);
        //   }
        // }
        const unsigned char pixel = file_data_[index];

        // Saved PGM semantics:
        //   255 = safest / free
        //     0 = most dangerous
        //
        // Convert directly to Nav2 cost range:
        //   0   = FREE_SPACE
        //   252 = maximum non-lethal traversal cost
        // const double danger = (255.0 - static_cast<double>(pixel)) / 255.0;
        const double danger = (static_cast<double>(pixel)) / 255.0;

        const unsigned char scaled_cost =
          static_cast<unsigned char>(std::round(danger * 252.0));

        master_grid.setCost(i, j, scaled_cost);
      }
    }
    return;
  }

  if (!active_map_ || !grid_from_master_.valid) return;

  resizePersistenceIfNeeded(master_grid);
  std::fill(gradient_touched_cells_.begin(), gradient_touched_cells_.end(), 0);

  const double inv_res   = 1.0 / active_map_->info.resolution;
  const double grid_ox   = active_map_->info.origin.position.x;
  const double grid_oy   = active_map_->info.origin.position.y;
  const int    grid_w    = static_cast<int>(active_map_->info.width);
  const int    grid_h    = static_cast<int>(active_map_->info.height);
  const auto & grid_data = active_map_->data;
  const int64_t now_ms = clock_ ? clock_->now().nanoseconds() / 1000000 : 0;
  const int64_t timeout_ms = static_cast<int64_t>(std::max(0.0, persistence_timeout_) * 1000.0);

  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      const size_t master_index = master_grid.getIndex(i, j);
      double wx_master, wy_master;
      master_grid.mapToWorld(i, j, wx_master, wy_master);

      // Master-frame world point -> grid-frame world point -> grid cell.
      double wx_grid, wy_grid;
      grid_from_master_.apply(wx_master, wy_master, wx_grid, wy_grid);

      const int map_x = static_cast<int>(std::floor((wx_grid - grid_ox) * inv_res));
      const int map_y = static_cast<int>(std::floor((wy_grid - grid_oy) * inv_res));

      if (map_x < 0 || map_y < 0 || map_x >= grid_w || map_y >= grid_h) {
        if (persistent_patch_ && master_index < persistent_costs_.size()) {
          if (persistent_stamps_ms_[master_index] > 0 &&
              now_ms - persistent_stamps_ms_[master_index] <= timeout_ms) {
            if (master_grid.getCost(i, j) != persistent_costs_[master_index]) {
              master_grid.setCost(i, j, persistent_costs_[master_index]);
            }
            gradient_owned_cells_[master_index] = 1;
            if (master_index < gradient_touched_cells_.size()) {
              gradient_touched_cells_[master_index] = 1;
            }
          }
        }
        continue;
      }

      const int index = map_y * grid_w + map_x;
      const int8_t cost = grid_data[index];

      if (cost < 0) {
        continue;
      }

      // Zero is an explicit clear from the traversability producer. Only apply
      // it to cells previously written by this layer so flat terrain does not
      // erase unrelated obstacle layers.
      if (cost <= 0) {
        if (master_index < gradient_touched_cells_.size()) {
          gradient_touched_cells_[master_index] = 1;
        }
        if (master_index < gradient_owned_cells_.size() && gradient_owned_cells_[master_index]) {
          if (master_index < persistent_costs_.size() &&
              master_grid.getCost(i, j) == persistent_costs_[master_index]) {
            master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
          }
          gradient_owned_cells_[master_index] = 0;
          if (master_index < persistent_stamps_ms_.size()) {
            persistent_stamps_ms_[master_index] = 0;
          }
        }
        continue;
      }

      // Scale 1-100 occupancy to 2-252 costmap range.
      const unsigned char scaled_cost = static_cast<unsigned char>((cost * 252) / 100);
      if (persistent_patch_ && master_index < persistent_costs_.size()) {
        if (scaled_cost < persistence_min_cost_) {
          continue;
        }
        persistent_costs_[master_index] = scaled_cost;
        persistent_stamps_ms_[master_index] = now_ms;
        gradient_touched_cells_[master_index] = 1;
        if (master_grid.getCost(i, j) != persistent_costs_[master_index]) {
          master_grid.setCost(i, j, persistent_costs_[master_index]);
        }
        gradient_owned_cells_[master_index] = 1;
      } else {
        if (master_grid.getCost(i, j) != scaled_cost) {
          master_grid.setCost(i, j, scaled_cost);
        }
        if (master_index < gradient_owned_cells_.size()) {
          gradient_owned_cells_[master_index] = 1;
        }
        if (master_index < gradient_touched_cells_.size()) {
          gradient_touched_cells_[master_index] = 1;
        }
        if (master_index < persistent_costs_.size()) {
          persistent_costs_[master_index] = scaled_cost;
        }
      }
    }
  }

  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      const size_t master_index = master_grid.getIndex(i, j);
      if (master_index >= gradient_owned_cells_.size() ||
          master_index >= gradient_touched_cells_.size() ||
          master_index >= persistent_costs_.size()) {
        continue;
      }

      if (!gradient_owned_cells_[master_index] || gradient_touched_cells_[master_index]) {
        continue;
      }

      if (master_grid.getCost(i, j) == persistent_costs_[master_index]) {
        master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      }
      gradient_owned_cells_[master_index] = 0;
      persistent_stamps_ms_[master_index] = 0;
      persistent_costs_[master_index] = nav2_costmap_2d::FREE_SPACE;
    }
  }
}

}  // namespace gradient_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_layer::GradientLayer, nav2_costmap_2d::Layer)