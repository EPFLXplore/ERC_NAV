#include "gradient_layer_plugin.hpp"

#include "nav2_costmap_2d/cost_values.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
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

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue("/occupancy_map_local_inflated"));
  declareParameter("use_map_file", rclcpp::ParameterValue(false));
  declareParameter("map_yaml_filename", rclcpp::ParameterValue(""));
  declareParameter("map_flip_x", rclcpp::ParameterValue(false));
  declareParameter("map_flip_y", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("map_subscribe_reliable", rclcpp::ParameterValue(true));
  declareParameter("expand_update_bounds", rclcpp::ParameterValue(true));
  
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
  latest_map_ = msg;
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

  if (!latest_map_) return;

  if (!expand_update_bounds_) return;

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
  if (use_map_file_) {
    if (!file_map_loaded_) {
      return;
    }

    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        const int map_x = static_cast<int>((wx - file_origin_x_) / file_resolution_);
        const int map_y = static_cast<int>((wy - file_origin_y_) / file_resolution_);

        if (map_x < 0 || map_y < 0 || map_x >= file_width_ || map_y >= file_height_) {
          continue;
        }

        const int sample_x = file_flip_x_ ? (file_width_ - 1 - map_x) : map_x;
        const int sample_y = file_flip_y_ ? (file_height_ - 1 - map_y) : map_y;
        const int index = sample_y * file_width_ + sample_x;
        const unsigned char pixel = file_data_[index];

        // PGM ROS convention: white (254-255) = free, black (0) = occupied,
        // gray (205) = unknown. Convert to costmap values.
        if (pixel == 205) {
          master_grid.setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
        } else {
          const double occ = (255.0 - pixel) / 255.0;
          if (occ < 0.25) {
            master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
          } else if (occ > 0.65) {
            master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
          } else {
            auto scaled = static_cast<unsigned char>(
              1 + (occ - 0.25) / (0.65 - 0.25) * 251);
            master_grid.setCost(i, j, scaled);
          }
        }
      }
    }
    return;
  }

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
      unsigned char scaled_cost = (unsigned char)((cost * 252) / 100);
      master_grid.setCost(i, j, scaled_cost);
    }
  }
}

}  // namespace gradient_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gradient_layer::GradientLayer, nav2_costmap_2d::Layer)