#include "lidar_yaw_base_finder/lidar_yaw_base_finder.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lidar_yaw_base_finder
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double degToRad(const double degrees) {return degrees * kPi / 180.0;}
double radToDeg(const double radians) {return radians * 180.0 / kPi;}

double wrapDeg(double angle_deg)
{
  while (angle_deg <= -180.0) {angle_deg += 360.0;}
  while (angle_deg > 180.0) {angle_deg -= 360.0;}
  return angle_deg;
}

double angularDifferenceDeg(const double first_deg, const double second_deg)
{
  return wrapDeg(first_deg - second_deg);
}

geometry_msgs::msg::Point pointAt(const double radius_m, const double angle_deg)
{
  geometry_msgs::msg::Point point;
  point.x = radius_m * std::cos(degToRad(angle_deg));
  point.y = radius_m * std::sin(degToRad(angle_deg));
  return point;
}
}  // namespace

LidarYawBaseFinder::LidarYawBaseFinder()
: Node("lidar_yaw_base_finder")
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/ouster/points");
  filtered_cloud_topic_ = declare_parameter<std::string>(
    "filtered_cloud_topic", "dead_zone/filtered_cloud");
  line_inliers_topic_ = declare_parameter<std::string>(
    "line_inliers_topic", "dead_zone/line_inliers");
  line_markers_topic_ = declare_parameter<std::string>(
    "line_markers_topic", "dead_zone/boundary_lines");
  sector_min_deg_ = declare_parameter<double>("sector_min_deg", -90.0);
  sector_max_deg_ = declare_parameter<double>("sector_max_deg", 0.0);
  min_radius_m_ = declare_parameter<double>("min_radius_m", 0.60);
  max_radius_m_ = declare_parameter<double>("max_radius_m", 1.10);
  angular_bin_size_deg_ = declare_parameter<double>("angular_bin_size_deg", 0.25);
  min_points_per_angular_bin_ = declare_parameter<int>("min_points_per_angular_bin", 2);
  edge_response_threshold_ = declare_parameter<double>("edge_response_threshold", 2.0);
  temporal_line_merge_angle_deg_ = declare_parameter<double>(
    "temporal_line_merge_angle_deg", 1.0);
  temporal_average_window_sec_ = declare_parameter<double>("temporal_average_window_sec", 2.0);
  line_track_timeout_sec_ = declare_parameter<double>("line_track_timeout_sec", 0.0);
  line_candidate_half_width_deg_ = declare_parameter<double>(
    "line_candidate_half_width_deg", 1.5);
  max_line_angle_error_deg_ = declare_parameter<double>("max_line_angle_error_deg", 2.0);
  pattern_gap_tolerance_deg_ = declare_parameter<double>("pattern_gap_tolerance_deg", 2.0);
  line_distance_threshold_m_ = declare_parameter<double>("line_distance_threshold_m", 0.025);
  line_origin_max_offset_m_ = declare_parameter<double>("line_origin_max_offset_m", 0.08);
  ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 300);
  min_line_inliers_ = declare_parameter<int>("min_line_inliers", 12);
  known_pattern_start_in_base_deg_ = declare_parameter<double>(
    "known_pattern_start_in_base_deg", 0.0);

  if (sector_min_deg_ >= sector_max_deg_ || min_radius_m_ <= 0.0 ||
    min_radius_m_ >= max_radius_m_ || angular_bin_size_deg_ <= 0.0 ||
    min_points_per_angular_bin_ < 1 || edge_response_threshold_ <= 0.0 ||
    temporal_line_merge_angle_deg_ <= 0.0 || temporal_average_window_sec_ <= 0.0 ||
    line_track_timeout_sec_ < 0.0 ||
    line_candidate_half_width_deg_ <= 0.0 || max_line_angle_error_deg_ <= 0.0 ||
    pattern_gap_tolerance_deg_ <= 0.0 ||
    line_distance_threshold_m_ <= 0.0 ||
    line_origin_max_offset_m_ <= 0.0 || ransac_max_iterations_ < 1 || min_line_inliers_ < 2)
  {
    throw std::invalid_argument("Invalid sector, radius, or azimuth-bin parameters");
  }

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    filtered_cloud_topic_, sensor_qos);
  line_inliers_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    line_inliers_topic_, sensor_qos);
  line_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    line_markers_topic_, marker_qos);
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, sensor_qos,
    std::bind(&LidarYawBaseFinder::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Listening on '%s'; filtering azimuth [%.1f, %.1f] deg and radius [%.2f, %.2f] m",
    input_cloud_topic_.c_str(), sector_min_deg_, sector_max_deg_, min_radius_m_, max_radius_m_);
}

void LidarYawBaseFinder::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  Cloud input_cloud;
  pcl::fromROSMsg(*message, input_cloud);

  Cloud filtered_cloud;
  filtered_cloud.reserve(input_cloud.size());
  for (const auto & point : input_cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double radius_m = std::hypot(point.x, point.y);
    const double angle_deg = radToDeg(std::atan2(point.y, point.x));
    if (radius_m >= min_radius_m_ && radius_m <= max_radius_m_ &&
      angle_deg >= sector_min_deg_ && angle_deg <= sector_max_deg_)
    {
      // Collapse all elevation planes onto the horizontal XY plane before
      // publishing or fitting. Cone-border lines are radial in this plane.
      pcl::PointXYZ projected_point = point;
      projected_point.z = 0.0F;
      filtered_cloud.push_back(projected_point);
    }
  }
  filtered_cloud.width = static_cast<std::uint32_t>(filtered_cloud.size());
  filtered_cloud.height = 1;
  filtered_cloud.is_dense = true;

  sensor_msgs::msg::PointCloud2 filtered_message;
  pcl::toROSMsg(filtered_cloud, filtered_message);
  filtered_message.header = message->header;
  filtered_cloud_publisher_->publish(filtered_message);

  const std::vector<FittedLine> detected_lines = extractDensityEdgeLines(filtered_cloud);
  const std::vector<FittedLine> averaged_lines = updateLineTracks(
    detected_lines, get_clock()->now());
  const auto pattern_lines = selectPatternLines(averaged_lines);
  const std::vector<FittedLine> matched_lines = pattern_lines ?
    std::vector<FittedLine>(pattern_lines->begin(), pattern_lines->end()) :
    std::vector<FittedLine>{};

  // Publish every fitted ray, even when the known gap pattern is absent.
  Cloud all_inliers;
  for (const auto & line : detected_lines) {
    for (const int index : line.inlier_indices) {
      all_inliers.push_back(filtered_cloud.at(static_cast<std::size_t>(index)));
    }
  }
  all_inliers.width = static_cast<std::uint32_t>(all_inliers.size());
  all_inliers.height = 1;
  all_inliers.is_dense = true;

  sensor_msgs::msg::PointCloud2 inliers_message;
  pcl::toROSMsg(all_inliers, inliers_message);
  inliers_message.header = message->header;
  line_inliers_publisher_->publish(inliers_message);
  publishLines(message->header, averaged_lines, matched_lines);

  if (!averaged_lines.empty()) {
    std::ostringstream detected_report;
    detected_report.setf(std::ios::fixed);
    detected_report.precision(2);
    detected_report << "Time-averaged RANSAC rays:";
    for (const auto & line : averaged_lines) {
      detected_report << " " << line.angle_deg << " deg";
    }
    if (averaged_lines.size() > 1U) {
      detected_report << "; gaps between successive rays:";
      for (std::size_t index = 1; index < averaged_lines.size(); ++index) {
        detected_report << " " << angularDifferenceDeg(
          averaged_lines[index].angle_deg, averaged_lines[index - 1].angle_deg) << " deg";
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s", detected_report.str().c_str());
  }

  if (!pattern_lines) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Detected %zu edge rays; retaining %zu time-averaged rays, but no 7 deg dead -> 9 deg live -> 17 deg dead pattern matched",
      detected_lines.size(), averaged_lines.size());
    return;
  }

  std::ostringstream report;
  report.setf(std::ios::fixed);
  report.precision(2);
  report << "RANSAC boundaries:";
  for (const auto & line : matched_lines) {
    report << " " << line.angle_deg << " deg";
  }
  report << "; measured gaps:";
  for (std::size_t index = 1; index < matched_lines.size(); ++index) {
    report << " " << angularDifferenceDeg(
      matched_lines[index].angle_deg, matched_lines[index - 1].angle_deg) << " deg";
  }
  report << "; lidar->base yaw: " << wrapDeg(
    known_pattern_start_in_base_deg_ - matched_lines.front().angle_deg) << " deg";
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", report.str().c_str());
}

std::optional<LidarYawBaseFinder::FittedLine> LidarYawBaseFinder::fitRadialLineCandidate(
  const Cloud & cloud, const double seed_angle_deg) const
{
  Cloud::Ptr candidates(new Cloud());
  std::vector<int> candidate_to_filtered;
  candidates->reserve(cloud.size());
  candidate_to_filtered.reserve(cloud.size());
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const auto & point = cloud.at(index);
    const double angle_deg = radToDeg(std::atan2(point.y, point.x));
    if (std::abs(angularDifferenceDeg(angle_deg, seed_angle_deg)) > line_candidate_half_width_deg_) {
      continue;
    }
    pcl::PointXYZ projected;
    projected.x = point.x;
    projected.y = point.y;
    projected.z = 0.0F;
    candidates->push_back(projected);
    candidate_to_filtered.push_back(static_cast<int>(index));
    mean_x += point.x;
    mean_y += point.y;
  }
  if (static_cast<int>(candidates->size()) < min_line_inliers_) {
    return std::nullopt;
  }
  mean_x /= static_cast<double>(candidates->size());
  mean_y /= static_cast<double>(candidates->size());

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_LINE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(ransac_max_iterations_);
  segmentation.setDistanceThreshold(line_distance_threshold_m_);
  segmentation.setInputCloud(candidates);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (static_cast<int>(inliers.indices.size()) < min_line_inliers_ || coefficients.values.size() < 6U) {
    return std::nullopt;
  }

  double direction_x = coefficients.values[3];
  double direction_y = coefficients.values[4];
  const double direction_norm = std::hypot(direction_x, direction_y);
  if (direction_norm < std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }
  direction_x /= direction_norm;
  direction_y /= direction_norm;
  const double line_origin_distance = std::abs(
    coefficients.values[0] * direction_y - coefficients.values[1] * direction_x);
  if (line_origin_distance > line_origin_max_offset_m_) {
    return std::nullopt;
  }
  if (direction_x * mean_x + direction_y * mean_y < 0.0) {
    direction_x = -direction_x;
    direction_y = -direction_y;
  }

  FittedLine result;
  result.angle_deg = radToDeg(std::atan2(direction_y, direction_x));
  if (std::abs(angularDifferenceDeg(result.angle_deg, seed_angle_deg)) >
    max_line_angle_error_deg_)
  {
    return std::nullopt;
  }
  result.inlier_indices.reserve(inliers.indices.size());
  for (const int candidate_index : inliers.indices) {
    result.inlier_indices.push_back(candidate_to_filtered.at(static_cast<std::size_t>(candidate_index)));
  }
  return result;
}

std::vector<LidarYawBaseFinder::FittedLine> LidarYawBaseFinder::extractDensityEdgeLines(
  const Cloud & cloud) const
{
  const int bin_count = static_cast<int>(std::ceil(
    (sector_max_deg_ - sector_min_deg_) / angular_bin_size_deg_));
  if (bin_count < 5) {
    return {};
  }

  // Each element says whether this angular slice contains enough projected
  // points to be considered live. This is the signal used by the convolution.
  std::vector<int> point_counts(static_cast<std::size_t>(bin_count), 0);
  for (const auto & point : cloud.points) {
    const double angle_deg = radToDeg(std::atan2(point.y, point.x));
    const int bin = static_cast<int>(std::floor(
      (angle_deg - sector_min_deg_) / angular_bin_size_deg_));
    if (bin >= 0 && bin < bin_count) {
      ++point_counts[static_cast<std::size_t>(bin)];
    }
  }
  std::vector<double> occupancy(static_cast<std::size_t>(bin_count), 0.0);
  for (int bin = 0; bin < bin_count; ++bin) {
    occupancy[static_cast<std::size_t>(bin)] =
      point_counts[static_cast<std::size_t>(bin)] >= min_points_per_angular_bin_ ? 1.0 : 0.0;
  }

  // First derivative of a lightly-smoothed binary signal. A positive response
  // is a live-zone start; a negative response is a live-zone end.
  std::vector<double> edge_response(static_cast<std::size_t>(bin_count), 0.0);
  for (int bin = 2; bin < bin_count - 2; ++bin) {
    edge_response[static_cast<std::size_t>(bin)] =
      -occupancy[static_cast<std::size_t>(bin - 2)] -
      2.0 * occupancy[static_cast<std::size_t>(bin - 1)] +
      2.0 * occupancy[static_cast<std::size_t>(bin + 1)] +
      occupancy[static_cast<std::size_t>(bin + 2)];
  }

  std::vector<FittedLine> lines;
  for (int bin = 3; bin < bin_count - 3; ++bin) {
    const double response = std::abs(edge_response[static_cast<std::size_t>(bin)]);
    const double previous_response = std::abs(edge_response[static_cast<std::size_t>(bin - 1)]);
    const double next_response = std::abs(edge_response[static_cast<std::size_t>(bin + 1)]);
    // Non-maximum suppression turns each density transition into one edge.
    if (response < edge_response_threshold_ || response < previous_response ||
      response <= next_response)
    {
      continue;
    }
    const double seed_angle_deg = sector_min_deg_ +
      (static_cast<double>(bin) + 0.5) * angular_bin_size_deg_;
    const auto line = fitRadialLineCandidate(cloud, seed_angle_deg);
    if (line) {
      lines.push_back(*line);
    }
  }
  return lines;
}

std::vector<LidarYawBaseFinder::FittedLine> LidarYawBaseFinder::updateLineTracks(
  const std::vector<FittedLine> & detected_lines, const rclcpp::Time & stamp)
{
  for (const auto & line : detected_lines) {
    auto closest_track = line_tracks_.end();
    double closest_difference_deg = temporal_line_merge_angle_deg_;
    for (auto track = line_tracks_.begin(); track != line_tracks_.end(); ++track) {
      const double difference_deg = std::abs(angularDifferenceDeg(line.angle_deg, track->angle_deg));
      if (difference_deg < closest_difference_deg) {
        closest_difference_deg = difference_deg;
        closest_track = track;
      }
    }

    if (closest_track == line_tracks_.end()) {
      LineTrack track;
      track.angle_deg = line.angle_deg;
      track.last_seen = stamp;
      track.observations.push_back(TimedAngle{stamp, line.angle_deg});
      line_tracks_.push_back(std::move(track));
    } else {
      closest_track->last_seen = stamp;
      closest_track->observations.push_back(TimedAngle{stamp, line.angle_deg});
    }
  }

  for (auto & track : line_tracks_) {
    while (!track.observations.empty() &&
      (stamp - track.observations.front().stamp).seconds() > temporal_average_window_sec_)
    {
      track.observations.pop_front();
    }
    if (track.observations.empty()) {
      continue;
    }

    // Circular mean avoids a discontinuity if a tracked line crosses +/-180 deg.
    double sum_cos = 0.0;
    double sum_sin = 0.0;
    for (const auto & observation : track.observations) {
      sum_cos += std::cos(degToRad(observation.angle_deg));
      sum_sin += std::sin(degToRad(observation.angle_deg));
    }
    track.angle_deg = radToDeg(std::atan2(sum_sin, sum_cos));
  }

  if (line_track_timeout_sec_ > 0.0) {
    line_tracks_.erase(
      std::remove_if(
        line_tracks_.begin(), line_tracks_.end(),
        [this, &stamp](const LineTrack & track) {
          return (stamp - track.last_seen).seconds() > line_track_timeout_sec_;
        }),
      line_tracks_.end());
  }

  std::vector<FittedLine> averaged_lines;
  averaged_lines.reserve(line_tracks_.size());
  for (const auto & track : line_tracks_) {
    FittedLine line;
    line.angle_deg = track.angle_deg;
    averaged_lines.push_back(line);
  }
  std::sort(
    averaged_lines.begin(), averaged_lines.end(),
    [](const FittedLine & first, const FittedLine & second) {
      return first.angle_deg < second.angle_deg;
    });
  return averaged_lines;
}

std::optional<std::array<LidarYawBaseFinder::FittedLine, 4>>
LidarYawBaseFinder::selectPatternLines(std::vector<FittedLine> lines) const
{
  if (lines.size() < 4U) {
    return std::nullopt;
  }
  std::sort(lines.begin(), lines.end(), [](const FittedLine & first, const FittedLine & second) {
    return first.angle_deg < second.angle_deg;
  });

  double best_error = std::numeric_limits<double>::infinity();
  std::optional<std::array<FittedLine, 4>> best_pattern;
  for (std::size_t first_gap = 0; first_gap + 1U < lines.size(); ++first_gap) {
    const double first_dead_gap = lines[first_gap + 1U].angle_deg - lines[first_gap].angle_deg;
    if (std::abs(first_dead_gap - 7.0) > pattern_gap_tolerance_deg_) {
      continue;
    }
    for (std::size_t second_gap = first_gap + 2U;
      second_gap + 1U < lines.size(); ++second_gap)
    {
      const double live_span = lines[second_gap].angle_deg - lines[first_gap + 1U].angle_deg;
      const double second_dead_gap = lines[second_gap + 1U].angle_deg - lines[second_gap].angle_deg;
      if (std::abs(live_span - 9.0) > pattern_gap_tolerance_deg_ ||
        std::abs(second_dead_gap - 17.0) > pattern_gap_tolerance_deg_)
      {
        continue;
      }
      const double error = std::abs(first_dead_gap - 7.0) +
        std::abs(live_span - 9.0) + std::abs(second_dead_gap - 17.0);
      if (error < best_error) {
        best_error = error;
        best_pattern = std::array<FittedLine, 4>{
          lines[first_gap], lines[first_gap + 1U],
          lines[second_gap], lines[second_gap + 1U]};
      }
    }
  }
  return best_pattern;
}

void LidarYawBaseFinder::publishLines(
  const std_msgs::msg::Header & header,
  const std::vector<FittedLine> & detected_lines,
  const std::vector<FittedLine> & matched_lines) const
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker line_marker;
  line_marker.header = header;
  line_marker.ns = "ransac_detected_rays";
  line_marker.id = 1;
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.025;
  line_marker.color.r = 0.2F;
  line_marker.color.g = 0.6F;
  line_marker.color.b = 1.0F;
  line_marker.color.a = 1.0F;
  for (const auto & line : detected_lines) {
    // Each boundary ray starts at the LiDAR origin, not at the donut's inner edge.
    line_marker.points.push_back(pointAt(0.0, line.angle_deg));
    line_marker.points.push_back(pointAt(max_radius_m_, line.angle_deg));
  }
  if (!line_marker.points.empty()) {
    markers.markers.push_back(line_marker);
  }

  visualization_msgs::msg::Marker matched_marker = line_marker;
  matched_marker.ns = "dead_zone_pattern_rays";
  matched_marker.id = 2;
  matched_marker.scale.x = 0.045;
  matched_marker.color.r = 0.0F;
  matched_marker.color.g = 1.0F;
  matched_marker.color.b = 0.2F;
  matched_marker.points.clear();
  for (const auto & line : matched_lines) {
    matched_marker.points.push_back(pointAt(0.0, line.angle_deg));
    matched_marker.points.push_back(pointAt(max_radius_m_, line.angle_deg));
  }
  if (!matched_marker.points.empty()) {
    markers.markers.push_back(matched_marker);
  }

  for (std::size_t index = 0; index < matched_lines.size(); ++index) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "dead_zone_pattern_labels";
    text_marker.id = static_cast<int>(index);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = pointAt(max_radius_m_ + 0.10, matched_lines[index].angle_deg);
    text_marker.pose.position.z = 0.10;
    text_marker.scale.z = 0.10;
    text_marker.color = matched_marker.color;
    std::ostringstream label;
    label.setf(std::ios::fixed);
    label.precision(2);
    label << matched_lines[index].angle_deg << " deg";
    text_marker.text = label.str();
    markers.markers.push_back(text_marker);
  }
  line_markers_publisher_->publish(markers);
}

}  // namespace lidar_yaw_base_finder

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_yaw_base_finder::LidarYawBaseFinder>());
  rclcpp::shutdown();
  return 0;
}
