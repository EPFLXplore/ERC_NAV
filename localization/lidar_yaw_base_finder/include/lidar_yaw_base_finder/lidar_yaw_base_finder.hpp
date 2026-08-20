#ifndef LIDAR_YAW_BASE_FINDER__LIDAR_YAW_BASE_FINDER_HPP_
#define LIDAR_YAW_BASE_FINDER__LIDAR_YAW_BASE_FINDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace lidar_yaw_base_finder
{

class LidarYawBaseFinder : public rclcpp::Node
{
public:
  LidarYawBaseFinder();

private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  struct FittedLine
  {
    double angle_deg;
    std::vector<int> inlier_indices;
  };

  struct ConvolutionSample
  {
    double angle_deg;
    double response;
    bool is_peak;
  };

  struct TimedAngle
  {
    rclcpp::Time stamp;
    double angle_deg;
  };

  struct LineTrack
  {
    double angle_deg;
    rclcpp::Time last_seen;
    std::deque<TimedAngle> observations;
  };

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  std::optional<FittedLine> fitRadialLineCandidate(
    const Cloud & cloud,
    double seed_angle_deg) const;
  std::vector<FittedLine> extractDensityEdgeLines(
    const Cloud & cloud,
    std::vector<ConvolutionSample> & convolution_samples) const;
  std::optional<std::array<FittedLine, 4>> selectPatternLines(
    std::vector<FittedLine> lines) const;
  std::vector<FittedLine> updateLineTracks(
    const std::vector<FittedLine> & detected_lines,
    const rclcpp::Time & stamp);
  void publishLines(
    const std_msgs::msg::Header & header,
    const std::vector<FittedLine> & detected_lines,
    const std::vector<FittedLine> & matched_lines,
    const std::vector<ConvolutionSample> & convolution_samples) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr line_inliers_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_markers_publisher_;

  std::string input_cloud_topic_;
  std::string filtered_cloud_topic_;
  std::string line_inliers_topic_;
  std::string line_markers_topic_;
  double sector_min_deg_;
  double sector_max_deg_;
  double min_radius_m_;
  double max_radius_m_;
  double angular_bin_size_deg_;
  int min_points_per_angular_bin_;
  double edge_response_threshold_;
  double convolution_response_scale_m_;
  double temporal_line_merge_angle_deg_;
  double temporal_average_window_sec_;
  double line_track_timeout_sec_;
  double line_candidate_half_width_deg_;
  double max_line_angle_error_deg_;
  std::vector<double> pattern_angles_deg_;
  double pattern_gap_tolerance_deg_;
  double line_distance_threshold_m_;
  double line_origin_max_offset_m_;
  int ransac_max_iterations_;
  int min_line_inliers_;
  double known_pattern_start_in_base_deg_;
  std::vector<LineTrack> line_tracks_;
};

}  // namespace lidar_yaw_base_finder

#endif  // LIDAR_YAW_BASE_FINDER__LIDAR_YAW_BASE_FINDER_HPP_
