#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#define ARUCO_BOX_OFFSET 0.125
#define MAX_ARUCO_DIST 10.0 // meters
#define MARKER_SIZE 0.144

class MultiViewArucoNode : public rclcpp::Node
{
public:
  MultiViewArucoNode()
      : Node("multi_view_aruco_node"),
        image_sub_1_(),
        image_sub_2_(),
        image_sub_3_(),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("aruco_dictionary_id", "DICT_5X5_250");
    declare_parameter<double>("marker_size", MARKER_SIZE);

    declare_parameter<std::string>("image_topic_1", "/NAV/feed_camera_nav_0");
    declare_parameter<std::string>("camera_frame_1", "oak_camera_top_right_1");
    declare_parameter<std::string>("image_topic_2", "/NAV/feed_camera_nav_1");
    declare_parameter<std::string>("camera_frame_2", "oak_camera_top_left_1");
    declare_parameter<std::string>("image_topic_3", "/NAV/feed_camera_nav_2");
    declare_parameter<std::string>("camera_frame_3", "oak_camera_front_1");

    camera_frames_ = {
        get_parameter("camera_frame_1").as_string(),
        get_parameter("camera_frame_2").as_string(),
        get_parameter("camera_frame_3").as_string()};

    const auto dictionary_name = get_parameter("aruco_dictionary_id").as_string();
    aruco_dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryIdFromName(dictionary_name));
    aruco_parameters_ = cv::aruco::DetectorParameters::create();
    aruco_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    initializeCameraModels();

    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
    markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10);

    image_sub_1_.subscribe(this, get_parameter("image_topic_1").as_string(), rmw_qos_profile_sensor_data);
    image_sub_2_.subscribe(this, get_parameter("image_topic_2").as_string(), rmw_qos_profile_sensor_data);
    image_sub_3_.subscribe(this, get_parameter("image_topic_3").as_string(), rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(2), image_sub_1_, image_sub_2_, image_sub_3_);

    sync_->registerCallback(std::bind(&MultiViewArucoNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "C++ multi-view ArUco node started.");
  }

private:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage, CompressedImage>;

  struct MarkerCandidate
  {
    cv::Vec3d tvec;
    cv::Vec3d rvec;
    cv::Matx33d rot_matrix;
    double yaw_score_deg;
    std::vector<cv::Point2f> corners;
  };

  static constexpr double kPi = 3.14159265358979323846;

  static double radToDeg(double rad)
  {
    return rad * 180.0 / kPi;
  }

  static double normalizeAngleDeg(double deg)
  {
    double out = std::fmod(deg + 180.0, 360.0);
    if (out < 0.0)
      out += 360.0;
    return out - 180.0;
  }

  static int markerToIndex(int erc_id)
  {
    if (erc_id >= 51 && erc_id <= 65)
      return erc_id - 51;
    return -1;
  }

  static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryIdFromName(const std::string &name)
  {
    static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};

    const auto it = dict.find(name);
    if (it == dict.end())
      return cv::aruco::DICT_5X5_250;
    return it->second;
  }

  static Eigen::Matrix3d toEigen(const cv::Matx33d &m)
  {
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        out(r, c) = m(r, c);
      }
    }
    return out;
  }

  bool isOakFrame(const std::string &camera_frame) const
  {
    return camera_frame == "oak_camera_top_right_1" ||
           camera_frame == "oak_camera_top_left_1" ||
           camera_frame == "oak_camera_front_1";
  }

  void initializeCameraModels()
  {
    distortion_oakd_ = (cv::Mat_<double>(1, 4) << 3.03940603e-01, -2.28313655e+00, -4.79004397e-03, 3.08347206e-03);
    intrinsic_oakd_ = (cv::Mat_<double>(3, 3) << 1.96685610e+03, 0.0, 9.99954556e+02, 0.0, 1.96039881e+03, 4.79997092e+02, 0.0, 0.0, 1.0);
  }

  std::pair<double, Eigen::Matrix4d> calculateArucoBoxBearing(const cv::Vec3d &tvec, const cv::Vec3d &rvec) const
  {
    cv::Mat R_tag_to_cam_cv;
    cv::Rodrigues(rvec, R_tag_to_cam_cv);
    const cv::Matx33d R_tag_to_cam = R_tag_to_cam_cv;

    const cv::Vec3d offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
    const cv::Vec3d offset_cam = R_tag_to_cam * offset;
    const cv::Vec3d box_center_tvec = tvec + offset_cam;

    const double bearing_rad = std::atan2(-box_center_tvec[0], box_center_tvec[2]);

    Eigen::Matrix4d T_cam_box = Eigen::Matrix4d::Identity();
    const Eigen::AngleAxisd yaw_rot(bearing_rad, Eigen::Vector3d::UnitZ());
    T_cam_box.block<3, 3>(0, 0) = yaw_rot.toRotationMatrix();
    T_cam_box.block<3, 1>(0, 3) = Eigen::Vector3d(box_center_tvec[0], box_center_tvec[1], box_center_tvec[2]);

    return {radToDeg(bearing_rad), T_cam_box};
  }

  Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::TransformStamped &transform) const
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(t.x, t.y, t.z);

    return T;
  }

  std::optional<double> calculateBearingWithFov(
      int image_width,
      const std::vector<cv::Point2f> &marker_corners,
      const std::string &camera_frame,
      const geometry_msgs::msg::TransformStamped &transform) const
  {
    if (marker_corners.size() != 4U)
      return std::nullopt;

    double fov_deg = 0.0;
    if (isOakFrame(camera_frame))
      fov_deg = 55.0;
    else
      return std::nullopt;

    const double fpx = static_cast<double>(image_width) / (2.0 * std::tan((fov_deg * kPi / 180.0) / 2.0));

    cv::Point2f center(0.0F, 0.0F);
    for (const auto &p : marker_corners)
    {
      center += p;
    }
    center *= 0.25F;

    const double dx = static_cast<double>(center.x) - static_cast<double>(image_width) * 0.5;
    const double bearing_manual_rad = -std::atan2(dx, fpx);

    const auto &q = transform.transform.rotation;
    tf2::Quaternion q_cam_base(q.x, q.y, q.z, q.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw_cam2base_rad = 0.0;
    tf2::Matrix3x3(q_cam_base).getRPY(roll, pitch, yaw_cam2base_rad);
    (void)pitch;

    const double yaw_base_rad = std::abs(roll) > 0.05 ? yaw_cam2base_rad - bearing_manual_rad : yaw_cam2base_rad + bearing_manual_rad;

    return normalizeAngleDeg(radToDeg(yaw_base_rad));
  }

  void processImage(
      const CompressedImage::ConstSharedPtr &img_msg,
      const cv::Mat &intrinsic_mat,
      const cv::Mat &distortion,
      const std::string &camera_frame,
      ros2_aruco_interfaces::msg::ArucoMarkers &markers,
      geometry_msgs::msg::PoseArray &pose_array)
  {
    if (img_msg->data.empty())
      return;

    cv::Mat encoded(1, static_cast<int>(img_msg->data.size()), CV_8UC1, const_cast<unsigned char *>(img_msg->data.data()));
    cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty())
      return;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, aruco_dictionary_, corners, marker_ids, aruco_parameters_);
    if (marker_ids.empty())
      return;

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, intrinsic_mat, distortion, rvecs, tvecs);

    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
      return;
    }

    std::unordered_map<int, MarkerCandidate> marker_candidates;

    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      const cv::Vec3d &tvec = tvecs[i];
      const double marker_distance = std::sqrt(tvec.dot(tvec));
      if (marker_distance > MAX_ARUCO_DIST)
        continue;

      cv::Mat R_tag2cam_cv;
      cv::Rodrigues(rvecs[i], R_tag2cam_cv);
      const cv::Matx33d R_tag2cam = R_tag2cam_cv;
      const cv::Matx33d R_cam2tag = R_tag2cam.t();
      const cv::Matx33d R_180_x(
          1.0, 0.0, 0.0,
          0.0, -1.0, 0.0,
          0.0, 0.0, -1.0);
      const cv::Matx33d R_corrected = R_180_x * R_cam2tag;

      const Eigen::Vector3d euler = toEigen(R_corrected).eulerAngles(0, 1, 2);
      const double score_deg = std::abs(radToDeg(euler[1]));

      MarkerCandidate candidate;
      candidate.tvec = tvec;
      candidate.rvec = rvecs[i];
      candidate.rot_matrix = R_tag2cam;
      candidate.yaw_score_deg = score_deg;
      candidate.corners = corners[i];

      const int marker_id = marker_ids[i];
      const auto it = marker_candidates.find(marker_id);
      if (it == marker_candidates.end() || score_deg < it->second.yaw_score_deg)
        marker_candidates[marker_id] = candidate;
    }

    for (const auto &[marker_id, candidate] : marker_candidates)
    {
      geometry_msgs::msg::Pose pose_cam;

      const cv::Vec3d face_to_center_offset(0.0, 0.0, -ARUCO_BOX_OFFSET);
      const cv::Vec3d adjusted_tvec = candidate.tvec + candidate.rot_matrix * face_to_center_offset;

      pose_cam.position.x = adjusted_tvec[2];
      pose_cam.position.y = -adjusted_tvec[0];
      pose_cam.position.z = -adjusted_tvec[1];
      pose_cam.orientation.x = 0.0;
      pose_cam.orientation.y = 0.0;
      pose_cam.orientation.z = 0.0;
      pose_cam.orientation.w = 1.0;

      geometry_msgs::msg::PoseStamped pose_in;
      geometry_msgs::msg::PoseStamped pose_out;
      pose_in.header = img_msg->header;
      pose_in.header.frame_id = camera_frame;
      pose_in.pose = pose_cam;

      try
      {
        tf2::doTransform(pose_in, pose_out, transform);
      }
      catch (const tf2::TransformException &)
      {
        continue;
      }

      const int aruco_index = markerToIndex(marker_id);
      if (aruco_index < 0)
        continue;

      if (std::find(markers.marker_ids.begin(), markers.marker_ids.end(), aruco_index) != markers.marker_ids.end())
        continue;

      pose_array.poses.push_back(pose_out.pose);
      markers.poses.push_back(pose_out.pose);
      markers.marker_ids.push_back(aruco_index);

      auto [bearing_deg, tf_cam_box] = calculateArucoBoxBearing(candidate.tvec, candidate.rvec);
      (void)bearing_deg;

      const Eigen::Matrix4d t_base_link_to_aruco_box = transformToMatrix(transform) * tf_cam_box;
      double yaw_deg = normalizeAngleDeg(radToDeg(std::atan2(t_base_link_to_aruco_box(1, 0), t_base_link_to_aruco_box(0, 0))));

      const auto yaw_from_fov = calculateBearingWithFov(gray.cols, candidate.corners, camera_frame, transform);

      if (yaw_from_fov.has_value())
        yaw_deg = yaw_from_fov.value();

      RCLCPP_INFO(get_logger(), "tag %d bearing rv frame: %.2f deg", aruco_index + 51, yaw_deg);

      const auto &lm = landmark_poses_[static_cast<size_t>(aruco_index)];
      markers.landmark_map_pos_x.push_back(lm.first);
      markers.landmark_map_pos_y.push_back(lm.second);
      markers.ar_angles_list.push_back(yaw_deg);
    }
  }

  void syncedCallback(const CompressedImage::ConstSharedPtr &img_msg_1, const CompressedImage::ConstSharedPtr &img_msg_2, const CompressedImage::ConstSharedPtr &img_msg_3)
  {
    ros2_aruco_interfaces::msg::ArucoMarkers markers;
    geometry_msgs::msg::PoseArray pose_array;

    markers.header.frame_id = base_frame_;
    pose_array.header.frame_id = base_frame_;
    markers.header.stamp = img_msg_1->header.stamp;
    pose_array.header.stamp = img_msg_1->header.stamp;

    processImage(img_msg_1, intrinsic_oakd_, distortion_oakd_, camera_frames_[0], markers, pose_array);
    processImage(img_msg_2, intrinsic_oakd_, distortion_oakd_, camera_frames_[1], markers, pose_array);
    processImage(img_msg_3, intrinsic_oakd_, distortion_oakd_, camera_frames_[2], markers, pose_array);

    poses_pub_->publish(pose_array);
    markers_pub_->publish(markers);
  }

  const std::string base_frame_{"base_link"};

  std::array<std::string, 3> camera_frames_{};

  const std::array<std::pair<double, double>, 15> landmark_poses_{
      std::make_pair(-0.585, 0.0),
      std::make_pair(2.62, 0.505),
      std::make_pair(1.46, 8.45),
      std::make_pair(-2.28, 15.81),
      std::make_pair(3.74, 19.07),
      std::make_pair(7.04, 14.67),
      std::make_pair(11.46, 19.78),
      std::make_pair(15.51, 19.33),
      std::make_pair(16.3, 14.87),
      std::make_pair(999999.0, 999999.0),
      std::make_pair(999999.0, 999999.0),
      std::make_pair(999999.0, 999999.0),
      std::make_pair(999999.0, 999999.0),
      std::make_pair(999999.0, 999999.0),
      std::make_pair(999999.0, 999999.0)};

  cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;

  cv::Mat distortion_oakd_;
  cv::Mat intrinsic_oakd_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_pub_;

  message_filters::Subscriber<CompressedImage> image_sub_1_;
  message_filters::Subscriber<CompressedImage> image_sub_2_;
  message_filters::Subscriber<CompressedImage> image_sub_3_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiViewArucoNode>());
  rclcpp::shutdown();
  return 0;
}
