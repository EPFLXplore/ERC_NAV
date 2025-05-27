#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>
#include <future>
#include <functional>                // for std::bind
using namespace std::placeholders;  // now _1, _2, etc. are visible


using sensor_msgs::msg::CompressedImage;
using nav_msgs::msg::Odometry;
namespace mf = message_filters;

// Simple struct to hold one image's detection results
struct DetectionResult {
  std::vector<int>                     ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<cv::Vec3d>               rvecs;
  std::vector<cv::Vec3d>               tvecs;
};

class MultiArucoDetector : public rclcpp::Node {
public:
  MultiArucoDetector()
  : Node("multi_aruco_detector")
  {
    // --- parameters ---
    marker_size_   = declare_parameter("marker_size", 0.144);
    box_offset_    = declare_parameter("box_offset", 0.125);
    double cam_fx  = declare_parameter("cam_fx", 0.0);
    double cam_fy  = declare_parameter("cam_fy", 0.0);
    double cam_cx  = declare_parameter("cam_cx", 0.0);
    double cam_cy  = declare_parameter("cam_cy", 0.0);

    // build camera matrix + zero distortion
    cam_mat_ = (cv::Mat1d(3,3) << cam_fx,0,cam_cx,
                                  0,cam_fy,cam_cy,
                                  0,      0,     1);
    dist_coeffs_ = cv::Mat::zeros(1,5,CV_64F);

    // ArUco
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    detector_params_ = cv::aruco::DetectorParameters::create();

    // known map landmarks (ids 0,1,2)
    landmarks_ = {
      Eigen::Vector2d{-1.08, 4.20},
      Eigen::Vector2d{-5.13, -0.17},
      Eigen::Vector2d{-2.24, 4.00}
    };

    // Subscriptions via message_filters
    image_sub1_.subscribe(this, "/NAV/feed_camera_nav_1", rmw_qos_profile_sensor_data);
    image_sub2_.subscribe(this, "/NAV/feed_camera_nav_2", rmw_qos_profile_sensor_data);

    // ApproximateTime sync for 2 streams
    using Policy2 = mf::sync_policies::ApproximateTime<CompressedImage, CompressedImage>;
    sync2_.reset(new mf::Synchronizer<Policy2>(Policy2(10), image_sub1_, image_sub2_));
    sync2_->registerCallback(std::bind(&MultiArucoDetector::onImages, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher
    odom_pub_ = create_publisher<Odometry>("/aruco_odom", 10);

    RCLCPP_INFO(get_logger(), "MultiArucoDetector ready, waiting for image streams...");
  }

private:
  // Runs detectMarkers + estimatePoseSingleMarkers on one image
  DetectionResult runDetection(const CompressedImage::ConstSharedPtr &msg) {
    DetectionResult res;
    cv::Mat gray = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (gray.empty()) return res;

    cv::aruco::detectMarkers(gray, dictionary_, res.corners, res.ids, detector_params_);
    if (res.ids.empty()) return res;

    cv::aruco::estimatePoseSingleMarkers(
      res.corners, marker_size_, cam_mat_, dist_coeffs_, res.rvecs, res.tvecs
    );

    //print detected marker ID
    RCLCPP_INFO(get_logger(), "ID detected : , %d", res.ids[0]);

    return res;
  }

  // synchronized callback for two cameras
  void onImages(const CompressedImage::ConstSharedPtr &img1,
                const CompressedImage::ConstSharedPtr &img2)
  {
    // 1) Launch both detections in parallel
    auto fut1 = std::async(std::launch::async, &MultiArucoDetector::runDetection, this, img1);
    auto fut2 = std::async(std::launch::async, &MultiArucoDetector::runDetection, this, img2);

    // 2) Gather results
    DetectionResult det1 = fut1.get();
    DetectionResult det2 = fut2.get();

    // 3) Fuse all marker observations into one least-squares
    std::vector<double>          distances;
    std::vector<Eigen::Vector2d> points;
    auto accumulate = [&](const DetectionResult &d){
      for (size_t i = 0; i < d.ids.size(); ++i) {
        cv::Matx33d R;
        cv::Rodrigues(d.rvecs[i], R);
        cv::Vec3d         t = d.tvecs[i] + R * cv::Vec3d(0,0,-box_offset_);
        double x =  t[2];
        double y = -t[0];
        distances.push_back(std::hypot(x,y));
        int id = d.ids[i];
        if (id >= 0 && id < static_cast<int>(landmarks_.size()))
          points.push_back(landmarks_[id]);
      }
    };
    accumulate(det1);
    accumulate(det2);

    if (distances.size() < 3 || points.size() < 3) {
      RCLCPP_WARN(get_logger(), "Not enough markers for pose estimation");
      return;
    }

    // 4) Gauss–Newton
    Eigen::Vector2d est = Eigen::Vector2d::Zero();
    for (int iter = 0; iter < 5; ++iter) {
      Eigen::Vector2d grad = Eigen::Vector2d::Zero();
      Eigen::Matrix2d H    = Eigen::Matrix2d::Zero();
      for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector2d diff = est - points[i];
        double pred = diff.norm() + 1e-6;
        Eigen::Vector2d J    = diff / pred;
        double r = pred - distances[i];
        grad += J * r;
        H    += J * J.transpose();
      }
      est -= H.ldlt().solve(grad);
    }

    // 5) Publish as Odometry
    Odometry odom;
    odom.header.stamp    = now();
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = est.x();
    odom.pose.pose.position.y = est.y();
    odom.child_frame_id = "base_link";
    odom_pub_->publish(odom);
  }

  // params & state
  double                                  marker_size_, box_offset_;
  cv::Mat                                 cam_mat_, dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary>          dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters>  detector_params_;
  std::vector<Eigen::Vector2d>            landmarks_;

  // message_filters subscribers + sync
  mf::Subscriber<CompressedImage>         image_sub1_, image_sub2_;
  std::shared_ptr<
    mf::Synchronizer<
      mf::sync_policies::ApproximateTime<CompressedImage, CompressedImage>
    >
  >                                       sync2_;

  // publisher
  rclcpp::Publisher<Odometry>::SharedPtr  odom_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // allow callbacks (and std::async tasks) to use multiple threads
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<MultiArucoDetector>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
