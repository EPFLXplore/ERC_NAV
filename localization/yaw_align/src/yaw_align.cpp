// aruco_pose_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp> //to use Rodrigues !

#include <vector>
#include <map>

using std::placeholders::_1;

// Length of each ArUco marker side in meters:
static constexpr double MARKER_LENGTH = 0.144; 

class ArucoPoseNode : public rclcpp::Node
{
public:
  ArucoPoseNode()
  : Node("aruco_pose_node")
  {
    // 1. Set up a simple world‐frame lookup for marker IDs → 3D positions.
    //    Here, marker IDs 0,1,2,3 correspond to the four predefined 3D points below:
    marker_map_[51] = cv::Point3f(-1.0f, 0.0f, 0.0f);
    marker_map_[52] = cv::Point3f(0.0f, 1.0f, 0.0f);
    marker_map_[53] = cv::Point3f(0.0f, 0.0f, 0.0f);
    marker_map_[54] = cv::Point3f(0.0f, 0.0f, 0.0f);

    // 2. (TODO) Replace these with *your* real camera intrinsics + distortion coefficients:
    //    For instance, load from a YAML, or paste your calibration here.
    //    fx, fy: focal lengths in pixels; cx, cy: principal point; distCoeffs: [k1, k2, p1, p2, k3]
    camera_matrix_ = (cv::Mat_<double>(3,3) <<
      910.77362061,   0.0, 635.58422852,
        0.0, 909.62548828, 355.33377075,
        0.0,   0.0,   1.0
    );
    dist_coeffs_ = (cv::Mat_<double>(1,5) << 
      0.0, 0.0, 0.0, 0.0, 0.0
    );

    // 3. Create the ArUco dictionary + detector parameters:
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    detector_params_ = cv::aruco::DetectorParameters::create();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/NAV/feed_camera_nav_1",
      qos,
      std::bind(&ArucoPoseNode::compressedImageCallback, this, _1)
    );
  }

private:
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    double ar1_x = marker_map_[51].x;
    double ar1_y = marker_map_[51].y;

    double ar2_x = marker_map_[53].x;
    double ar2_y =  marker_map_[53].y;

    const double angle_theory = atan2((ar2_y - ar1_y), (ar2_x - ar1_x));
   
    cv::Mat compressed_data(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat gray = cv::imdecode(compressed_data, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame after imdecode — skipping.");
      return;
    }

    // 3. Detect ArUco markers:
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners; 
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    if (ids.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/, 
                           "No ArUco markers detected.");
    } else {
      // 4. Draw detected markers (outline + ID):
      //cv::aruco::drawDetectedMarkers(frame, corners, ids, cv::Scalar(0,255,0));

      // 5. Estimate each marker’s pose (rvec, tvec):
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        int marker_id = ids[i];

        // 5a. If this ID isn’t in our marker_map_, skip it:
        if (marker_map_.find(marker_id) == marker_map_.end()) {
          RCLCPP_WARN(this->get_logger(), "Marker ID %d not in marker_map_; skipping.", marker_id);
          continue;
        }

        // 5c. Convert rvec → R (3×3 rotation), and get tvec as a 3×1 column:
        // cv::Mat R;
        // cv::Rodrigues(rvecs[i], R);  // rvec → 3×3 rotation matrix

        //ros2 frame = URDF frame = x frowards, y left, z up
        //opencv image frame = x right, y down, z outwards
        
        //obs is in ros2 frame
        x_obs = tvecs[i][2];
        y_obs = (-1.0) * tvecs[i][0];

        
        if(marker_id == 51){
          //detect_tf_1_x = tvecs[i][0]; //x map = opencv x
          //detect_tf_1_y = tvecs[i][1]; //y map = opencv z
          seen51 = true;
          x51 = x_obs;
          y51 = y_obs;
          RCLCPP_INFO(this->get_logger(), "MARKER 51 → (x′=%.2f, y′=%.2f)", x51, y51);          //double yaw_rad_51 = std::atan2(R.at<double>(0,1), R.at<double>(0,0));
          //double yaw_deg_51 = yaw_rad_51 * 180.0 / CV_PI;
         // RCLCPP_INFO(this->get_logger(), "Marker 51 yaw: %.2f°", yaw_deg_51);

        }else if(marker_id == 53){
          //detect_tf_2_x = tvecs[i][0];
          //detect_tf_2_y = tvecs[i][1];
          seen53 = true;
          x53 = x_obs;
          y53 = y_obs;
          RCLCPP_INFO(this->get_logger(), "MARKER 53 → (x′=%.2f, y′=%.2f)", x53, y53);
        }

      } // end for each detected marker

      if(seen51 && seen53){
        double dx_cam = x53 - x51;
        double dy_cam = y53 - y51;
        double angle_cam = std::atan2(dy_cam, dx_cam);
        double yaw_offset = angle_theory - angle_cam; //will have to be corrected later with the URDF to 
        yaw_offset = std::fmod(yaw_offset + M_PI, 2 * M_PI) - M_PI;

        RCLCPP_INFO(
          this->get_logger(),
          "Yaw offset (cam → map): %.2f°",
          yaw_offset * 180.0 / M_PI
        );

        seen51 = false;
        seen53  =false;
      }
    }

    // 6. Show the annotated frame in an OpenCV window (for debugging)
    // cv::imshow("aruco_detections", frame);
    // cv::waitKey(1);
  }

  // Member variables:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat camera_matrix_, dist_coeffs_;
  std::map<int, cv::Point3f> marker_map_;
  double x51 = 0.0;
  double x53 = 0.0;
  double y51 = 0.0;
  double y53 = 0.0;
  bool seen51 = false;
  bool seen53 = false;
  double x_obs;
  double y_obs;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
