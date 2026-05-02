#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>
#include <mutex>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"


// #include <pcl/filters/project_inliers.h>


namespace {

inline double wrap180(double deg) {
    double d = fmod(deg + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}

inline double angDeltaDeg(double a_deg, double b_deg) {
    return fabs(wrap180(b_deg - a_deg));
}
} 

class DetectCubeNode : public rclcpp::Node {
public:
    DetectCubeNode()
        : rclcpp::Node("detect_cube_node"), tf_buffer_(this->get_clock()) {
        this->declare_parameter<double>("distance_threshold_inliers", 0.04);
        this->declare_parameter<int>("max_iterations", 150);
        this->declare_parameter<double>("t", 0.25);
        this->declare_parameter<int>("min_inliers", 10);
        this->declare_parameter<int>("max_lines", 4);
        // Half-width (m) of radial band: |r_point − r_expected| < tol, where r_expected is from map landmark → LiDAR frame.
        this->declare_parameter<double>("max_distance_from_aruco", 0.3);
        // Map frame for landmark_map_pos_* (same as lidar_phi_filter_node / multiview_aruco).
        this->declare_parameter<std::string>("map_frame", "map");
        // Angular gate vs ref_angle (camera bearing + offset); wide default while upstream filter is tight.
        this->declare_parameter<double>("angular_tolerance_deg", 45.0);

        input_cloud_topic_ = "/ouster_points_aruco";
        output_cloud_topic_ = "/cloud_with_lines";
        aruco_topic_ = "aruco_markers";

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        this->get_parameter("distance_threshold_inliers", distance_threshold_inliers);
        this->get_parameter("max_iterations", max_iterations_);
        this->get_parameter("t", t);
        this->get_parameter("min_inliers", min_inliers_);
        this->get_parameter("max_lines", max_lines_);
        this->get_parameter("max_distance_from_aruco", max_distance_from_aruco_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("angular_tolerance_deg", angular_tolerance_deg_);
        min_process_period_ns_ = static_cast<int64_t>(1e9 / 5.0); // 5 Hz max
        last_process_time_ns_ = 0;
        full_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                detect_lignes(*message);
            }
        );

        lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("detected_lines_markers", qos);
        centre_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("centre_cube_beleck", qos);
        cube_markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/cube_markers", qos);

        aruco_subscriber_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            aruco_topic_, qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                on_aruco_markers(msg);
            }
        );
        //RCLCPP_INFO(this->get_logger(), "initialized detect_cube_node");

    }

private:
    void on_aruco_markers(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(aruco_mutex_);
        aruco_ids_ = msg->marker_ids;
        aruco_poses_ = msg->poses;
        aruco_landmark_map_x_ = msg->landmark_map_pos_x;
        aruco_landmark_map_y_ = msg->landmark_map_pos_y;

        // RCLCPP_INFO(this->get_logger(),
        //    "[detect_cube INPUT] received aruco_markers: ids=%zu poses=%zu map_x=%zu map_y=%zu",
        //    aruco_ids_.size(), aruco_poses_.size(),
        //    aruco_landmark_map_x_.size(), aruco_landmark_map_y_.size());
    }

    // Converts map coordinates to cloud frame coordinates and returns the horizontal range in the cloud frame
    bool expected_horizontal_range_from_map(
        double map_x,
        double map_y,
        const std::string &cloud_frame_id,
        double &out_range_xy)
    {
        double dummy_x = 0.0;
        double dummy_y = 0.0;
        double dummy_bearing_deg = 0.0;
        builtin_interfaces::msg::Time zero_stamp;
        return expected_landmark_in_cloud_frame(
            map_x, map_y, cloud_frame_id, zero_stamp,
            dummy_x, dummy_y, out_range_xy, dummy_bearing_deg);
    }

    // Transforms a map landmark into the cloud frame and reports its (x, y), planar range
    // and bearing (deg, atan2(y, x)) all expressed in the cloud frame.
    bool expected_landmark_in_cloud_frame(
        double map_x,
        double map_y,
        const std::string &cloud_frame_id,
        const builtin_interfaces::msg::Time &cloud_stamp,
        double &out_x,
        double &out_y,
        double &out_range_xy,
        double &out_bearing_deg)
    {
        geometry_msgs::msg::PointStamped pin;
        pin.header.frame_id = map_frame_;
        pin.header.stamp = rclcpp::Time(0);
        pin.point.x = map_x;
        pin.point.y = map_y;
        pin.point.z = 0.0;
        geometry_msgs::msg::TransformStamped tf;
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                    return false;
                }
                tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
            } else {
                const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
                if (tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, t, rclcpp::Duration::from_seconds(0.1))) {
                    tf = tf_buffer_.lookupTransform(
                        cloud_frame_id, map_frame_, t, rclcpp::Duration::from_seconds(0.1));
                } else {
                    if (!tf_buffer_.canTransform(
                            cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                        return false;
                    }
                    tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
                }
            }
            geometry_msgs::msg::PointStamped pout;
            tf2::doTransform(pin, pout, tf);
            out_x = pout.point.x;
            out_y = pout.point.y;
            out_range_xy = std::hypot(out_x, out_y);
            out_bearing_deg = std::atan2(out_y, out_x) * 180.0 / M_PI;
            return true;
        } catch (const tf2::TransformException &ex) {
            (void)ex;
            try {
                if (!tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                    return false;
                }
                tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
                geometry_msgs::msg::PointStamped pout;
                tf2::doTransform(pin, pout, tf);
                out_x = pout.point.x;
                out_y = pout.point.y;
                out_range_xy = std::hypot(out_x, out_y);
                out_bearing_deg = std::atan2(out_y, out_x) * 180.0 / M_PI;
                return true;
            } catch (const tf2::TransformException &) {
                return false;
            }
        }
    }

    static bool is_invalid_landmark_xy(double mx, double my)
    {
        constexpr double kSentinel = 999999.0;
        return mx >= kSentinel - 1.0 || my >= kSentinel - 1.0;
    }

    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
        std::vector<int64_t> aruco_ids;
        std::vector<geometry_msgs::msg::Pose> aruco_poses;
        std::vector<double> aruco_landmark_map_x;
        std::vector<double> aruco_landmark_map_y;
        {
            std::lock_guard<std::mutex> lk(aruco_mutex_);
            aruco_ids = aruco_ids_;
            aruco_poses = aruco_poses_;
            aruco_landmark_map_x = aruco_landmark_map_x_;
            aruco_landmark_map_y = aruco_landmark_map_y_;
        }
        if (aruco_ids.empty()) {
            return;
        }

        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns - last_process_time_ns_ < min_process_period_ns_) {
            return;
        }
        last_process_time_ns_ = now_ns;

        full_cloud_->clear();
        pcl::fromROSMsg(cloud_msg, *full_cloud_);

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::MarkerArray points;
        ros2_aruco_interfaces::msg::ArucoMarkers cube_msg;
        // RCLCPP_INFO(this->get_logger(),
        //    "[detect_cube CLOUD] processing cloud frame=%s with aruco_ids=%zu points=%zu",
        //    cloud_msg.header.frame_id.c_str(), aruco_ids.size(), full_cloud_->points.size());

        for (size_t aruco_idx = 0; aruco_idx < aruco_ids.size(); ++aruco_idx) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_inliers(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_inliers_2d(new pcl::PointCloud<pcl::PointXYZ>());

            const auto& aruco_pose = aruco_poses[aruco_idx];
            float aruco_x = static_cast<float>(aruco_pose.position.x);
            float aruco_y = static_cast<float>(aruco_pose.position.y);

            // Compute the expected landmark in the cloud frame (so range and bearing are both
            // consistent with the lidar points). Fall back to the camera pose only if TF or
            // the landmark map position is unavailable.
            double expected_x_cloud = 0.0;
            double expected_y_cloud = 0.0;
            double expected_range_xy = 0.0;
            double expected_bearing_cloud_deg = 0.0;
            bool have_expected_landmark = false;
            if (aruco_idx < aruco_landmark_map_x.size() && aruco_idx < aruco_landmark_map_y.size()) {
                const double mx = aruco_landmark_map_x[aruco_idx];
                const double my = aruco_landmark_map_y[aruco_idx];
                if (!is_invalid_landmark_xy(mx, my)) {
                    have_expected_landmark = expected_landmark_in_cloud_frame(
                        mx, my, cloud_msg.header.frame_id, cloud_msg.header.stamp,
                        expected_x_cloud, expected_y_cloud,
                        expected_range_xy, expected_bearing_cloud_deg);
                }
            }
            const bool have_expected_range = have_expected_landmark;
            const double range_for_inlier_heuristic =
                have_expected_range ? expected_range_xy
                                    : std::max(0.5, std::hypot(static_cast<double>(aruco_x), static_cast<double>(aruco_y)));
            const int dynamic_min_inliers =
                static_cast<int>(0.3 / (range_for_inlier_heuristic * 0.002967) / 3);
            const double radial_tol =
                have_expected_range
                    ? std::min(max_distance_from_aruco_, std::max(0.05, expected_range_xy / 5.0))
                    : max_distance_from_aruco_;
            const int min_inliers = std::max(min_inliers_, dynamic_min_inliers);
            // Angular gate reference: prefer the landmark bearing in the cloud frame so
            // we are not mixing base_link bearings with lidar-frame point angles. If the
            // map TF is unavailable, fall back to the camera pose bearing (best effort).
            const double aruco_angle_deg_base = atan2(aruco_y, aruco_x) * 180.0 / M_PI;
            const double ref_angle = have_expected_landmark
                                         ? wrap180(expected_bearing_cloud_deg)
                                         : wrap180(aruco_angle_deg_base);

            // RCLCPP_INFO(this->get_logger(),
            //    "[detect_cube CANDIDATE] idx=%zu id=%ld camera_xy=(%.3f, %.3f) "
            //    "have_expected=%d expected_xy=(%.3f, %.3f) expected_range=%.3f "
            //    "ref_angle=%.2f deg",
            //    aruco_idx, aruco_ids[aruco_idx], aruco_x, aruco_y,
            //    have_expected_landmark ? 1 : 0,
            //    expected_x_cloud, expected_y_cloud, expected_range_xy,
            //    ref_angle);

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_minus_lines(new pcl::PointCloud<pcl::PointXYZ>());
            size_t stage_in = full_cloud_->points.size();
            size_t stage_pass_radial = 0;
            size_t stage_pass_both = 0;
            for (const auto& pt : full_cloud_->points) {
                double r_point = sqrt(static_cast<double>(pt.x)*static_cast<double>(pt.x) + static_cast<double>(pt.y)*static_cast<double>(pt.y));
                if (have_expected_range && fabs(expected_range_xy - r_point) >= radial_tol) {
                    continue;
                }
                ++stage_pass_radial;

                double pt_angle_deg = atan2(static_cast<double>(pt.y), static_cast<double>(pt.x)) * 180.0 / M_PI;
                if (angDeltaDeg(pt_angle_deg, ref_angle) < angular_tolerance_deg_) {
                    ++stage_pass_both;
                    pointcloud_minus_lines->push_back(pt);
                }
            }
            if (pointcloud_minus_lines->size() < static_cast<size_t>(min_inliers)) {
                // RCLCPP_WARN(this->get_logger(),
                //     "[detect_cube SKIP pre3D] id=%ld in=%zu pass_radial=%zu "
                //     "pass_angle=%zu min_inliers=%d ref_angle=%.2f deg "
                //     "ang_tol=%.2f deg expected_range=%.3f radial_tol=%.3f",
                //     aruco_ids[aruco_idx], stage_in, stage_pass_radial,
                //     stage_pass_both, min_inliers, ref_angle,
                //     angular_tolerance_deg_,
                //     have_expected_range ? expected_range_xy : -1.0,
                //     radial_tol);
                continue;
            }

            struct LineDetection {
                float x0, y0, z0;
                float dx, dy, dz;
                geometry_msgs::msg::Point p_first, p_last;
                // float t_milieu ;
                // geometry_msgs::msg::Point p_milieu, p_last;


            };
            std::vector<LineDetection> lines;
            lines.reserve(static_cast<size_t>(max_lines_));
            // RCLCPP_INFO(this->get_logger(), "avant le for");

            unsigned int cant_find_line_counter = 0;
            for (int line_idx = 0; line_idx < max_lines_; ++line_idx) {

                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_LINE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(distance_threshold_inliers);
                seg.setMaxIterations(max_iterations_);
                seg.setInputCloud(pointcloud_minus_lines);

                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
                seg.segment(*inliers, *coefficients);
                // RCLCPP_INFO(this->get_logger(), "min inliners juste avant %.3f", min_inliers_);

                if (inliers->indices.size() < static_cast<size_t>(min_inliers)) {
                    // RCLCPP_INFO(this->get_logger(), "detect pas suffisamment de points pour une ligne");
                    cant_find_line_counter++;
                    break;
                }
                // RCLCPP_INFO(this->get_logger(), "detecte une ligne avec %zu inliers", inliers->indices.size());

                const float x0 = coefficients->values[0];
                const float y0 = coefficients->values[1];
                const float z0 = coefficients->values[2];
                float dx = coefficients->values[3];
                float dy = coefficients->values[4];
                float dz = coefficients->values[5];

                // if (!lines.empty()){
                //     for (const auto &L : lines) {
                //         float norm_existing = sqrt(L.dx * L.dx + L.dy * L.dy + L.dz * L.dz);
                //    is_parallel     float ex = L.dx / norm_existing;
                //         float ey = L.dy / norm_existing;
                //         float ez = L.dz / norm_existing;

                //         float dot = ndx * ex + ndy * ey + ndz * ez;
                //         if (fabs(dot) > parallel_cos_threshold) {
                //             is_parallel = true;
                //             RCLCPP_INFO(this->get_logger(), "ligneparalele");

                //             break;
                //         }

                //     }
                // }

                // Extraire d'abord tous les inliers de cette ligne
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(pointcloud_minus_lines);
                extract.setIndices(inliers);
                extract.setNegative(false);
                pcl::PointCloud<pcl::PointXYZ>::Ptr detected_lines(new pcl::PointCloud<pcl::PointXYZ>());
                extract.filter(*detected_lines);

                
                *all_inliers += *detected_lines;

                geometry_msgs::msg::Point p_first, p_last;
                //je prend ma ligne qui commence a xo,y,zo 
                //a changer parce que ca donne des segment decallé mais sur la bonne direction juste pour check 
                float norm = sqrt(dx*dx + dy*dy + dz*dz);
                p_first.x = x0 - dx*t/norm/2;
                p_first.y = y0 - dy*t/norm/2;
                p_first.z = z0 - dz*t/norm/2;
                p_last.x = p_first.x + dx*t/norm;
                p_last.y= p_first.y + dy*t/norm;
                p_last.z= p_first.z + dz*t/norm;


                


                LineDetection det;
                det.x0 = x0; det.y0 = y0; det.z0 = z0;
                det.dx = dx; det.dy = dy; det.dz = dz;
                det.p_first = p_first;
                det.p_last = p_last;
                //det.p_milieu = p_milieu
                //det.t_milieu = (t_max+t_min)/2

                lines.push_back(det);




                extract.setNegative(true);
                pcl::PointCloud<pcl::PointXYZ>::Ptr remainder(new pcl::PointCloud<pcl::PointXYZ>());
                extract.filter(*remainder);
                pointcloud_minus_lines.swap(remainder);


            }

            // if(cant_find_line_counter >= ){
            //     return;
            // }


            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                projected_cloud->reserve(all_inliers->size());
            for (const auto& point : all_inliers->points) {
                    pcl::PointXYZ p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0.0;  
                    projected_cloud->push_back(p);
                    // RCLCPP_INFO(this->get_logger(), "forrr point");
                    // RCLCPP_INFO(this->get_logger(), "all_inliers size: %zu", all_inliers->size());
                    // RCLCPP_INFO(this->get_logger(), "forrr point", point.x, point.y, point.z);
                }

            
            std::vector<LineDetection> lines_2d;
            lines_2d.reserve(static_cast<size_t>(max_lines_));


                
            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
            *projected_cloud_temp = *projected_cloud;
            
            for (int line_idx_2d = 0; line_idx_2d < 2; ++line_idx_2d) {
                if(projected_cloud_temp->points.size() < static_cast<size_t>(min_inliers)) {
                    break;
                }

                pcl::SACSegmentation<pcl::PointXYZ> seg_2d;
                seg_2d.setOptimizeCoefficients(true);
                seg_2d.setModelType(pcl::SACMODEL_LINE);
                seg_2d.setMethodType(pcl::SAC_RANSAC);
                seg_2d.setDistanceThreshold(distance_threshold_inliers);
                seg_2d.setMaxIterations(max_iterations_);
                seg_2d.setInputCloud(projected_cloud_temp);
                // RCLCPP_INFO(this->get_logger(), "forrr pointnummm2");

                pcl::PointIndices::Ptr inliers_2d(new pcl::PointIndices());
                pcl::ModelCoefficients::Ptr coefficients_2d(new pcl::ModelCoefficients());
                seg_2d.segment(*inliers_2d, *coefficients_2d);
                // RCLCPP_INFO(this->get_logger(), "forrr pointnummm2");
                const float x0_2d = coefficients_2d->values[0];
                const float y0_2d = coefficients_2d->values[1];
                float dx_2d = coefficients_2d->values[3];
                float dy_2d = coefficients_2d->values[4];
                float dz_2d = coefficients_2d->values[5];  // devrait être ~0
                
                // Normaliser la direction
                float norm_2d = sqrt(dx_2d * dx_2d + dy_2d * dy_2d + dz_2d * dz_2d);
                geometry_msgs::msg::Point p_first_2d, p_last_2d;
                p_first_2d.x = x0_2d - dx_2d * t / norm_2d / 2;
                p_first_2d.y = y0_2d - dy_2d * t / norm_2d / 2;
                p_first_2d.z = 0.0f;
                p_last_2d.x = p_first_2d.x + dx_2d * t / norm_2d;
                p_last_2d.y = p_first_2d.y + dy_2d * t / norm_2d;
                p_last_2d.z = 0.0f;
                
                LineDetection det_2d;
                det_2d.x0 = x0_2d; det_2d.y0 = y0_2d; det_2d.z0 = 0.0f;
                det_2d.dx = dx_2d; det_2d.dy = dy_2d; det_2d.dz = 0.0f;
                det_2d.p_first = p_first_2d;
                det_2d.p_last = p_last_2d;
                lines_2d.push_back(det_2d);
                // RCLCPP_INFO(this->get_logger(), "forrr pointnummm2");

                // Créer un marker pour cette ligne 2D
                visualization_msgs::msg::Marker line_marker_2d;
                line_marker_2d.header = cloud_msg.header;
                line_marker_2d.ns = "detected_lines_2d";
                line_marker_2d.id = line_idx_2d;
                line_marker_2d.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line_marker_2d.action = visualization_msgs::msg::Marker::ADD;
                line_marker_2d.lifetime = rclcpp::Duration::from_seconds(0.5);
                line_marker_2d.scale.x = 0.015;  // Un peu plus épais pour voir la différence
                line_marker_2d.color.r = 1.0f;  // Rouge pour les lignes 2D
                line_marker_2d.color.g = 0.0f;
                line_marker_2d.color.b = 0.0f;
                line_marker_2d.color.a = 1.0f;
                
                line_marker_2d.points.push_back(p_first_2d);
                line_marker_2d.points.push_back(p_last_2d);
                
                markers.markers.push_back(line_marker_2d);
                // RCLCPP_INFO(this->get_logger(), "forrr pointnummm2");
                
                // Retirer les inliers du nuage temporaire
                pcl::ExtractIndices<pcl::PointXYZ> extract_2d;
                extract_2d.setInputCloud(projected_cloud_temp);
                extract_2d.setIndices(inliers_2d);  
                extract_2d.setNegative(false);
                pcl::PointCloud<pcl::PointXYZ>::Ptr detected_lines_2d(new pcl::PointCloud<pcl::PointXYZ>());
                extract_2d.filter(*detected_lines_2d);

                
                *all_inliers_2d += *detected_lines_2d;




                extract_2d.setNegative(true);
                pcl::PointCloud<pcl::PointXYZ>::Ptr remainder_2d(new pcl::PointCloud<pcl::PointXYZ>());
                extract_2d.filter(*remainder_2d);
                projected_cloud_temp.swap(remainder_2d);
            }
            sensor_msgs::msg::PointCloud2 cloud_with_all_lines;
            pcl::toROSMsg(*all_inliers_2d, cloud_with_all_lines);
            cloud_with_all_lines.header = cloud_msg.header;
            lines_pub_->publish(cloud_with_all_lines);
            markers_pub_->publish(markers);

            // RCLCPP_INFO(this->get_logger(),
            //    "[detect_cube LINES] id=%ld in=%zu radial=%zu angle=%zu "
            //    "3d_lines=%zu 3d_inliers=%zu projected=%zu 2d_lines=%zu min_inliers=%d",
            //    aruco_ids[aruco_idx], stage_in, stage_pass_radial,
            //    stage_pass_both, lines.size(), all_inliers->size(),
            //    projected_cloud->size(), lines_2d.size(), min_inliers);

            if (lines_2d.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "[detect_cube SKIP no2D] id=%ld projected=%zu min_inliers=%d",
                    aruco_ids[aruco_idx], projected_cloud->size(), min_inliers);
            }


            
            std::vector<geometry_msgs::msg::Point> cube_centres;

            if (!lines_2d.empty()) {
                // Vérifier les paires de lignes proches (< 0.6 m entre les milieux)
                for (size_t i = 0; i < lines_2d.size(); ++i) {
                    
                    // RCLCPP_INFO(this->get_logger(), "forrr detectlinesss1");
                    geometry_msgs::msg::Point p_centre;

                    const auto &L1 = lines_2d[i];
                    // milieu de la ligne i
                    geometry_msgs::msg::Point mid1;
                    mid1.x = 0.5 * (L1.p_first.x + L1.p_last.x);
                    mid1.y = 0.5 * (L1.p_first.y + L1.p_last.y);
                    mid1.z = 0.5 * (L1.p_first.z + L1.p_last.z);

                    // vecteur normal:
                    float norm_dir = sqrt(L1.dx * L1.dx + L1.dy * L1.dy + L1.dz * L1.dz);
                    float ndx = -L1.dy / norm_dir;
                    float ndy = L1.dx / norm_dir;

                    // vérifier l'orientation du vecteur normal
                    //vecteur lidar->milieu
                    float v_lidar_mid_x = mid1.x;
                    float v_lidar_mid_y = mid1.y;

                    float dot_product = ndx * v_lidar_mid_x + ndy * v_lidar_mid_y;
                    if (dot_product < 0) {
                        // le vecteur normal pointe vers le lidar, on l'inverse
                        ndx = -ndx;
                        ndy = -ndy;
                    }

                    p_centre.x = mid1.x + ndx * t / 2;
                    p_centre.y = mid1.y + ndy * t / 2;
                    p_centre.z = lines_2d[i].z0;

                    cube_centres.push_back(p_centre);

                }


                // calculer le centre moyen calcule pour chaque ligne
                geometry_msgs::msg::Point centre_moyen;
                centre_moyen.x = 0.0;
                centre_moyen.y = 0.0;
                centre_moyen.z = 0.0;
                for (const auto& centre : cube_centres) {
                    centre_moyen.x += centre.x;
                    centre_moyen.y += centre.y;
                    centre_moyen.z += centre.z;
                }

                centre_moyen.x /= static_cast<float>(cube_centres.size());
                centre_moyen.y /= static_cast<float>(cube_centres.size());
                centre_moyen.z /= static_cast<float>(cube_centres.size());

                // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "marker ids: %s", std::to_string(aruco_ids[aruco_idx]).c_str());
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "Centre position in frame %s: (%.3f, %.3f, %.3f)",
                //            cloud_msg.header.frame_id.c_str(), centre_moyen.x, centre_moyen.y, centre_moyen.z);

                // Default: keep data in original lidar frame
                std::string target_frame = cloud_msg.header.frame_id;
                // std::string target_frame = "base_link";

                
                // Transform chain: Lidar_v2_1 -> Service_Module_v5_1 -> base_link
                // TF2 should handle this automatically, but we need to check each step
                try {
                    // First try direct transform (TF2 should chain automatically)
                    if (tf_buffer_.canTransform("base_link", 
                                               cloud_msg.header.frame_id,
                                               tf2::TimePointZero,
                                               tf2::durationFromSec(0.05))) {
                        auto transform = tf_buffer_.lookupTransform(
                            "base_link",
                            cloud_msg.header.frame_id,
                            tf2::TimePointZero
                        );
                        
                        // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "Direct transform available! Translation: (%.3f, %.3f, %.3f)",
                        //            transform.transform.translation.x,
                        //            transform.transform.translation.y,
                        //            transform.transform.translation.z);

                        geometry_msgs::msg::PointStamped point_in, point_out;
                        point_in.header.frame_id = cloud_msg.header.frame_id;
                        point_in.header.stamp = rclcpp::Time(0);
                        point_in.point = centre_moyen;
                        tf2::doTransform(point_in, point_out, transform);
                        centre_moyen = point_out.point;
                        target_frame = "base_link";
                        
                        // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "AFTER transform - frame: %s, pos: (%.3f, %.3f, %.3f)",
                        //            target_frame.c_str(), centre_moyen.x, centre_moyen.y, centre_moyen.z);
                    } else {
                        // If direct transform fails, try manual chain via Service_Module_v5_1
                        // RCLCPP_WARN(this->get_logger(), "Direct transform not available, trying via Service_Module_v5_1");
                        
                        geometry_msgs::msg::PointStamped point_lidar, point_service, point_base;
                        point_lidar.header.frame_id = cloud_msg.header.frame_id;
                        point_lidar.header.stamp = rclcpp::Time(0);
                        point_lidar.point = centre_moyen;
                        
                        // Step 1: Lidar_v2_1 -> Service_Module_v5_1
                        if (tf_buffer_.canTransform("Service_Module_v5_1", 
                                                   cloud_msg.header.frame_id,
                                                   tf2::TimePointZero,
                                                   tf2::durationFromSec(0.05))) {
                            auto transform1 = tf_buffer_.lookupTransform(
                                "Service_Module_v5_1",
                                cloud_msg.header.frame_id,
                                tf2::TimePointZero
                            );
                            
                            // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "Transform 1 (%s -> Service_Module_v5_1): (%.3f, %.3f, %.3f)",
                            //            cloud_msg.header.frame_id.c_str(),
                            //            transform1.transform.translation.x,
                            //            transform1.transform.translation.y,
                            //            transform1.transform.translation.z);

                            tf2::doTransform(point_lidar, point_service, transform1);
                            
                            // Step 2: Service_Module_v5_1 -> base_link
                            if (tf_buffer_.canTransform("base_link", 
                                                       "Service_Module_v5_1",
                                                       tf2::TimePointZero,
                                                       tf2::durationFromSec(0.05))) {
                                auto transform2 = tf_buffer_.lookupTransform(
                                    "base_link",
                                    "Service_Module_v5_1",
                                    tf2::TimePointZero
                                );
                                
                                // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "Transform 2 (Service_Module_v5_1 -> base_link): (%.3f, %.3f, %.3f)",
                                //            transform2.transform.translation.x,
                                //            transform2.transform.translation.y,
                                //            transform2.transform.translation.z);

                                tf2::doTransform(point_service, point_base, transform2);
                                centre_moyen = point_base.point;
                                target_frame = "base_link";
                                
                                // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500, "AFTER chained transform - frame: %s, pos: (%.3f, %.3f, %.3f)",
                                //            target_frame.c_str(), centre_moyen.x, centre_moyen.y, centre_moyen.z);
                            } else {
                                // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                                //            "Transform Service_Module_v5_1 -> base_link not available. "
                                //            "Check: ros2 run tf2_ros tf2_echo base_link Service_Module_v5_1");
                                centre_moyen = point_service.point;
                                target_frame = "Service_Module_v5_1";
                            }
                        } else {
                            // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                            //            "Transform %s -> Service_Module_v5_1 not available. "
                            //            "Check: ros2 run tf2_ros tf2_echo Service_Module_v5_1 %s",
                            //            cloud_msg.header.frame_id.c_str(), cloud_msg.header.frame_id.c_str());
                        }
                    }
                } catch (const tf2::TransformException &ex) {
                    // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                    //            "Transform error: %s. Keeping data in %s frame.",
                    //            ex.what(), cloud_msg.header.frame_id.c_str());
                    (void)ex;
                }


                // Marker RViz pour le centre moyen
                visualization_msgs::msg::Marker centre_marker_moyen;
                centre_marker_moyen.header.stamp = cloud_msg.header.stamp;
                centre_marker_moyen.header.frame_id = target_frame;
                centre_marker_moyen.ns = "cube_center_mean";
                centre_marker_moyen.id = static_cast<int>(aruco_ids[aruco_idx]);  // ID fixe pour le centre moyen

                centre_marker_moyen.type = visualization_msgs::msg::Marker::SPHERE;
                centre_marker_moyen.action = visualization_msgs::msg::Marker::ADD;
                centre_marker_moyen.pose.position = centre_moyen;
                centre_marker_moyen.pose.orientation.x = 0.0;
                centre_marker_moyen.pose.orientation.y = 0.0;
                centre_marker_moyen.pose.orientation.z = 0.0;
                centre_marker_moyen.pose.orientation.w = 1.0;
                centre_marker_moyen.scale.x = 0.1;
                centre_marker_moyen.scale.y = 0.1;
                centre_marker_moyen.scale.z = 0.1;
                centre_marker_moyen.color.r = 0.0f;
                centre_marker_moyen.color.g = 1.0f;
                centre_marker_moyen.color.b = 0.0f;
                centre_marker_moyen.color.a = 1.0f;
                points.markers.push_back(centre_marker_moyen);

                // visualization_msgs::msg::Marker text_marker;
                // text_marker.header = cloud_msg.header;
                // text_marker.ns = "cube_aruco_id";
                // text_marker.id = static_cast<int>(aruco_ids_[aruco_idx]);
                // text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                // text_marker.action = visualization_msgs::msg::Marker::ADD;
                // text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                // text_marker.pose.position.x = centre_moyen.x;
                // text_marker.pose.position.y = centre_moyen.y;
                // text_marker.pose.position.z = centre_moyen.z + 0.15;
                // text_marker.pose.orientation.w = 1.0;
                // text_marker.scale.z = 0.08;
                // text_marker.color.r = 1.0f;
                // text_marker.color.g = 1.0f;
                // text_marker.color.b = 1.0f;
                // text_marker.color.a = 1.0f;
                // text_marker.text = "ArUco " + std::to_string(aruco_ids_[aruco_idx]);
                // points.markers.push_back(text_marker);

                cube_msg.marker_ids.push_back(aruco_ids[aruco_idx]);
                geometry_msgs::msg::Pose pose;
                pose.position = centre_moyen;
                pose.orientation.w = 1.0;
                cube_msg.poses.push_back(pose);
                float angle_deg = std::atan2(centre_moyen.y, centre_moyen.x) * 180.0f / static_cast<float>(M_PI);
                cube_msg.ar_angles_list.push_back(angle_deg);
                // RCLCPP_INFO(this->get_logger(),
                //    "[detect_cube QUEUE] id=%ld center_base=(%.3f, %.3f, %.3f) "
                //    "angle=%.2f deg batch_so_far=%zu",
                //    aruco_ids[aruco_idx],
                //    centre_moyen.x, centre_moyen.y, centre_moyen.z,
                //    angle_deg, cube_msg.marker_ids.size());
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[detect_cube SKIP no_center] id=%ld lines_2d=%zu",
                    aruco_ids[aruco_idx], lines_2d.size());
            }

            

            centre_pub_->publish(points);



            if (lines.empty()) {
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                //     "[detect_cube] id=%ld no 3D line found despite %zu candidate points",
                //     aruco_ids[aruco_idx], pointcloud_minus_lines->size());
            }

        }

        /* One publish per cloud: pose_estimator_lidar_node rate-limits /cube_markers after
         * init; multiple publishes in the same callback caused only the first (partial)
         * message to be processed, so valid_markers stayed 1 with the same id. */
        if (!cube_msg.marker_ids.empty()) {
            cube_msg.header.stamp = cloud_msg.header.stamp;
            cube_msg.header.frame_id = "base_link";
            RCLCPP_INFO(this->get_logger(),
                "[detect_cube PUBLISH BATCH] markers=%zu (one message per cloud)",
                cube_msg.marker_ids.size());
            cube_markers_pub_->publish(cube_msg);
        }
    }
    
private:
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string aruco_topic_;
    double distance_threshold_inliers;
    int max_iterations_;
    int min_inliers_;
    int max_lines_;
    double t;
    double max_distance_from_aruco_;
    double angular_tolerance_deg_;
    std::string map_frame_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lines_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;      
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    std::vector<int64_t> aruco_ids_;
    std::vector<geometry_msgs::msg::Pose> aruco_poses_;
    std::vector<double> aruco_landmark_map_x_;
    std::vector<double> aruco_landmark_map_y_;
    std::mutex aruco_mutex_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;

    int64_t min_process_period_ns_;
    int64_t last_process_time_ns_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}