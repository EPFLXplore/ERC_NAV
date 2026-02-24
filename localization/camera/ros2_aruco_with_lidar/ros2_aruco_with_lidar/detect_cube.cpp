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
#include <vector>
#include <cmath>
#include <string>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>


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
        : rclcpp::Node("detect_cube_node") {
        input_cloud_topic_ = "/ouster_points_aruco";
        output_cloud_topic_ = "/cloud_with_lines";
        aruco_topic_ = "aruco_markers";

        distance_threshold_inliers = 0.05; 
        max_iterations_ = 300;
        t=0.25; //longeur des normes des coté du cube 
        min_inliers_ = 10; // at least 20 points to define a line
        max_lines_ = 3; // at least 2 lines to form a corner

        max_distance_from_aruco_ = 1; // rayon autour de l'ArUco
        angular_tolerance_deg_ = 10; // tolérance angulaire
        
        
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
            aruco_topic_, rclcpp::SensorDataQoS(),
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                on_aruco_markers(msg);
            }
        );
        //RCLCPP_INFO(this->get_logger(), "initialized detect_cube_node");

    }

private:
    void on_aruco_markers(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        aruco_ids_ = msg->marker_ids;
        aruco_poses_ = msg->poses;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000, 
                             "ArUco markers reçus: %zu markers détectés", aruco_ids_.size());
        
        for (size_t i = 0; i < aruco_ids_.size(); ++i) {
            double x = aruco_poses_[i].position.x;
            double y = aruco_poses_[i].position.y;
            double z = aruco_poses_[i].position.z;
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Marker ID %ld: position (%.3f, %.3f, %.3f)", 
                        aruco_ids_[i], x, y, z);
        }
    }
    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
         pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *full_cloud);

        if (aruco_ids_.empty()) {
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_without_lines;
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::MarkerArray points;
        ros2_aruco_interfaces::msg::ArucoMarkers cube_msg;

        for (size_t aruco_idx = 0; aruco_idx < aruco_ids_.size(); ++aruco_idx) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_inliers(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_inliers_2d(new pcl::PointCloud<pcl::PointXYZ>());

            const auto& aruco_pose = aruco_poses_[aruco_idx];
            float aruco_x = static_cast<float>(aruco_pose.position.x);
            float aruco_y = static_cast<float>(aruco_pose.position.y);
            float aruco_z = static_cast<float>(aruco_pose.position.z);
            double aruco_angle_deg = atan2(aruco_y, aruco_x) * 180.0f / M_PI;

            // FILTRAGE SPATIAL : créer sous-nuage pour cet ArUco
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_minus_lines(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& pt : full_cloud->points) {
                float dx = aruco_x;
                float dy = aruco_y;
                float dz = aruco_z;
                double dist = static_cast<double>(sqrt(dx*dx + dy*dy));
                double r_point = sqrt(static_cast<double>(pt.x)*static_cast<double>(pt.x) + static_cast<double>(pt.y)*static_cast<double>(pt.y));
                double pt_angle_deg = atan2(static_cast<double>(pt.y), static_cast<double>(pt.x)) * 180.0 / M_PI;
                
                // Utiliser angDeltaDeg avec offset +100° (comme dans lidar_phi_filter_node)              
                double angle_diff = angDeltaDeg(pt_angle_deg, aruco_angle_deg + 100.0);
                RCLCPP_INFO(this->get_logger(), "angle_diff %.3f", angle_diff);


                // New parameters for line detection depending on distance to get robust lines
                // Parameters got from minimum number of points got from the lidar horizontally at a given distance
                // 0.3 is the width of the aruco tag cubes
                // 0.002967 is the tan(delta_xy) where delta_xy is the minimum horizontal resolution of the lidar
                // We divide by 2 since we want at least this 
                RCLCPP_INFO(this->get_logger(), "dist %.3f", dist);
                RCLCPP_INFO(this->get_logger(), "r_point %.3f", r_point);

                min_inliers_ = static_cast<int>( 0.3/(dist*0.002967) /3);


                // Works well with max_dsitance_from_aruco = 1 at 5 meter and 0.5 at 2.5 meter, hence the relationship
                max_distance_from_aruco_ = static_cast<float>(dist/5);

                // RCLCPP_INFO(this->get_logger(), "min inliners %d", min_inliers_);
                
                if ( abs(dist-r_point) < max_distance_from_aruco_ && angle_diff < angular_tolerance_deg_) {
                    pointcloud_minus_lines->push_back(pt);
                    RCLCPP_INFO(this->get_logger(), "pushback");

                }
                
            }
            if (pointcloud_minus_lines->size() < static_cast<size_t>(min_inliers_)) {
                RCLCPP_INFO(this->get_logger(), "min inliners %zu", static_cast<size_t>(min_inliers_));
                RCLCPP_INFO(this->get_logger(), "pas assez de points");
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
                RCLCPP_INFO(this->get_logger(), "min inliners juste avant %.3f", min_inliers_);

                if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
                    RCLCPP_INFO(this->get_logger(), "detect pas suffisamment de points pour une ligne");
                    cant_find_line_counter++;
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "detecte une ligne avec %zu inliers", inliers->indices.size());

                const float x0 = coefficients->values[0];
                const float y0 = coefficients->values[1];
                const float z0 = coefficients->values[2];
                float dx = coefficients->values[3];
                float dy = coefficients->values[4];
                float dz = coefficients->values[5];
        
                float norm_dir = sqrt(dx * dx + dy * dy + dz * dz);
                
                float ndx = dx / norm_dir;
                float ndy = dy / norm_dir;
                float ndz = dz / norm_dir;

                bool is_parallel = false;
                const float parallel_cos_threshold = 0.99; // 8degre a peut pres
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



                    // pcl::PointCloud<pcl::PointXYZ> projected_inliers;
                    // pcl::ProjectInliers<pcl::PointXYZ> proj;
                    // proj.setModelType(pcl::SACMODEL_LINE);
                    // proj.setInputCloud(detected_lines);
                // proj.setModelCoefficients(coefficients);
                // proj.filter(projected_inliers);

                // std::vector<float> ts;
                // ts.reserve(projected_inliers.size());

                // for (const auto &p : projected_inliers.points) {
                //     float vx = p.x - x0;
                //     float vy = p.y - y0;
                //     float vz = p.z - z0;

                //     float t = vx * dx + vy * dy + vz * dz;
                //     ts.push_back(t);
                // }
                // auto [t_min_it, t_max_it] = std::minmax_element(ts.begin(), ts.end());
                //     float t_min = *t_min_it;
                //     float t_max = *t_max_it;
                
                //geometry_msgs::msg::Point p_first, p_last,p_milieu;
                // p_first.x = x0 + t_min * dx/norm;
                // p_first.y = y0 + t_min * dy/norm;
                // p_first.z = z0 + t_min * dz/norm;

                // p_last.x = x0 + t_max * dx/norm;
                // p_last.y = y0 + t_max * dy/norm;
                // p_last.z = z0 + t_max * dz/norm;

                // p_milieu.z = z0 + (t_max+t_min)/2 * dz/norm;
                // p_milieu.y = y0 + (t_max+t_min)/2 * dy/norm;
                // p_milieu.z = x0 + (t_max+t_min)/2 * dx/norm;
                


                LineDetection det;
                det.x0 = x0; det.y0 = y0; det.z0 = z0;
                det.dx = dx; det.dy = dy; det.dz = dz;
                det.p_first = p_first;
                det.p_last = p_last;
                //det.p_milieu = p_milieu
                //det.t_milieu = (t_max+t_min)/2

                lines.push_back(det);

                // visualization_msgs::msg::Marker line_marker;
                // line_marker.header = cloud_msg.header;
                // line_marker.ns = "detected_lines";
                // line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                // line_marker.action = visualization_msgs::msg::Marker::ADD;
                // line_marker.scale.x = 0.01; // épaisseur de la ligne
                // line_marker.color.r = 0.0f;
                // line_marker.color.g = 1.0f;
                // line_marker.color.b = 0.0f;
                // line_marker.color.a = 1.0f;

                // line_marker.points.push_back(p_first);
                // line_marker.points.push_back(p_last);

                // markers.markers.push_back(line_marker);
                
                RCLCPP_INFO(this->get_logger(), "avantswap111");


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
                // RCLCPP_INFO(this->get_logger(), "forrr pointnummm2");
                
                if(projected_cloud_temp->points.size() < static_cast<size_t>(min_inliers_)) {
                    RCLCPP_INFO(this->get_logger(), "detect pas suffisamment de points pour une ligne 2d AAAAAA");
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
                const float z0_2d = coefficients_2d->values[2];  // devrait être ~0
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


            
            std::vector<geometry_msgs::msg::Point> cube_centres;

            if (!lines_2d.empty()) {
                // Vérifier les paires de lignes proches (< 0.6 m entre les milieux)
                const float distance_threshold_lines = 0.6;
                int non_prendpasligne1=10;
                int non_prendpasligne2=10;
                int non_prendpasligne3=10;
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

                // Marker RViz pour le centre moyen
                visualization_msgs::msg::Marker centre_marker_moyen;
                centre_marker_moyen.header = cloud_msg.header;
                centre_marker_moyen.ns = "cube_center_mean";
                centre_marker_moyen.id = static_cast<int>(aruco_idx);  // ID fixe pour le centre moyen

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

                visualization_msgs::msg::Marker text_marker;
                text_marker.header = cloud_msg.header;
                text_marker.ns = "cube_aruco_id";
                text_marker.id = static_cast<int>(aruco_idx);
                text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::msg::Marker::ADD;
                text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                text_marker.pose.position.x = centre_moyen.x;
                text_marker.pose.position.y = centre_moyen.y;
                text_marker.pose.position.z = centre_moyen.z + 0.15;
                text_marker.pose.orientation.w = 1.0;
                text_marker.scale.z = 0.08;
                text_marker.color.r = 1.0f;
                text_marker.color.g = 1.0f;
                text_marker.color.b = 1.0f;
                text_marker.color.a = 1.0f;
                text_marker.text = "ArUco " + std::to_string(aruco_ids_[aruco_idx]);
                points.markers.push_back(text_marker);

                cube_msg.marker_ids.push_back(aruco_idx);
                geometry_msgs::msg::Pose pose;
                pose.position = centre_moyen;
                pose.orientation.w = 1.0;
                cube_msg.poses.push_back(pose);
                float angle_deg = std::atan2(centre_moyen.y, centre_moyen.x) * 180.0f / static_cast<float>(M_PI);
                cube_msg.ar_angles_list.push_back(angle_deg);
                cube_markers_pub_->publish(cube_msg);
            }

            

            centre_pub_->publish(points);



            if (!lines.empty()) {
                for (size_t i = 0; i < lines.size(); ++i) {
                    const auto &L = lines[i];
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Ligne %zu | Eq: (x,y,z)=(%.3f,%.3f,%.3f)+t*(%.3f,%.3f,%.3f)",
                        i,
                        L.x0, L.y0, L.z0, L.dx, L.dy, L.dz
                    );
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "problemeeeetechnique");

            }

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
    float t ;
    float max_distance_from_aruco_;
    double angular_tolerance_deg_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lines_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;      
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    std::vector<int64_t> aruco_ids_;
    std::vector<geometry_msgs::msg::Pose> aruco_poses_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}