#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
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
// #include <pcl/filters/project_inliers.h>




class DetectCubeNode : public rclcpp::Node {
public:
    DetectCubeNode()
        : rclcpp::Node("detect_cube_node") {
        input_cloud_topic_ = "/ouster_points_aruco";
        output_cloud_topic_ = "/cloud_without_lines";
        distance_threshold_inliers = 0.01; 
        max_iterations_ = 300;
        t=0.25; //longeur des normes des coté du cube 
        min_inliers_ = 30; // at least 20 points to define a line
        max_lines_ = 3; // at least 2 lines to form a corner
        
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                detect_lignes(*message);
            }
        );
        lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("detected_lines_markers", qos);

        RCLCPP_INFO(this->get_logger(), "initialized detect_cube_node");

    }

private:

    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_minus_lines(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *pointcloud_minus_lines);
        sensor_msgs::msg::PointCloud2 cloud_without_lines;
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_inliers(new pcl::PointCloud<pcl::PointXYZ>());

        visualization_msgs::msg::MarkerArray markers;
                
        
        visualization_msgs::msg::MarkerArray points;


        struct LineDetection {
            float x0, y0, z0;
            float dx, dy, dz;
            geometry_msgs::msg::Point p_first, p_last;
            // float t_milieu ;
            // geometry_msgs::msg::Point p_milieu, p_last;


        };
        std::vector<LineDetection> lines;
        lines.reserve(static_cast<size_t>(max_lines_));
        RCLCPP_INFO(this->get_logger(), "avant le for");


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

            if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
                RCLCPP_INFO(this->get_logger(), "detect pas suffisamment de points pour une ligne");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "detecte une ligne avec %zu inliers", inliers->indices.size());

            const float x0 = coefficients->values[0];
            const float y0 = coefficients->values[1];
            const float z0 = coefficients->values[2];
            float dx = coefficients->values[3];
            float dy = coefficients->values[4];
            float dz = coefficients->values[5];
            
            

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

            visualization_msgs::msg::Marker line_marker;
            line_marker.header = cloud_msg.header;
            line_marker.ns = "detected_lines";
            line_marker.id = line_idx;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.01; // épaisseur de la ligne
            line_marker.color.r = 0.0f;
            line_marker.color.g = 1.0f;
            line_marker.color.b = 0.0f;
            line_marker.color.a = 1.0f;
            line_marker.pose.orientation.w = 1.0;

            line_marker.points.push_back(p_first);
            line_marker.points.push_back(p_last);

            markers.markers.push_back(line_marker);

            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr remainder(new pcl::PointCloud<pcl::PointXYZ>());
            extract.filter(*remainder);
            pointcloud_minus_lines.swap(remainder);


        }
        sensor_msgs::msg::PointCloud2 cloud_with_all_lines;
        pcl::toROSMsg(*all_inliers, cloud_with_all_lines);
        cloud_with_all_lines.header = cloud_msg.header;
        lines_pub_->publish(cloud_with_all_lines);
        markers_pub_->publish(markers);
        // geometry_msgs::msg::Point p_a_tester_1, p_a_tester_2;

        // for (size_t i = 0; i < lines.size(); ++i) {
        //     p_a_tester_1.x = lines[i].milieu.x + lines[i+1].dx*t_mileu;
        //     p_a_tester_1.y = lines[i].milieu.y + lines[i+1].dy* t_milieu;
        //     p_a_tester_1.z = lines[i].milieu.z + lines[i+1].dz* t_milieu;

        //     p_a_tester_2.x = lines[i+1].milieu.x + lines[i].dx*t_mileu;
        //     p_a_tester_2.y = lines[i+1].milieu.y + lines[i].dy* t_milieu;
        //     p_a_tester_2.z = lines[i+1].milieu.z + lines[i].dz* t_milieu;
        //test= sqrt((p_a_tester_1.x-p_a_tester_2.x)**2+ (p_a_tester_1.y-p_a_tester_2.y)**2+ (p_a_tester_1.z-p_a_tester_2.z)**2);
        //if (test>0.05){
        //      p_a_tester_1.x = lines[i].milieu.x - lines[i+1].dx*t_mileu;
        // //   p_a_tester_1.y = lines[i].milieu.y - lines[i+1].dy* t_milieu;
        // //   p_a_tester_1.z = lines[i].milieu.z - lines[i+1].dz* t_milieu;
        //     }
            // test= sqrt((p_a_tester_1.x-p_a_tester_2.x)**2+ (p_a_tester_1.y-p_a_tester_2.y)**2+ (p_a_tester_1.z-p_a_tester_2.z)**2);
//             if (test>0.05){
        //        p_a_tester_1.x = lines[i].milieu.x - lines[i+1].dx*t_mileu;
        // //     p_a_tester_1.y = lines[i].milieu.y - lines[i+1].dy* t_milieu;
        // //     p_a_tester_1.z = lines[i].milieu.z - lines[i+1].dz* t_milieu;
        //     }




            // }

            // visualization_msgs::msg::Marker points;

            // //line_marker.header = cloud_msg.header;
            // line_marker.ns = "detected_lines";
            // line_marker.type = visualization_msgs::msg::Marker::POINTS;
            // line_marker.action = visualization_msgs::msg::Marker::ADD;
            // line_marker.scale.x = 0.02; // épaisseur de la ligne
            // line_marker.color.r = 0.0f;
            // line_marker.color.g = 1.0f;
            // line_marker.color.b = 0.0f;
            // line_marker.color.a = 1.0f;




            



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

private:
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    double distance_threshold_inliers;
    int max_iterations_;
    int min_inliers_;
    int max_lines_;
    float t ;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lines_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

