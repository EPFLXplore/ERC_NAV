/**
 * @file traversability_filter.cpp
 * @brief ROS 2 node for converting raw LiDAR point clouds into a local traversability height map.
 *
 * Input:
 * - Subscribes to `input_cloud_topic`, default: `/ouster/points`
 *
 * Output:
 * - Publishes filtered terrain cloud on `output_cloud_topic`, default: `/filtered_pointcloud`
 * - Optional/debug outputs:
 *   - `/filtered_pointcloud_visual_high_res`
 *   - `/filtered_pointcloud_visual_low_res`
 *   - `/pointcloud_2_laserscan`
 *
 * This node consumes XYZ fields from incoming PointCloud2 LiDAR data, transforms points into the map frame, filters points by range
 * and by a LiDAR/source-frame robot body exclusion box, then bins them into a robot-centered 2D height
 * grid. Each occupied grid cell stores the observed maximum height below the configured
 * obstacle-height threshold, producing a downsampled point cloud representation of the local terrain.
 *
 * The filter supports optional sensor-frame voxel downsampling and point striding to reduce CPU load
 * on dense LiDAR clouds. TF is used to locate both the LiDAR/source frame and the robot base frame,
 * allowing the output map to stay centered around the robot while being published in the map frame.
 *
 * Typical use:
 * - Remove far LiDAR points outside the configured sensor range.
 * - Remove self-returns inside the rover body box in LiDAR/source-frame coordinates.
 * - Build a lightweight local height map for navigation or traversability analysis.
 * - Reduce dense 3D LiDAR data into a lower-cost terrain representation.
 */

#include "utility.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>

class TraversabilityFilter : public rclcpp::Node {
    
private:

    // ROS subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud;
    // ROS publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualHiRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualLowRes;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan;
    // Point Cloud
    // Raw sensor cloud deliberately uses XYZ: traversability does not consume
    // return intensity, and the Ouster may be configured with point_type: xyz.
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudRaw;
    // Internal/output clouds retain PointType (XYZI) so the published filtered
    // cloud remains compatible with traversability_map and existing consumers.
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
    pcl::PointCloud<PointType>::Ptr laserCloudWork; // optional voxel output (reused buffer)
    pcl::PointCloud<PointType>::Ptr laserCloudDenoised; // optional radius outlier output (reused buffer)
    pcl::PointCloud<PointType>::Ptr laserCloudOut; // filtered and downsampled point cloud
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles; // cloud for saving points that are classified as obstables, convert them to laser scan
    // Transform Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // A few points
    PointType robotPoint;
    PointType localMapOrigin;
    // point cloud saved as N_SCAN * Horizon_SCAN form
    vector<vector<PointType>> laserCloudMatrix;
    // Matrice
    cv::Mat obstacleMatrix; // -1 - invalid, 0 - free, 1 - obstacle
    cv::Mat rangeMatrix; // -1 - invalid, >0 - valid range value
    // laser scan message
    sensor_msgs::msg::LaserScan laserScan;
    // for downsample
    float **minHeight;
    float **maxHeight;
    float **sumHeight;
    int **heightCount;
    bool **obstFlag;
    bool **initFlag;
    int cloudWidth = 0;
    int cloudHeight = 0;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string source_frame_;
    bool use_msg_frame_id_;
    /// Process every k-th point after optional voxel (1 = all points). Cheap CPU win on dense LiDAR.
    int point_stride_{1};
    /// Voxel leaf in **sensor** frame (m); 0 = disabled. Runs before transform; use ~mapResolution–2× for speed.
    float sensor_voxel_leaf_m_{0.0f};
    float noise_radius_m_{0.0f};
    int noise_min_neighbors_{2};
    float max_lidar_z_m_{0.5f};
    bool use_lidar_body_box_filter_{lidarBodyBoxFilterEnabled};
    float lidar_body_box_min_x_m_{lidarBodyBoxMinX};
    float lidar_body_box_max_x_m_{lidarBodyBoxMaxX};
    float lidar_body_box_min_y_m_{lidarBodyBoxMinY};
    float lidar_body_box_max_y_m_{lidarBodyBoxMaxY};
    float lidar_body_box_min_z_m_{lidarBodyBoxMinZ};
    float lidar_body_box_max_z_m_{lidarBodyBoxMaxZ};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;


public:
    TraversabilityFilter() : Node("traversability_filter") {
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/ouster/points");
        output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "/filtered_pointcloud");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        source_frame_ = this->declare_parameter<std::string>("source_frame", "Lidar_v2_1");
        use_msg_frame_id_ = this->declare_parameter<bool>("use_msg_frame_id", true);
        point_stride_ = static_cast<int>(this->declare_parameter<int>("point_stride", 1));
        if (point_stride_ < 1) {
            point_stride_ = 1;
        }
        sensor_voxel_leaf_m_ = static_cast<float>(this->declare_parameter<double>("sensor_voxel_leaf_m", 0.0));
        noise_radius_m_ = static_cast<float>(this->declare_parameter<double>("noise_radius_m", 0.0));
        noise_min_neighbors_ = this->declare_parameter<int>("noise_min_neighbors", 2);
        if (noise_radius_m_ < 0.0f) {
            noise_radius_m_ = 0.0f;
        }
        if (noise_min_neighbors_ < 1) {
            noise_min_neighbors_ = 1;
        }
        max_lidar_z_m_ = static_cast<float>(this->declare_parameter<double>("max_lidar_z_m", 0.5));
        use_lidar_body_box_filter_ = this->declare_parameter<bool>("use_lidar_body_box_filter", lidarBodyBoxFilterEnabled);
        lidar_body_box_min_x_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_min_x_m", lidarBodyBoxMinX));
        lidar_body_box_max_x_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_max_x_m", lidarBodyBoxMaxX));
        lidar_body_box_min_y_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_min_y_m", lidarBodyBoxMinY));
        lidar_body_box_max_y_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_max_y_m", lidarBodyBoxMaxY));
        lidar_body_box_min_z_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_min_z_m", lidarBodyBoxMinZ));
        lidar_body_box_max_z_m_ = static_cast<float>(this->declare_parameter<double>("lidar_body_box_max_z_m", lidarBodyBoxMaxZ));

        if (point_stride_ > 1 || sensor_voxel_leaf_m_ > 1e-6f || noise_radius_m_ > 1e-6f) {
            RCLCPP_INFO(
                this->get_logger(),
                "Filter downsampling: point_stride=%d sensor_voxel_leaf_m=%.3f (0=off), noise_radius_m=%.3f noise_min_neighbors=%d, max_lidar_z_m=%.3f",
                point_stride_,
                static_cast<double>(sensor_voxel_leaf_m_),
                static_cast<double>(noise_radius_m_),
                noise_min_neighbors_,
                static_cast<double>(max_lidar_z_m_));
        }
        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR body box filter: %s x=[%.2f, %.2f] y=[%.2f, %.2f] z=[%.2f, %.2f] m in source frame",
            use_lidar_body_box_filter_ ? "on" : "off",
            lidar_body_box_min_x_m_, lidar_body_box_max_x_m_,
            lidar_body_box_min_y_m_, lidar_body_box_max_y_m_,
            lidar_body_box_min_z_m_, lidar_body_box_max_z_m_);

        auto qos = rclcpp::SensorDataQoS();

        subCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            std::bind(&TraversabilityFilter::cloudHandler, this, std::placeholders::_1));

        pubCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        // pubCloudVisualHiRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_high_res", 5);
        // pubCloudVisualLowRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_low_res", 5);
        // pubLaserScan = this->create_publisher<sensor_msgs::msg::LaserScan>("/pointcloud_2_laserscan", 5);

        allocateMemory();

        pointcloud2laserscanInitialization();

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TraversabilityFilter::onSetParameters, this, std::placeholders::_1));
    }

    // Live parameter tuning (ros2 param set / rqt_reconfigure). The node spins single-threaded, so
    // updating members between cloud callbacks is safe. Validate the batch first, then apply.
    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            const std::string &name = p.get_name();
            std::string reason;
            if (name == "input_cloud_topic" || name == "output_cloud_topic" ||
                name == "map_frame" || name == "base_frame") {
                reason = name + " is structural (topics/frames); restart the node to change it";
            } else if (name == "point_stride" && p.as_int() < 1) {
                reason = "point_stride must be >= 1";
            } else if ((name == "sensor_voxel_leaf_m" || name == "noise_radius_m") && p.as_double() < 0.0) {
                reason = name + " must be >= 0 (0 disables)";
            } else if (name == "noise_min_neighbors" && p.as_int() < 1) {
                reason = "noise_min_neighbors must be >= 1";
            }
            if (!reason.empty()) {
                result.successful = false;
                result.reason = reason;
                return result;
            }
        }

        for (const auto &p : params) {
            const std::string &name = p.get_name();
            if      (name == "point_stride")              point_stride_ = static_cast<int>(p.as_int());
            else if (name == "sensor_voxel_leaf_m")       sensor_voxel_leaf_m_ = static_cast<float>(p.as_double());
            else if (name == "noise_radius_m")            noise_radius_m_ = static_cast<float>(p.as_double());
            else if (name == "noise_min_neighbors")       noise_min_neighbors_ = static_cast<int>(p.as_int());
            else if (name == "max_lidar_z_m")             max_lidar_z_m_ = static_cast<float>(p.as_double());
            else if (name == "use_msg_frame_id")          use_msg_frame_id_ = p.as_bool();
            else if (name == "source_frame")              source_frame_ = p.as_string();
            else if (name == "use_lidar_body_box_filter") use_lidar_body_box_filter_ = p.as_bool();
            else if (name == "lidar_body_box_min_x_m")    lidar_body_box_min_x_m_ = static_cast<float>(p.as_double());
            else if (name == "lidar_body_box_max_x_m")    lidar_body_box_max_x_m_ = static_cast<float>(p.as_double());
            else if (name == "lidar_body_box_min_y_m")    lidar_body_box_min_y_m_ = static_cast<float>(p.as_double());
            else if (name == "lidar_body_box_max_y_m")    lidar_body_box_max_y_m_ = static_cast<float>(p.as_double());
            else if (name == "lidar_body_box_min_z_m")    lidar_body_box_min_z_m_ = static_cast<float>(p.as_double());
            else if (name == "lidar_body_box_max_z_m")    lidar_body_box_max_z_m_ = static_cast<float>(p.as_double());
            else continue;
            RCLCPP_INFO(this->get_logger(), "Runtime update: %s = %s", name.c_str(), p.value_to_string().c_str());
        }
        return result;
    }

    void allocateMemory(){
        laserCloudRaw.reset(new pcl::PointCloud<pcl::PointXYZ>());
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudWork.reset(new pcl::PointCloud<PointType>());
        laserCloudDenoised.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32SC1, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32FC1, cv::Scalar::all(FLT_MAX));

        minHeight = new float*[filterHeightMapArrayLength];
        maxHeight = new float*[filterHeightMapArrayLength];
        sumHeight = new float*[filterHeightMapArrayLength];
        heightCount = new int*[filterHeightMapArrayLength];
        obstFlag = new bool*[filterHeightMapArrayLength];
        initFlag = new bool*[filterHeightMapArrayLength];

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            minHeight[i] = new float[filterHeightMapArrayLength];
            maxHeight[i] = new float[filterHeightMapArrayLength];
            sumHeight[i] = new float[filterHeightMapArrayLength];
            heightCount[i] = new int[filterHeightMapArrayLength];
            obstFlag[i] = new bool[filterHeightMapArrayLength];
            initFlag[i] = new bool[filterHeightMapArrayLength];
        }
    }

    void resetParameters(){
        laserCloudRaw->clear();
        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();
        
        for (int i = 0; i < N_SCAN; ++i)
            fill(laserCloudMatrix[i].begin(), laserCloudMatrix[i].end(), PointType());
        
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            fill(minHeight[i], minHeight[i] + filterHeightMapArrayLength, FLT_MAX);
            fill(maxHeight[i], maxHeight[i] + filterHeightMapArrayLength, -FLT_MAX);
            fill(sumHeight[i], sumHeight[i] + filterHeightMapArrayLength, 0.0f);
            fill(heightCount[i], heightCount[i] + filterHeightMapArrayLength, 0);
            fill(obstFlag[i], obstFlag[i] + filterHeightMapArrayLength, false);
            fill(initFlag[i], initFlag[i] + filterHeightMapArrayLength, false);
        }
    }

    ~TraversabilityFilter(){}


    std::chrono::time_point<std::chrono::high_resolution_clock> last_cloud_time_;

    // Do not deserialize as PointXYZI: raw input intensity is not used by this
    // filter, and a valid Ouster `point_type: xyz` cloud has no such field.
    // Populate the internal XYZI cloud with a neutral intensity solely for the
    // output schema expected by downstream nodes.
    void load_input_cloud_xyz(const sensor_msgs::msg::PointCloud2 &msg) {
        pcl::fromROSMsg(msg, *laserCloudRaw);
        laserCloudIn->clear();
        laserCloudIn->reserve(laserCloudRaw->size());
        for (const auto &raw : laserCloudRaw->points) {
            PointType point;
            point.x = raw.x;
            point.y = raw.y;
            point.z = raw.z;
            point.intensity = 0.0f;
            laserCloudIn->push_back(point);
        }
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cloud_time_).count() < 500) {
            return;
        }
        last_cloud_time_ = now;

        load_input_cloud_xyz(*laserCloudMsg);

        // Look up transforms
        Eigen::Matrix3f rot;
        Eigen::Vector3f trans;
        try {
            std::string source_frame = source_frame_;
            if (use_msg_frame_id_ && !laserCloudMsg->header.frame_id.empty())
                source_frame = laserCloudMsg->header.frame_id;

            auto tf = tf_buffer_->lookupTransform(map_frame_, source_frame, tf2::TimePointZero);
            auto robot_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);

            robotPoint.x = robot_tf.transform.translation.x;
            robotPoint.y = robot_tf.transform.translation.y;
            robotPoint.z = robot_tf.transform.translation.z;

            const auto &t = tf.transform.translation;
            const auto &q = tf.transform.rotation;
            rot = Eigen::Quaternionf(q.w, q.x, q.y, q.z).toRotationMatrix();
            trans = Eigen::Vector3f(t.x, t.y, t.z);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "Transform failure: %s", ex.what());
            laserCloudIn->clear();
            return;
        }

        // Height-map origin centred on robot
        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        localMapOrigin.x = roundedX - sensorMaxRangeLimit;
        localMapOrigin.y = roundedY - sensorMaxRangeLimit;

        // Reset per-scan height bins.
        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            std::memset(initFlag[i], 0, filterHeightMapArrayLength * sizeof(bool));
            std::fill(sumHeight[i], sumHeight[i] + filterHeightMapArrayLength, 0.0f);
            std::fill(heightCount[i], heightCount[i] + filterHeightMapArrayLength, 0);
        }

        // Fused single-pass: range filter + self-filter + transform + height-map binning
        const float maxR2 = sensorMaxRangeLimit * sensorMaxRangeLimit;
        const float invRes = 1.0f / mapResolution;

        pcl::PointCloud<PointType>::ConstPtr cloud_for_loop = laserCloudIn;
        if (sensor_voxel_leaf_m_ > 1e-6f) {
            pcl::VoxelGrid<PointType> vg;
            vg.setInputCloud(laserCloudIn);
            vg.setLeafSize(sensor_voxel_leaf_m_, sensor_voxel_leaf_m_, sensor_voxel_leaf_m_);
            vg.filter(*laserCloudWork);
            cloud_for_loop = laserCloudWork;
        }

        if (noise_radius_m_ > 1e-6f) {
            pcl::RadiusOutlierRemoval<PointType> radius_filter;
            radius_filter.setInputCloud(cloud_for_loop);
            radius_filter.setRadiusSearch(noise_radius_m_);
            radius_filter.setMinNeighborsInRadius(noise_min_neighbors_);
            radius_filter.filter(*laserCloudDenoised);
            cloud_for_loop = laserCloudDenoised;
        }

        const size_t npts = cloud_for_loop->points.size();
        const size_t stride = static_cast<size_t>(point_stride_);

        for (size_t i = 0; i < npts; i += stride) {
            const auto &p = cloud_for_loop->points[i];
            float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
            if (r2 > maxR2) continue;
            if (p.z > max_lidar_z_m_) continue;

            const Eigen::Vector3f sensor_pt(p.x, p.y, p.z);
            if (use_lidar_body_box_filter_) {
                if (sensor_pt.x() >= lidar_body_box_min_x_m_ && sensor_pt.x() <= lidar_body_box_max_x_m_ &&
                    sensor_pt.y() >= lidar_body_box_min_y_m_ && sensor_pt.y() <= lidar_body_box_max_y_m_ &&
                    sensor_pt.z() >= lidar_body_box_min_z_m_ && sensor_pt.z() <= lidar_body_box_max_z_m_) {
                    continue;
                }
            }

            Eigen::Vector3f pt = rot * sensor_pt + trans;

            int idx = static_cast<int>((pt.x() - localMapOrigin.x) * invRes);
            int idy = static_cast<int>((pt.y() - localMapOrigin.y) * invRes);
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;

            float z = pt.z();
            if (z > maxObstacleHeight) {
                continue;
            }
            if (!initFlag[idx][idy]) {
                minHeight[idx][idy] = z;
                maxHeight[idx][idy] = z;
                sumHeight[idx][idy] = z;
                heightCount[idx][idy] = 1;
                initFlag[idx][idy] = true;
            } else {
                if (z < minHeight[idx][idy]) minHeight[idx][idy] = z;
                if (z > maxHeight[idx][idy]) maxHeight[idx][idy] = z;
                sumHeight[idx][idy] += z;
                heightCount[idx][idy]++;
            }
        }

        // Convert height map to output cloud
        laserCloudOut->clear();
        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                if (!initFlag[i][j]) continue;
                PointType tp;
                tp.x = localMapOrigin.x + i * mapResolution + mapResolution * 0.5f;
                tp.y = localMapOrigin.y + j * mapResolution + mapResolution * 0.5f;
                tp.z = sumHeight[i][j] / std::max(1, heightCount[i][j]);
                tp.intensity = 0;
                laserCloudOut->push_back(tp);
            }
        }

        publishCloud();
        laserCloudRaw->clear();
        laserCloudIn->clear();
    }

    void extractRawCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        // RCLCPP_INFO(this->get_logger(), "Converting ROS message to PCL...");
        load_input_cloud_xyz(*laserCloudMsg);
        
        // Initialize matrices
        obstacleMatrix.setTo(-1);  // -1 = invalid
        rangeMatrix.setTo(-1);     // -1 = invalid
        
        int valid_points = 0;
        int nan_points = 0;
        int out_of_bounds = 0;
    
        // DEBUG
        int num_printed = 0;
    
        // Create a temporary cloud to hold filtered points
        pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>());
    
        // Extract range info
        // RCLCPP_INFO(this->get_logger(), "Starting range extraction loop...");
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j + i * Horizon_SCAN;
                
                // Check bounds first
                if (index >= laserCloudIn->points.size()) {
                    out_of_bounds++;
                    continue;
                }
                                   
                // Perform the actual filtering
                // find the squared distance from the origin.
                float pointDepth2 = sqrt((laserCloudIn->points[index].x * laserCloudIn->points[index].x) +
                                        (laserCloudIn->points[index].y * laserCloudIn->points[index].y) + 
                                        (laserCloudIn->points[index].z * laserCloudIn->points[index].z));
    
                // Store the range and reset obstacle status. Self-filtering is
                // performed in cloudHandler() with the base_link body box.
                rangeMatrix.at<float>(i, j) = pointDepth2;
                obstacleMatrix.at<int>(i, j) = 0;
                
                // Add filtered point to temporary cloud
                filteredCloud->push_back(laserCloudIn->points[index]);
                valid_points++;
            }
        }
    
        // // DEBUG: Publish filtered cloud
        // sensor_msgs::msg::PointCloud2 debug_cloud;
        // pcl::toROSMsg(*filteredCloud, debug_cloud);
        // debug_cloud.header.stamp = this->get_clock()->now();
        // debug_cloud.header.frame_id = "Lidar_v2_1";
        // pubCloud->publish(debug_cloud);
        // RCLCPP_INFO(this->get_logger(), "Published filtered cloud: %zu points (filtered out %d too close)", 
        //             filteredCloud->size(), num_too_close);
    }

    bool transformCloud(const std::string & msg_frame){
        try{
            std::string source_frame = source_frame_;
            if (use_msg_frame_id_ && !msg_frame.empty()) {
                source_frame = msg_frame;
            }

            auto transform = tf_buffer_->lookupTransform(map_frame_, source_frame, tf2::TimePointZero);

            auto robot_transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            robotPoint.x = robot_transform.transform.translation.x;
            robotPoint.y = robot_transform.transform.translation.y;
            robotPoint.z = robot_transform.transform.translation.z;

            const auto &t = transform.transform.translation;
            const auto &q = transform.transform.rotation;
            Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3f rot = quat.toRotationMatrix();
            Eigen::Vector3f trans(t.x, t.y, t.z);

            for (auto &p : laserCloudIn->points) {
                Eigen::Vector3f pt(p.x, p.y, p.z);
                pt = rot * pt + trans;
                p.x = pt.x();
                p.y = pt.y();
                p.z = pt.z();
            }
        }
        catch (tf2::TransformException& ex){
            RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
            return false;
        }

        return true;
    }

    void cloud2Matrix(){
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void extractFilteredCloud(){
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // invalid points and points too far are skipped
                if (rangeMatrix.at<float>(i, j) > sensorMaxRangeLimit ||
                    rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // update point intensity (occupancy) into
                PointType p = laserCloudMatrix[i][j];
                laserCloudOut->push_back(p);
            }
        }
        // RCLCPP_INFO(this->get_logger(), "laserCloudOut contains %zu points", laserCloudOut->size());
    }

    void downsampleCloud(){

        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        // height map origin
        localMapOrigin.x = roundedX - sensorMaxRangeLimit;
        localMapOrigin.y = roundedY - sensorMaxRangeLimit;
        
        // convert from point cloud to height map
        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i){

            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution;
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;
            // points out of boundry
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;
            if (laserCloudOut->points[i].z > maxObstacleHeight)
                continue;
            // // obstacle point (decided by curb or slope filter)
            // if (laserCloudOut->points[i].intensity == 100)
            //     obstFlag[idx][idy] = true;
            // save min and max height of a grid
            // maybe can add an average filter to smooth out the point readings from ouster as they are noisy
            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                sumHeight[idx][idy] = laserCloudOut->points[i].z;
                heightCount[idx][idy] = 1;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
                sumHeight[idx][idy] += laserCloudOut->points[i].z;
                heightCount[idx][idy]++;
            }
        }
        // intermediate cloud
        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
        // convert from height map to point cloud
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // no point at this grid
                if (initFlag[i][j] == false)
                    continue;
                // convert grid to point
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = sumHeight[i][j] / std::max(1, heightCount[i][j]);

                thisPoint.intensity = 0; // free
                laserCloudTemp->push_back(thisPoint);
            }
        }

        *laserCloudOut = *laserCloudTemp;
    }

    void predictCloudBGK(){

        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // skip observed point
                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z; // this value is not used except for computing distance with robotPoint
                // skip grids too far
                if (pointDistance(testPoint, robotPoint) > sensorMaxRangeLimit)
                    continue;
                // Training data
                vector<float> xTrainVec; // training data x and y coordinates
                vector<float> yTrainVecElev; // training data elevation
                vector<float> yTrainVecOccu; // training data occupancy
                // Fill trainig data (vector)
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m){
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n){
                        // skip grids too far
                        if (std::sqrt(float(m*m + n*n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;
                        // index out of boundry
                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;
                        // save only observed grid in this scan
                        if (initFlag[idx][idy] == true){
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(sumHeight[idx][idy] / std::max(1, heightCount[idx][idy]));
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }
                // no training data available, continue
                if (xTrainVec.size() == 0)
                    continue;
                // convert from vector to eigen
                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
                // Test data (current grid)
                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
                // Predict
                Eigen::MatrixXf Ks; // covariance matrix
                covSparse(xTest, xTrain, Ks); // sparse kernel

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                // Update Elevation with Prediction
                if (std::isnan(ybarElev(0,0)) || std::isnan(ybarOccu(0,0)) || std::isnan(kbar(0,0)))
                    continue;

                if (kbar(0,0) == 0)
                    continue;

                float elevation = ybarElev(0,0) / kbar(0,0);
                float occupancy = ybarOccu(0,0) / kbar(0,0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = (occupancy > 0.5) ? 100 : 0;

                laserCloudOut->push_back(p);
            }
        }
    }

    void dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &d) const {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const {
        dist(xStar/(predictionKernalSize+0.1), xTrain/(predictionKernalSize+0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
              (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= predictionKernalSize
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i,j) < 0) Kxz(i,j) = 0;
    }

    void publishCloud(){
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = this->get_clock()->now();
        laserCloudTemp.header.frame_id = map_frame_;
        pubCloud->publish(laserCloudTemp);
    }


    void pointcloud2laserscanInitialization(){

        laserScan.header.frame_id = base_frame_;

        laserScan.angle_min = -M_PI;
        laserScan.angle_max =  M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);
    }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TraversabilityFilter>();

    RCLCPP_INFO(node->get_logger(), "Traversability Filter Started.");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
