/*
 * @file traversability_mapping.cpp
 * @brief ROS 2 node for building local traversability and occupancy maps from filtered terrain point clouds.
 *
 * Input:
 * - Subscribes to `pointcloud_topic`, default: `/cloud_pcd`
 * - Typical live input: `/filtered_pointcloud`
 *
 * Outputs:
 * - Publishes local traversability occupancy grid on `output_local_topic`,
 *   default: `/occupancy_map_local`
 * - Publishes inflated local traversability occupancy grid on `output_local_inflated_topic`,
 *   default: `/occupancy_map_local_inflated`
 * - Publishes elevation/traversability point cloud on `output_elevation_topic`,
 *   default: `/elevation_pointcloud`
 *
 * This node consumes a filtered PointCloud2 terrain cloud and inserts the points into a tiled elevation
 * map. For each observed grid cell, it estimates terrain traversability from local surface geometry:
 * slope is computed by fitting a plane to neighboring elevation cells, and roughness is computed from
 * the height residuals around that plane. The resulting traversability cost is converted into occupancy
 * values suitable for local navigation.
 *
 * The map can operate in robot-centered local mode or fixed-origin/global mode. Observed cell costs are
 * kept in map memory until the cell is reobserved and overwritten; an optional output lookup radius
 * bridges small point/grid alignment shifts. An optional inflation step expands high-cost traversability
 * regions using a distance-decay kernel so navigation can account for robot footprint and safety margin.
 *
 * Typical use:
 * - Convert filtered LiDAR terrain points into a 2D traversability cost map.
 * - Publish local occupancy grids for path planning or Nav2 integration.
 * - Visualize elevation and traversability as a PointCloud2 map.
 */

#include "utility.h"
#include <unordered_map>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class TraversabilityMapping : public rclcpp::Node {

private:

    // Mutex Memory Lock
    std::mutex mtx;
    // Transform Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subFilteredGroundCloud;
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMapLocal;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMapLocalInflated;
    // rclcpp::Publisher<elevation_msgs::msg::OccupancyElevation>::SharedPtr pubOccupancyMapLocalHeight;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubElevationCloud;
    // Point Cloud Pointer
    pcl::PointCloud<PointType>::Ptr laserCloud; // save input filtered laser cloud for mapping
    pcl::PointCloud<PointType>::Ptr laserCloudElevation; // a cloud for publishing elevation map
    // Occupancy Grid Map
    nav_msgs::msg::OccupancyGrid occupancyMap2D; // local occupancy grid map
    nav_msgs::msg::OccupancyGrid occupancyMap2DInflated; // local occupancy grid map
    // elevation_msgs::msg::OccupancyElevation occupancyMap2DHeight; // customized message that includes occupancy map and elevation info

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

    // Dense traversability grid. This replaces the previous 2x2-cell submap lookup so
    // neighborhood searches operate directly in map-cell coordinates.
    float grid_size_m_{globalMapDim};
    float grid_resolution_m_{mapResolution};
    float output_map_size_m_{localMapLength};
    int map_width_cells_{0};
    int map_height_cells_{0};
    int output_map_cells_{0};
    float map_origin_x_{0.0f};
    float map_origin_y_{0.0f};
    vector<mapCell_t> map_cells_;
    vector<PointType> map_points_;

    // Robot pose in the map frame used for local output window placement.
    PointType robotPoint;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    vector<mapCell_t*> observingList1; // save newly observed cells for traversability update

    // Synthetic observations are applied only while publishing. Saving the
    // previous state lets the next scan restore real/unknown cells before any
    // elevation fusion or traversability calculation takes place.
    struct FootprintCellBackup {
        int grid_index;
        int observe_times;
        float elevation;
        float elevation_var;
        float occupancy;
        float occupancy_var;
        bool needs_update;
        float point_z;
        float point_intensity;
    };
    vector<FootprintCellBackup> footprint_cell_backups_;

    // Inflation Parameters
    std::string pointcloud_topic_;
    std::string output_local_topic_;
    std::string output_local_inflated_topic_;
    std::string output_elevation_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string source_frame_;
    bool use_robot_centered_origin_;
    double fixed_origin_x_;
    double fixed_origin_y_;
    bool lidar_dead_zone_enabled_{true};
    float lidar_dead_zone_min_angle_deg_{-70.0f};
    float lidar_dead_zone_max_angle_deg_{-20.0f};
    float lidar_dead_zone_radius_m_{10.0f};
    bool footprint_output_clearing_enabled_{true};
    std::vector<std::array<float, 2>> footprint_clear_polygon_;
    int footprint_observation_stride_cells_{2};
    bool has_robot_pose_{false};
    float robot_yaw_{0.0f};
    bool has_lidar_from_map_transform_{false};
    Eigen::Matrix3f lidar_from_map_rot_{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f lidar_from_map_trans_{Eigen::Vector3f::Zero()};
    uint64_t current_scan_id_;
    std::unordered_map<mapCell_t*, uint64_t> last_observed_scan_;
    rclcpp::Time current_scan_time_;
    std::unordered_map<mapCell_t*, rclcpp::Time> last_observed_time_;
    int output_lookup_radius_cells_{2};
    int radius_inflation = this->declare_parameter<int>("inflation_radius", 5); // in cells
    float alpha_inflation = static_cast<float>(this->declare_parameter<double>("inflation_factor", 1.0)); // inflation factor, can be tuned based on how much we want to inflate
    float sigmoid_k = static_cast<float>(this->declare_parameter<double>("sigmoid_k", 3.0)); // weight for slope in occupancy calculation
    float sigmoid_x0 = static_cast<float>(this->declare_parameter<double>("sigmoid_x0", 0.7)); // threshold for slope in occupancy calculation
    float neighbor_search_radius_m_ = static_cast<float>(this->declare_parameter<double>("neighbor_search_radius_m", 0.6));
    float slope_angle_limit_deg_ = static_cast<float>(this->declare_parameter<double>("slope_angle_limit_deg", filterAngleLimit));
    float roughness_norm_m_ = static_cast<float>(this->declare_parameter<double>("roughness_norm_m", filterMaxRoughness));
    int min_neighbor_points_ = this->declare_parameter<int>("min_neighbor_points", 3);
    int min_neighbor_quadrants_ = this->declare_parameter<int>("min_neighbor_quadrants", 3);
    float cell_clear_timeout_s_ = static_cast<float>(this->declare_parameter<double>("cell_clear_timeout_s", 0.7)); // cells with no new points for this long revert to unknown; 0 disables
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    
    float applySigmoidToOccupancy(float p) const {
        p = std::clamp(p, 0.0f, 1.0f);

        // Logistic remap around threshold x0
        const float s = 1.0f / (1.0f + std::exp(-sigmoid_k * (p - sigmoid_x0)));

        // Normalize so p=0 maps to 0 and p=1 maps to 1 (important for stable scaling)
        const float s0 = 1.0f / (1.0f + std::exp(-sigmoid_k * (0.0f - sigmoid_x0)));
        const float s1 = 1.0f / (1.0f + std::exp(-sigmoid_k * (1.0f - sigmoid_x0)));
        const float denom = std::max(1e-6f, s1 - s0);

        return std::clamp((s - s0) / denom, 0.0f, 1.0f);
    }


public:
    TraversabilityMapping() : Node("traversability_mapping"),
        current_scan_id_(0) {

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // CHANGE ME:
        // 1. If you want to use the gazebo pointcloud / or local pointcloud  use /filtered_pointcloud
        // 2. If you want to use the marsyard pcd file, use /cloud_pcd (when computing the new .yaml and .pgm file in path_planning/saved_maps)

        pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/cloud_pcd");
        output_local_topic_ = this->declare_parameter<std::string>("output_local_topic", "/occupancy_map_local");
        output_local_inflated_topic_ = this->declare_parameter<std::string>("output_local_inflated_topic", "/occupancy_map_local_inflated");
        output_elevation_topic_ = this->declare_parameter<std::string>("output_elevation_topic", "/elevation_pointcloud");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        source_frame_ = this->declare_parameter<std::string>("source_frame", "Lidar_v2_1");
        use_robot_centered_origin_ = this->declare_parameter<bool>("use_robot_centered_origin", true);
        fixed_origin_x_ = this->declare_parameter<double>("fixed_origin_x", -localMapLength / 2.0);
        fixed_origin_y_ = this->declare_parameter<double>("fixed_origin_y", -localMapLength / 2.0);
        grid_size_m_ = static_cast<float>(this->declare_parameter<double>("grid_size_m", globalMapDim));
        grid_resolution_m_ = static_cast<float>(this->declare_parameter<double>("grid_resolution_m", mapResolution));
        lidar_dead_zone_enabled_ = this->declare_parameter<bool>("lidar_dead_zone_enabled", true);
        lidar_dead_zone_min_angle_deg_ = static_cast<float>(this->declare_parameter<double>("lidar_dead_zone_min_angle_deg", -70.0));
        lidar_dead_zone_max_angle_deg_ = static_cast<float>(this->declare_parameter<double>("lidar_dead_zone_max_angle_deg", -20.0));
        lidar_dead_zone_radius_m_ = static_cast<float>(this->declare_parameter<double>("lidar_dead_zone_radius_m", 10.0));
        footprint_output_clearing_enabled_ =
            this->declare_parameter<bool>("footprint_output_clearing_enabled", true);
        const std::vector<double> default_footprint_xy{
            0.74, 0.318, 0.318, 0.74, -0.318, 0.74, -0.74, 0.318,
            -0.74, -0.318, -0.318, -0.74, 0.318, -0.74, 0.74, -0.318};
        const auto footprint_xy = this->declare_parameter<std::vector<double>>(
            "footprint_clear_polygon_xy", default_footprint_xy);
        footprint_observation_stride_cells_ =
            this->declare_parameter<int>("footprint_observation_stride_cells", 2);
        if (footprint_xy.size() >= 6 && footprint_xy.size() % 2 == 0) {
            footprint_clear_polygon_.reserve(footprint_xy.size() / 2);
            for (size_t i = 0; i < footprint_xy.size(); i += 2) {
                footprint_clear_polygon_.push_back({
                    static_cast<float>(footprint_xy[i]),
                    static_cast<float>(footprint_xy[i + 1])});
            }
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "footprint_clear_polygon_xy must contain at least three flattened XY pairs; "
                "producer footprint clearing is disabled");
            footprint_output_clearing_enabled_ = false;
        }
        if (footprint_observation_stride_cells_ < 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "footprint_observation_stride_cells must be >= 1, clamping to 1");
            footprint_observation_stride_cells_ = 1;
        }
        output_lookup_radius_cells_ = this->declare_parameter<int>("output_lookup_radius_cells", 2);
        if (grid_size_m_ <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), "grid_size_m must be > 0, using %.3f", static_cast<float>(globalMapDim));
            grid_size_m_ = globalMapDim;
        }
        if (grid_resolution_m_ <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), "grid_resolution_m must be > 0, using %.3f", mapResolution);
            grid_resolution_m_ = mapResolution;
        }
        map_width_cells_ = std::max(1, static_cast<int>(std::round(grid_size_m_ / grid_resolution_m_)));
        map_height_cells_ = map_width_cells_;
        output_map_cells_ = std::max(1, static_cast<int>(std::round(output_map_size_m_ / grid_resolution_m_)));
        map_origin_x_ = -0.5f * map_width_cells_ * grid_resolution_m_;
        map_origin_y_ = -0.5f * map_height_cells_ * grid_resolution_m_;
        if (output_lookup_radius_cells_ < 0) {
            RCLCPP_WARN(this->get_logger(), "output_lookup_radius_cells must be >= 0, clamping to 0");
            output_lookup_radius_cells_ = 0;
        }
        if (lidar_dead_zone_radius_m_ < 0.0f) {
            RCLCPP_WARN(this->get_logger(), "lidar_dead_zone_radius_m must be >= 0, clamping to 0");
            lidar_dead_zone_radius_m_ = 0.0f;
        }
        if (neighbor_search_radius_m_ < grid_resolution_m_) {
            RCLCPP_WARN(this->get_logger(), "neighbor_search_radius_m must be >= grid_resolution_m, clamping to %.3f", grid_resolution_m_);
            neighbor_search_radius_m_ = grid_resolution_m_;
        }
        if (slope_angle_limit_deg_ <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), "slope_angle_limit_deg must be > 0, using %.3f", filterAngleLimit);
            slope_angle_limit_deg_ = filterAngleLimit;
        }
        if (roughness_norm_m_ <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), "roughness_norm_m must be > 0, using %.3f", filterMaxRoughness);
            roughness_norm_m_ = filterMaxRoughness;
        }
        if (min_neighbor_points_ < 3) {
            RCLCPP_WARN(this->get_logger(), "min_neighbor_points must be >= 3 for plane fitting, clamping to 3");
            min_neighbor_points_ = 3;
        }
        if (min_neighbor_quadrants_ < 1) {
            RCLCPP_WARN(this->get_logger(), "min_neighbor_quadrants must be >= 1, clamping to 1");
            min_neighbor_quadrants_ = 1;
        } else if (min_neighbor_quadrants_ > 4) {
            RCLCPP_WARN(this->get_logger(), "min_neighbor_quadrants must be <= 4, clamping to 4");
            min_neighbor_quadrants_ = 4;
        }

        subFilteredGroundCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&TraversabilityMapping::cloudHandler, this, std::placeholders::_1));

        pubOccupancyMapLocal = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_local_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        pubOccupancyMapLocalInflated = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_local_inflated_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        // pubOccupancyMapLocalHeight = this->create_publisher<elevation_msgs::msg::OccupancyElevation>("/occupancy_map_local_height", 10);
        pubElevationCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_elevation_topic_, 5);

        allocateMemory();

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TraversabilityMapping::onSetParameters, this, std::placeholders::_1));
    }

    ~TraversabilityMapping(){}

    // Live parameter tuning (ros2 param set / rqt_reconfigure). Validates the whole batch first so a
    // rejected set never half-applies, then updates the members read by the scan/publish path under mtx.
    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            const std::string &name = p.get_name();
            std::string reason;
            if (name == "pointcloud_topic" || name == "output_local_topic" ||
                name == "output_local_inflated_topic" || name == "output_elevation_topic" ||
                name == "map_frame" || name == "base_frame" || name == "source_frame" ||
                name == "grid_size_m" || name == "grid_resolution_m" ||
                name == "footprint_clear_polygon_xy") {
                reason = name + " is structural (topics/frames/grid allocation); restart the node to change it";
            } else if (name == "inflation_radius" && p.as_int() < 0) {
                reason = "inflation_radius must be >= 0";
            } else if (name == "neighbor_search_radius_m" && p.as_double() < grid_resolution_m_) {
                reason = "neighbor_search_radius_m must be >= grid_resolution_m";
            } else if (name == "slope_angle_limit_deg" && p.as_double() <= 0.0) {
                reason = "slope_angle_limit_deg must be > 0";
            } else if (name == "roughness_norm_m" && p.as_double() <= 0.0) {
                reason = "roughness_norm_m must be > 0";
            } else if (name == "min_neighbor_points" && p.as_int() < 3) {
                reason = "min_neighbor_points must be >= 3 for plane fitting";
            } else if (name == "min_neighbor_quadrants" && (p.as_int() < 1 || p.as_int() > 4)) {
                reason = "min_neighbor_quadrants must be in [1, 4]";
            } else if (name == "output_lookup_radius_cells" && p.as_int() < 0) {
                reason = "output_lookup_radius_cells must be >= 0";
            } else if (name == "lidar_dead_zone_radius_m" && p.as_double() < 0.0) {
                reason = "lidar_dead_zone_radius_m must be >= 0";
            } else if (name == "cell_clear_timeout_s" && p.as_double() < 0.0) {
                reason = "cell_clear_timeout_s must be >= 0 (0 disables)";
            } else if (name == "footprint_observation_stride_cells" && p.as_int() < 1) {
                reason = "footprint_observation_stride_cells must be >= 1";
            }
            if (!reason.empty()) {
                result.successful = false;
                result.reason = reason;
                return result;
            }
        }

        std::lock_guard<std::mutex> lock(mtx);
        for (const auto &p : params) {
            const std::string &name = p.get_name();
            if      (name == "inflation_radius")           radius_inflation = static_cast<int>(p.as_int());
            else if (name == "inflation_factor")           alpha_inflation = static_cast<float>(p.as_double());
            else if (name == "sigmoid_k")                  sigmoid_k = static_cast<float>(p.as_double());
            else if (name == "sigmoid_x0")                 sigmoid_x0 = static_cast<float>(p.as_double());
            else if (name == "neighbor_search_radius_m")   neighbor_search_radius_m_ = static_cast<float>(p.as_double());
            else if (name == "slope_angle_limit_deg")      slope_angle_limit_deg_ = static_cast<float>(p.as_double());
            else if (name == "roughness_norm_m")           roughness_norm_m_ = static_cast<float>(p.as_double());
            else if (name == "min_neighbor_points")        min_neighbor_points_ = static_cast<int>(p.as_int());
            else if (name == "min_neighbor_quadrants")     min_neighbor_quadrants_ = static_cast<int>(p.as_int());
            else if (name == "output_lookup_radius_cells") output_lookup_radius_cells_ = static_cast<int>(p.as_int());
            else if (name == "lidar_dead_zone_enabled")    lidar_dead_zone_enabled_ = p.as_bool();
            else if (name == "lidar_dead_zone_min_angle_deg") lidar_dead_zone_min_angle_deg_ = static_cast<float>(p.as_double());
            else if (name == "lidar_dead_zone_max_angle_deg") lidar_dead_zone_max_angle_deg_ = static_cast<float>(p.as_double());
            else if (name == "lidar_dead_zone_radius_m")   lidar_dead_zone_radius_m_ = static_cast<float>(p.as_double());
            else if (name == "footprint_output_clearing_enabled") footprint_output_clearing_enabled_ = p.as_bool();
            else if (name == "footprint_observation_stride_cells") footprint_observation_stride_cells_ = static_cast<int>(p.as_int());
            else if (name == "cell_clear_timeout_s")       cell_clear_timeout_s_ = static_cast<float>(p.as_double());
            else if (name == "fixed_origin_x")             fixed_origin_x_ = p.as_double();
            else if (name == "fixed_origin_y")             fixed_origin_y_ = p.as_double();
            else if (name == "use_robot_centered_origin")  use_robot_centered_origin_ = p.as_bool();
            else continue;
            RCLCPP_INFO(this->get_logger(), "Runtime update: %s = %s", name.c_str(), p.value_to_string().c_str());
        }
        return result;
    }

    bool isCellKnownForOutput(const mapCell_t *cell) const {
        return cell != nullptr && cell->observeTimes > 0;
    }

    mapCell_t* getOutputCellFromPoint(const PointType &point) {
        PointType query = point;
        mapCell_t *cell = getCellFromPoint(&query);
        if (isCellKnownForOutput(cell) || output_lookup_radius_cells_ <= 0) {
            return cell;
        }

        mapCell_t *best_cell = nullptr;
        int best_dist_sq = output_lookup_radius_cells_ * output_lookup_radius_cells_ + 1;

        for (int dx = -output_lookup_radius_cells_; dx <= output_lookup_radius_cells_; ++dx) {
            for (int dy = -output_lookup_radius_cells_; dy <= output_lookup_radius_cells_; ++dy) {
                if (dx == 0 && dy == 0) continue;

                const int dist_sq = dx * dx + dy * dy;
                if (dist_sq > output_lookup_radius_cells_ * output_lookup_radius_cells_) continue;

                PointType neighbor = point;
                neighbor.x += dx * grid_resolution_m_;
                neighbor.y += dy * grid_resolution_m_;

                mapCell_t *candidate = getCellFromPoint(&neighbor);
                if (!isCellKnownForOutput(candidate)) continue;

                if (best_cell == nullptr ||
                    dist_sq < best_dist_sq ||
                    (dist_sq == best_dist_sq && candidate->occupancy > best_cell->occupancy)) {
                    best_cell = candidate;
                    best_dist_sq = dist_sq;
                }
            }
        }

        return best_cell;
    }

    void allocateMemory(){
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());

        const int total_cells = map_width_cells_ * map_height_cells_;
        map_cells_.resize(total_cells);
        map_points_.resize(total_cells);

        for (int y = 0; y < map_height_cells_; ++y) {
            for (int x = 0; x < map_width_cells_; ++x) {
                const int index = x + y * map_width_cells_;
                map_points_[index].x = map_origin_x_ + (x + 0.5f) * grid_resolution_m_;
                map_points_[index].y = map_origin_y_ + (y + 0.5f) * grid_resolution_m_;
                map_points_[index].z = std::numeric_limits<float>::quiet_NaN();
                map_points_[index].intensity = 0.0f;

                map_cells_[index].xyz = &map_points_[index];
                map_cells_[index].grid.mapID = 0;
                map_cells_[index].grid.cubeX = 0;
                map_cells_[index].grid.cubeY = 0;
                map_cells_[index].grid.gridX = x;
                map_cells_[index].grid.gridY = y;
                map_cells_[index].grid.gridIndex = index;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        auto start = std::chrono::high_resolution_clock::now();

        // Limit the speed of the callback
        if (std::chrono::duration_cast<std::chrono::milliseconds>(start - last_time).count() < cloudHandlerRate)
        {
            // RCLCPP_WARN(this->get_logger(), "TOO EARLY");
            return;
        }
        last_time = start;
        
        // Lock the the processes to prevent new data from interrupting old data
        std::lock_guard<std::mutex> lock(mtx);

        // Remove the synthetic footprint observations from the preceding
        // publication before ingesting real measurements from this scan.
        restoreFootprintObservationCells();

        ++current_scan_id_;
        current_scan_time_ = this->now();
        
        auto t1 = std::chrono::high_resolution_clock::now();
        if (use_robot_centered_origin_) {
            if (getRobotPosition() == false) {
                RCLCPP_WARN(this->get_logger(), "Location not found");
                return;
            }
        } else {
            (void)getRobotPosition();
        }
        
        auto t2 = std::chrono::high_resolution_clock::now();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);
        
        auto t3 = std::chrono::high_resolution_clock::now();
        updateElevationMap();

        auto t4 = std::chrono::high_resolution_clock::now();
        // updateOccupancyGrid();

        auto t5 = std::chrono::high_resolution_clock::now();
        // publishMap();
        auto end = std::chrono::high_resolution_clock::now();
        
        // Print timing breakdown
        auto lock_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - start).count();
        auto tf_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto convert_time = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto update_time = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto inflation_time = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
        auto publish_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - t5).count();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        // RCLCPP_WARN(this->get_logger(), 
        // "Processing: total=%ldms (lock=%ld, tf=%ld, convert=%ld, update=%ld, publish=%ld)",
        //     total_time, lock_time, tf_time, convert_time, update_time, publish_time);
    }

    void updateElevationMap(){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            // laserCloud->points[i].z -= 0.2; // TODO: CHANGEME, commented for visualization
            updateElevationMap(&laserCloud->points[i]);
        }

        // for points in the map that don't have any 
    }

    void updateElevationMap(PointType *point){
        mapCell_t *cell = getCellFromPoint(point);
        if (cell == nullptr) {
            return;
        }

        updateOccupancyCell(cell, point);
    }

    void updateOccupancyCell(mapCell_t *cell, PointType *point){
        // if (point->intensity == 100) // obstacle
        //     RCLCPP_ERROR(this->get_logger(), "OBSTACLE");
        //     // updateOccupancyBel(cell, true);      #### THIS IS BECAUSE I DON't USE INTENSITY
        // else if (point->intensity == 0) // free
        // updateOccupancyBel(cell, false);
        
        cell->observeTimes++;
        
        updateElevationBGK(cell, point);    // fuses elevation from all points within a cell

        if (last_observed_scan_[cell] != current_scan_id_) {
            last_observed_scan_[cell] = current_scan_id_;
            last_observed_time_[cell] = current_scan_time_;
            observingList1.push_back(cell);
        }
    }

    // Not used
    // void updateOccupancyBel(mapCell_t *cell, bool occupied){
    //     if (occupied == true)
    //         cell->log_odds += log(p_occupied_when_laser / (1 - p_occupied_when_laser));
    //     else
    //         cell->log_odds += log(p_occupied_when_no_laser / (1 - p_occupied_when_no_laser));

    //     if (cell->log_odds < -large_log_odds)
    //         cell->log_odds = -large_log_odds;
    //     else if (cell->log_odds > large_log_odds)
    //         cell->log_odds = large_log_odds;

    //     if (cell->log_odds >= 0)
    //         cell->updateOccupancy(float(1.0 - 1.0/(1.0 + exp(cell->log_odds))));
    //     else
    //         cell->updateOccupancy(float(1.0/(1.0 + exp(-cell->log_odds))));
    // }

    void updateElevationBGK(mapCell_t *cell, PointType *point){
        // Use raw measurement directly: no temporal smoothing / decay memory.
        cell->updateElevation(point->z, 1e-3f);
    }

    void updateOccupancyGrid(){
        // This function can be used to implement additional occupancy grid updates if needed
        // Here we perform inflation here based on the traversability of cells and a kernel
        // We do it here, since inflation layer in plugin works with obstacle not traversability, and we want to have the flexibility to inflate based on traversability if needed.

        // Publish standard 2D occupancy grid
        occupancyMap2DInflated.header.frame_id = map_frame_;
        occupancyMap2DInflated.header.stamp = this->get_clock()->now();
        
        // Set map parameters
        occupancyMap2DInflated.info.resolution = grid_resolution_m_;
        occupancyMap2DInflated.info.width = output_map_cells_;
        occupancyMap2DInflated.info.height = output_map_cells_;
        
        // Set origin
        if (use_robot_centered_origin_) {
            occupancyMap2DInflated.info.origin.position.x = robotPoint.x - output_map_size_m_/2.0;
            occupancyMap2DInflated.info.origin.position.y = robotPoint.y - output_map_size_m_/2.0;
        } else {
            occupancyMap2DInflated.info.origin.position.x = fixed_origin_x_;
            occupancyMap2DInflated.info.origin.position.y = fixed_origin_y_;
        }
        occupancyMap2DInflated.info.origin.position.z = 0.0;
        occupancyMap2DInflated.info.origin.orientation.x = 0.0;
        occupancyMap2DInflated.info.origin.orientation.y = 0.0;
        occupancyMap2DInflated.info.origin.orientation.z = 0.0;
        occupancyMap2DInflated.info.origin.orientation.w = 1.0;
        // Fill data
        occupancyMap2DInflated.data.assign(output_map_cells_ * output_map_cells_, -1);
        std::vector<int8_t> source_costs(output_map_cells_ * output_map_cells_, -1);
        std::vector<uint8_t> footprint_mask(output_map_cells_ * output_map_cells_, 0);


        const float origin_x = occupancyMap2DInflated.info.origin.position.x;
        const float origin_y = occupancyMap2DInflated.info.origin.position.y;
        const float half_res = grid_resolution_m_ / 2.0f;
        const float inv_rad1 = 1.0f / (radius_inflation + 1);

        // Rasterize the footprint only over its small grid-aligned bounding
        // box. The former implementation ran the polygon test for every cell
        // in the full output map, which was especially expensive at 0.05 m.
        markFootprintCellsInGrid(occupancyMap2DInflated, footprint_mask);

        // First pass: calculate each observed cell's uninflated cost. Keeping
        // this separate from inflation makes the result independent of scan
        // order; a later source cell can no longer erase an earlier kernel.
        for (int i = 0; i < output_map_cells_; ++i) {
            for (int j = 0; j < output_map_cells_; ++j) {
                PointType point;
                point.x = origin_x + i * grid_resolution_m_ + half_res;
                point.y = origin_y + j * grid_resolution_m_ + half_res;
                point.z = 0.0f;

                int index = i + j * output_map_cells_;
                if (footprint_mask[index]) {
                    source_costs[index] = 0;
                    occupancyMap2DInflated.data[index] = 0;
                    continue;
                }
                if (isInLidarDeadZone(point)) {
                    occupancyMap2DInflated.data[index] = 100;
                    continue;
                }

                mapCell_t *cell = getOutputCellFromPoint(point);
                if (!isCellKnownForOutput(cell)) continue;

                const int source_cost = std::clamp(
                    static_cast<int>(applySigmoidToOccupancy(cell->occupancy) * 100.0f),
                    0, 100);
                source_costs[index] = static_cast<int8_t>(source_cost);
                occupancyMap2DInflated.data[index] = static_cast<int8_t>(source_cost);
            }
        }

        // Second pass: spread a decayed copy of each positive source cost.
        // At distance zero the kernel is 1, so the source keeps its original
        // value. The old `cost * (1 + kernel)` doubled every source and, with
        // a small decay factor, nearly doubled the whole local patch.
        for (int i = 0; i < output_map_cells_; ++i) {
            for (int j = 0; j < output_map_cells_; ++j) {
                const int source_cost = source_costs[i + j * output_map_cells_];
                if (source_cost <= 0) continue;

                for (int m = -radius_inflation; m <= radius_inflation; ++m) {
                    for (int n = -radius_inflation; n <= radius_inflation; ++n) {
                        int x = i + m;
                        int y = j + n;
                        if (x < 0 || x >= output_map_cells_ || y < 0 || y >= output_map_cells_)
                            continue;
                        int kernel_idx = x + y * output_map_cells_;
                        if (footprint_mask[kernel_idx])
                            continue;
                        const float cost_kernel = std::exp(
                            -alpha_inflation * std::sqrt(float(m * m + n * n)) * inv_rad1);
                        const int cost_inflated = std::clamp(
                            static_cast<int>(source_cost * cost_kernel), 0, 100);
                        if (occupancyMap2DInflated.data[kernel_idx] < cost_inflated) {
                            occupancyMap2DInflated.data[kernel_idx] =
                                static_cast<int8_t>(cost_inflated);
                        }
                    }
                }
            }
        }

    }

    void TraversabilityThread(){
        rclcpp::Rate rate(10); // Hz

        while (rclcpp::ok()){
            traversabilityMapCalculation();
            rate.sleep();
        }
    }

    // Reset a cell to its never-observed state so it drops out of the output grids and
    // its stale elevation no longer feeds neighbor plane fits. Call with mtx held.
    void resetCell(mapCell_t *cell){
        cell->log_odds = 0.5f;
        cell->observeTimes = 0;
        cell->elevation = -FLT_MAX;
        cell->elevationVar = 1e3f;
        cell->occupancy = 0.0f;
        cell->occupancyVar = 1e3f;
        cell->xyz->z = std::numeric_limits<float>::quiet_NaN();
        cell->xyz->intensity = 0.0f;
    }

    // Clear cells that received no points for cell_clear_timeout_s_. Returns the number of
    // cells cleared. Call with mtx held.
    size_t clearStaleCells(){
        if (cell_clear_timeout_s_ <= 0.0f || last_observed_time_.empty())
            return 0;

        const rclcpp::Time now = this->now();
        size_t cleared = 0;
        for (auto it = last_observed_time_.begin(); it != last_observed_time_.end();) {
            if ((now - it->second).seconds() > cell_clear_timeout_s_) {
                resetCell(it->first);
                last_observed_scan_.erase(it->first);
                it = last_observed_time_.erase(it);
                ++cleared;
            } else {
                ++it;
            }
        }
        return cleared;
    }

    void traversabilityMapCalculation(){
        // Copy data with lock

        vector<mapCell_t*> cellsToProcess;
        {
            std::lock_guard<std::mutex> lock(mtx);

            // Synthetic footprint cells must never participate in stale-cell
            // handling or terrain plane fitting.
            restoreFootprintObservationCells();

            const size_t clearedCells = clearStaleCells();

            if (observingList1.empty() && clearedCells == 0)
            {
                return;

            }
            cellsToProcess.swap(observingList1);
        }
        
        // Process without lock
        for (auto cell : cellsToProcess) {
            calculateTraversability(cell);
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            const size_t footprint_cells = injectFootprintObservationCells();
            updateOccupancyGrid();
            publishMap();
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Injected %zu synthetic zero-cost observations inside the robot footprint",
                footprint_cells);
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Calculating traversability");
    }

    void calculateTraversability(mapCell_t *cell)
    {
        // BGK-based traversability analysis
        vector<PointType> neighborPoints;
        getNeighborCells(cell, neighborPoints);

        if (neighborPoints.size() < static_cast<size_t>(min_neighbor_points_))
            return;

        if (!hasEnoughNeighborCoverage(cell, neighborPoints))
            return;

        // ===== NEW: Convert to Eigen format =====
        // Build vector of XYZ coordinates
        std::vector<float> xyzVector;
        xyzVector.reserve(neighborPoints.size() * 3);
        
        for (const auto& p : neighborPoints) {
            xyzVector.push_back(p.x);
            xyzVector.push_back(p.y);
            xyzVector.push_back(p.z);
        }

        // Map to Eigen matrix (N × 3)
        Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
            xyzVector.data(), xyzVector.size() / 3, 3);
        
        // **************** Calculate Slope ****************
        // The slope s of a cell is calculated by fitting a plane in a circular region around the cell with a diameter corresponding
        // to the maximum diameter of the robot. The angle between the plane normal and the z-axis of the global coordinate
        // frame gives the slope inclination s. 
        Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean();
        Eigen::MatrixXf cov = (centered.adjoint() * centered);
        cv::eigen2cv(cov, matCov); // copy data from eigen to cv::Mat
        cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix

        float slopeAngle = (std::acos(std::abs(matVec.at<float>(2, 2))) / M_PI) * 180;
        float occupancy;
        float slopeCost;
        slopeCost = slopeAngle / slope_angle_limit_deg_;
        slopeCost = std::min(slopeCost, 1.0f);
        // if (std::isnan(slopeAngle))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Slope: %f", slopeAngle);
        //     return;
        // }   
        

        // **************** Calculate Terrain Roughness ****************
        // The terrain roughness r is calculated as the standard deviation of the terrain height
        // values from the computed plane in the circular region around the cell.
        Eigen::Vector3f normal(
            matVec.at<float>(2, 0),
            matVec.at<float>(2, 1),
            matVec.at<float>(2, 2)
        );

        normal.normalize();
        Eigen::Vector3f centroid = matPoints.colwise().mean();
        float roughness = 0.0f;
        float mean = 0.0f;
        float sq_sum = 0.0f;

        const int N = matPoints.rows();

        for (int i = 0; i < N; ++i) {
            Eigen::Vector3f p = matPoints.row(i);
            float d = std::abs(normal.dot(p - centroid)); // orthogonal distance
            mean += d;
            sq_sum += d * d;
        }

        mean /= N;
        roughness = std::sqrt(sq_sum / N - mean * mean);
        roughness = roughness / roughness_norm_m_;
        roughness = std::min(roughness, 1.0f);

        // Calculate cell occupancy
        occupancy = slopeCoeff * slopeCost + roughnessCoeff * roughness;

    //     //    ===== PRINT CELL POSITION AND SLOPE =====
    //    if (slopeAngle > 20)
    //    {
    //         RCLCPP_INFO(this->get_logger(), 
    //             "Cell [%.2f, %.2f, %.2f] | Slope: %.2f° | Occupancy: %.3f | Roughness: %.3f | Neighbors: %zu", 
    //             cell->xyz->x, 
    //             cell->xyz->y, 
    //             cell->elevation,
    //             slopeAngle,
    //             occupancy,
    //             roughness,
    //             neighborPoints.size());
    //    }
        
        cell->updateOccupancy(occupancy);  
    }

    void getNeighborCells(mapCell_t *cell, vector<PointType> &neighborPoints){
        const int centerX = cell->grid.gridX;
        const int centerY = cell->grid.gridY;
        const int maxRadiusCells = std::max(1, static_cast<int>(std::ceil(neighbor_search_radius_m_ / grid_resolution_m_)));

        for (int radiusCells = 1; radiusCells <= maxRadiusCells; ++radiusCells) {
            neighborPoints.clear();
            const float radius_m = std::min(neighbor_search_radius_m_, radiusCells * grid_resolution_m_);
            const float radius_sq = radius_m * radius_m;

            for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
                for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
                    const float dist_sq = static_cast<float>(dx * dx + dy * dy) * grid_resolution_m_ * grid_resolution_m_;
                    if (dist_sq > radius_sq) {
                        continue;
                    }

                    mapCell_t *neighborCell = getCellAt(centerX + dx, centerY + dy);
                    if (neighborCell != nullptr && neighborCell->observeTimes > 0) {
                        PointType p;
                        p.x = neighborCell->xyz->x;
                        p.y = neighborCell->xyz->y;
                        p.z = neighborCell->elevation;
                        neighborPoints.push_back(p);
                    }
                }
            }

            if (neighborPoints.size() >= static_cast<size_t>(min_neighbor_points_)) {
                return;
            }
        }
    }

    bool hasEnoughNeighborCoverage(const mapCell_t *cell, const vector<PointType> &neighborPoints) const {
        bool quadrants[4] = {false, false, false, false};

        for (const auto &p : neighborPoints) {
            const float dx = p.x - cell->xyz->x;
            const float dy = p.y - cell->xyz->y;
            if (std::abs(dx) < 1e-6f && std::abs(dy) < 1e-6f) {
                continue;
            }

            const int quadrant = (dx >= 0.0f ? 0 : 1) + (dy >= 0.0f ? 0 : 2);
            quadrants[quadrant] = true;
        }

        int occupiedQuadrants = 0;
        for (bool hasNeighbor : quadrants) {
            if (hasNeighbor) {
                ++occupiedQuadrants;
            }
        }

        return occupiedQuadrants >= min_neighbor_quadrants_;
    }

    bool getRobotPosition(){
        has_robot_pose_ = false;
        try{
            auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            
            robotPoint.x = transform.transform.translation.x;
            robotPoint.y = transform.transform.translation.y;
            robotPoint.z = transform.transform.translation.z;

            const auto &q = transform.transform.rotation;
            robot_yaw_ = static_cast<float>(std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
            has_robot_pose_ = true;

            updateLidarDeadZoneTransform();

            return true;
        }
        catch (tf2::TransformException& ex){ 
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return false;
        }
    }

    void updateLidarDeadZoneTransform(){
        has_lidar_from_map_transform_ = false;

        if (!lidar_dead_zone_enabled_) {
            return;
        }

        if (source_frame_.empty() || source_frame_ == map_frame_) {
            lidar_from_map_rot_.setIdentity();
            lidar_from_map_trans_.setZero();
            has_lidar_from_map_transform_ = true;
            return;
        }

        try {
            auto transform = tf_buffer_->lookupTransform(source_frame_, map_frame_, tf2::TimePointZero);
            const auto &t = transform.transform.translation;
            const auto &q = transform.transform.rotation;

            Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
            quat.normalize();
            lidar_from_map_rot_ = quat.toRotationMatrix();
            lidar_from_map_trans_ = Eigen::Vector3f(t.x, t.y, t.z);
            has_lidar_from_map_transform_ = true;
        }
        catch (tf2::TransformException& ex){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "LiDAR dead-zone transform failure (%s <- %s): %s",
                source_frame_.c_str(), map_frame_.c_str(), ex.what());
        }
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// Publish Map //////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void publishMap(){
        if (pubOccupancyMapLocal->get_subscription_count() == 0 &&
            pubOccupancyMapLocalInflated->get_subscription_count() == 0 &&
            pubElevationCloud->get_subscription_count() == 0)
            return;

        if (pubOccupancyMapLocal->get_subscription_count() > 0) {
            publishLocalOccupancyGrid();
        }
        if (pubOccupancyMapLocalInflated->get_subscription_count() > 0) {
            publishLocalOccupancyGridInflated();
        }
        // publishLocalOccupancyGridWithHeight();
        publishTraversabilityMap();
    }

    void publishLocalOccupancyGrid(){
        // Publish standard 2D occupancy grid
        occupancyMap2D.header.frame_id = map_frame_;
        occupancyMap2D.header.stamp = this->get_clock()->now();
        
        // Set map parameters
        occupancyMap2D.info.resolution = grid_resolution_m_;
        occupancyMap2D.info.width = output_map_cells_;
        occupancyMap2D.info.height = output_map_cells_;
        
        // Set origin
        if (use_robot_centered_origin_) {
            occupancyMap2D.info.origin.position.x = robotPoint.x - output_map_size_m_/2.0;
            occupancyMap2D.info.origin.position.y = robotPoint.y - output_map_size_m_/2.0;
        } else {
            occupancyMap2D.info.origin.position.x = fixed_origin_x_;
            occupancyMap2D.info.origin.position.y = fixed_origin_y_;
        }
        occupancyMap2D.info.origin.position.z = 0.0;
        occupancyMap2D.info.origin.orientation.x = 0.0;
        occupancyMap2D.info.origin.orientation.y = 0.0;
        occupancyMap2D.info.origin.orientation.z = 0.0;
        occupancyMap2D.info.origin.orientation.w = 1.0;
        
        // Fill data
        occupancyMap2D.data.clear();
        occupancyMap2D.data.resize(output_map_cells_ * output_map_cells_, -1);

        for (int i = 0; i < output_map_cells_; ++i) {
            for (int j = 0; j < output_map_cells_; ++j) {
                PointType point;
                point.x = occupancyMap2D.info.origin.position.x + i * grid_resolution_m_ + grid_resolution_m_/2.0;
                point.y = occupancyMap2D.info.origin.position.y + j * grid_resolution_m_ + grid_resolution_m_/2.0;
                point.z = 0.0f;

                int index = i + j * output_map_cells_;
                if (isInLidarDeadZone(point)) {
                    occupancyMap2D.data[index] = 100;
                    continue;
                }
                
                mapCell_t *cell = getOutputCellFromPoint(point);
                if (isCellKnownForOutput(cell)) {
                    occupancyMap2D.data[index] = int(cell->occupancy * 100);
                }
            }
        }

        const size_t cleared_cells = clearFootprintInGrid(occupancyMap2D);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Forced %zu footprint cells to occupancy cost 0 immediately before publishing %s",
            cleared_cells, output_local_topic_.c_str());
        pubOccupancyMapLocal->publish(occupancyMap2D);
    }

    void publishLocalOccupancyGridInflated(){
        const size_t cleared_cells = clearFootprintInGrid(occupancyMap2DInflated);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Forced %zu footprint cells to occupancy cost 0 immediately before publishing %s",
            cleared_cells, output_local_inflated_topic_.c_str());
        pubOccupancyMapLocalInflated->publish(occupancyMap2DInflated);
    }

    bool getGridIndexFromPoint(const PointType &point, int &gridX, int &gridY) const {
        gridX = static_cast<int>(std::floor((point.x - map_origin_x_) / grid_resolution_m_));
        gridY = static_cast<int>(std::floor((point.y - map_origin_y_) / grid_resolution_m_));

        return gridX >= 0 && gridX < map_width_cells_ &&
               gridY >= 0 && gridY < map_height_cells_;
    }

    mapCell_t* getCellAt(int gridX, int gridY) {
        if (gridX < 0 || gridX >= map_width_cells_ || gridY < 0 || gridY >= map_height_cells_) {
            return nullptr;
        }

        return &map_cells_[gridX + gridY * map_width_cells_];
    }

    mapCell_t* getCellFromPoint(PointType *point){
        int gridX, gridY;
        if (!getGridIndexFromPoint(*point, gridX, gridY)) {
            return nullptr;
        }

        return getCellAt(gridX, gridY);
    }

    float normalizeAngleDeg(float angle_deg) const {
        while (angle_deg <= -180.0f) angle_deg += 360.0f;
        while (angle_deg > 180.0f) angle_deg -= 360.0f;
        return angle_deg;
    }

    bool angleInSectorDeg(float angle_deg, float min_deg, float max_deg) const {
        angle_deg = normalizeAngleDeg(angle_deg);
        min_deg = normalizeAngleDeg(min_deg);
        max_deg = normalizeAngleDeg(max_deg);

        if (min_deg <= max_deg) {
            return angle_deg >= min_deg && angle_deg <= max_deg;
        }

        return angle_deg >= min_deg || angle_deg <= max_deg;
    }

    bool isInLidarDeadZone(const PointType &point) const {
        if (!lidar_dead_zone_enabled_ || lidar_dead_zone_radius_m_ <= 0.0f || !has_lidar_from_map_transform_) {
            return false;
        }

        const Eigen::Vector3f map_point(point.x, point.y, point.z);
        const Eigen::Vector3f lidar_point = lidar_from_map_rot_ * map_point + lidar_from_map_trans_;

        const float dist_sq = lidar_point.x() * lidar_point.x() + lidar_point.y() * lidar_point.y();
        if (dist_sq > lidar_dead_zone_radius_m_ * lidar_dead_zone_radius_m_) {
            return false;
        }

        const float angle_deg = std::atan2(lidar_point.y(), lidar_point.x()) * 180.0f / static_cast<float>(M_PI);
        return angleInSectorDeg(angle_deg, lidar_dead_zone_min_angle_deg_, lidar_dead_zone_max_angle_deg_);
    }

    bool isInRobotFootprint(const PointType &point) const {
        if (!footprint_output_clearing_enabled_ || footprint_clear_polygon_.size() < 3) {
            return false;
        }

        const float dx = point.x - robotPoint.x;
        const float dy = point.y - robotPoint.y;
        float x = dx;
        float y = dy;

        // In local mode the published grid is already in base_link and is
        // centered on the robot. Do not make footprint clearing depend on a
        // second TF condition in that mode. For a map-frame grid, transform
        // the cell into base_link using the robot pose as before.
        const bool robot_centered_base_grid =
            use_robot_centered_origin_ && map_frame_ == base_frame_;
        if (!robot_centered_base_grid) {
            if (!has_robot_pose_) {
                return false;
            }
            const float cos_yaw = std::cos(robot_yaw_);
            const float sin_yaw = std::sin(robot_yaw_);
            x = cos_yaw * dx + sin_yaw * dy;
            y = -sin_yaw * dx + cos_yaw * dy;
        }

        bool inside = false;
        for (size_t i = 0, j = footprint_clear_polygon_.size() - 1;
            i < footprint_clear_polygon_.size(); j = i++) {
            const auto &pi = footprint_clear_polygon_[i];
            const auto &pj = footprint_clear_polygon_[j];

            const float edge_x = pi[0] - pj[0];
            const float edge_y = pi[1] - pj[1];
            const float point_x = x - pj[0];
            const float point_y = y - pj[1];
            const float cross = edge_x * point_y - edge_y * point_x;
            if (std::abs(cross) <= 1e-6f &&
                x >= std::min(pi[0], pj[0]) - 1e-6f &&
                x <= std::max(pi[0], pj[0]) + 1e-6f &&
                y >= std::min(pi[1], pj[1]) - 1e-6f &&
                y <= std::max(pi[1], pj[1]) + 1e-6f) {
                return true;
            }

            const bool crosses_scanline = (pi[1] > y) != (pj[1] > y);
            if (crosses_scanline) {
                const float intersection_x =
                    pj[0] + (y - pj[1]) * (pi[0] - pj[0]) / (pi[1] - pj[1]);
                if (x < intersection_x) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    bool isFootprintObservationCell(const PointType &point) const {
        int grid_x;
        int grid_y;
        if (!getGridIndexFromPoint(point, grid_x, grid_y)) {
            return false;
        }

        return grid_x % footprint_observation_stride_cells_ == 0 &&
               grid_y % footprint_observation_stride_cells_ == 0;
    }

    // Restore the cells changed by injectFootprintObservationCells(). This is
    // called before processing the next real scan so synthetic observations do
    // not affect elevation fusion, slope fitting, or persist behind the rover.
    void restoreFootprintObservationCells() {
        for (const auto &backup : footprint_cell_backups_) {
            mapCell_t *cell = getCellAt(
                backup.grid_index % map_width_cells_,
                backup.grid_index / map_width_cells_);
            if (cell == nullptr) {
                continue;
            }

            cell->observeTimes = backup.observe_times;
            cell->elevation = backup.elevation;
            cell->elevationVar = backup.elevation_var;
            cell->occupancy = backup.occupancy;
            cell->occupancyVar = backup.occupancy_var;
            cell->needsUpdate = backup.needs_update;
            cell->xyz->z = backup.point_z;
            cell->xyz->intensity = backup.point_intensity;
        }
        footprint_cell_backups_.clear();
    }

    // Add sampled observations at centers of the existing internal grid.
    // Unknown cells become synthetic observations and already observed cells
    // retain their elevation, but every sampled footprint cost is forced to
    // zero. Their original state is restored before the next scan.
    size_t injectFootprintObservationCells() {
        restoreFootprintObservationCells();

        if (!footprint_output_clearing_enabled_ || footprint_clear_polygon_.size() < 3) {
            return 0;
        }

        const bool robot_centered_base_grid =
            use_robot_centered_origin_ && map_frame_ == base_frame_;
        if (!robot_centered_base_grid && !has_robot_pose_) {
            return 0;
        }

        const float yaw = robot_centered_base_grid ? 0.0f : robot_yaw_;
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &vertex : footprint_clear_polygon_) {
            const float map_x = robotPoint.x + cos_yaw * vertex[0] - sin_yaw * vertex[1];
            const float map_y = robotPoint.y + sin_yaw * vertex[0] + cos_yaw * vertex[1];
            min_x = std::min(min_x, map_x);
            min_y = std::min(min_y, map_y);
            max_x = std::max(max_x, map_x);
            max_y = std::max(max_y, map_y);
        }

        const int min_grid_x = std::max(
            0, static_cast<int>(std::floor((min_x - map_origin_x_) / grid_resolution_m_)) - 1);
        const int min_grid_y = std::max(
            0, static_cast<int>(std::floor((min_y - map_origin_y_) / grid_resolution_m_)) - 1);
        const int max_grid_x = std::min(
            map_width_cells_ - 1,
            static_cast<int>(std::floor((max_x - map_origin_x_) / grid_resolution_m_)) + 1);
        const int max_grid_y = std::min(
            map_height_cells_ - 1,
            static_cast<int>(std::floor((max_y - map_origin_y_) / grid_resolution_m_)) + 1);

        if (min_grid_x <= max_grid_x && min_grid_y <= max_grid_y) {
            const size_t bounding_cell_count =
                static_cast<size_t>(max_grid_x - min_grid_x + 1) *
                static_cast<size_t>(max_grid_y - min_grid_y + 1);
            footprint_cell_backups_.reserve(bounding_cell_count);
        }

        for (int grid_y = min_grid_y; grid_y <= max_grid_y; ++grid_y) {
            for (int grid_x = min_grid_x; grid_x <= max_grid_x; ++grid_x) {
                mapCell_t *cell = getCellAt(grid_x, grid_y);
                if (cell == nullptr || !isFootprintObservationCell(*cell->xyz) ||
                    !isInRobotFootprint(*cell->xyz)) {
                    continue;
                }

                footprint_cell_backups_.push_back({
                    cell->grid.gridIndex,
                    cell->observeTimes,
                    cell->elevation,
                    cell->elevationVar,
                    cell->occupancy,
                    cell->occupancyVar,
                    cell->needsUpdate,
                    cell->xyz->z,
                    cell->xyz->intensity});

                if (cell->observeTimes <= 0 || !std::isfinite(cell->elevation)) {
                    // A synthetic observation needs a finite height for the
                    // PointCloud2 output. It is excluded from calculations and
                    // restored before the next scan, so this height is only a
                    // visualization placeholder.
                    cell->elevation = robotPoint.z;
                    cell->elevationVar = 0.0f;
                    cell->xyz->z = robotPoint.z;
                }
                cell->observeTimes = std::max(1, cell->observeTimes);
                cell->occupancy = 0.0f;
                cell->occupancyVar = 0.0f;
                cell->needsUpdate = false;
                cell->xyz->intensity = 0.0f;
            }
        }

        return footprint_cell_backups_.size();
    }

    // Rasterize sampled robot-footprint cells into an existing occupancy-grid-
    // sized mask. Only cells in the footprint bounding box require a polygon
    // test; all later full-map passes use a constant-time mask lookup.
    size_t markFootprintCellsInGrid(
        const nav_msgs::msg::OccupancyGrid &grid,
        std::vector<uint8_t> &footprint_mask) const {
        const size_t grid_cell_count =
            static_cast<size_t>(grid.info.width) * static_cast<size_t>(grid.info.height);
        if (!footprint_output_clearing_enabled_ || footprint_clear_polygon_.size() < 3 ||
            grid.info.resolution <= 0.0 || grid.info.width == 0 || grid.info.height == 0 ||
            footprint_mask.size() != grid_cell_count) {
            return 0;
        }

        const bool robot_centered_base_grid =
            use_robot_centered_origin_ && map_frame_ == base_frame_;
        if (!robot_centered_base_grid && !has_robot_pose_) {
            return 0;
        }

        const float yaw = robot_centered_base_grid ? 0.0f : robot_yaw_;
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &vertex : footprint_clear_polygon_) {
            const float map_x = robotPoint.x + cos_yaw * vertex[0] - sin_yaw * vertex[1];
            const float map_y = robotPoint.y + sin_yaw * vertex[0] + cos_yaw * vertex[1];
            min_x = std::min(min_x, map_x);
            min_y = std::min(min_y, map_y);
            max_x = std::max(max_x, map_x);
            max_y = std::max(max_y, map_y);
        }

        const float origin_x = static_cast<float>(grid.info.origin.position.x);
        const float origin_y = static_cast<float>(grid.info.origin.position.y);
        const float resolution = grid.info.resolution;
        const int width = static_cast<int>(grid.info.width);
        const int height = static_cast<int>(grid.info.height);
        const int min_cell_x = std::max(
            0, static_cast<int>(std::floor((min_x - origin_x) / resolution)) - 1);
        const int min_cell_y = std::max(
            0, static_cast<int>(std::floor((min_y - origin_y) / resolution)) - 1);
        const int max_cell_x = std::min(
            width - 1, static_cast<int>(std::floor((max_x - origin_x) / resolution)) + 1);
        const int max_cell_y = std::min(
            height - 1, static_cast<int>(std::floor((max_y - origin_y) / resolution)) + 1);

        size_t marked_cells = 0;
        for (int y = min_cell_y; y <= max_cell_y; ++y) {
            for (int x = min_cell_x; x <= max_cell_x; ++x) {
                PointType point;
                point.x = origin_x + (static_cast<float>(x) + 0.5f) * resolution;
                point.y = origin_y + (static_cast<float>(y) + 0.5f) * resolution;
                point.z = 0.0f;
                if (!isFootprintObservationCell(point) || !isInRobotFootprint(point)) {
                    continue;
                }

                footprint_mask[
                    static_cast<size_t>(x) + static_cast<size_t>(y) * grid.info.width] = 1;
                ++marked_cells;
            }
        }
        return marked_cells;
    }

    // This is intentionally the last operation on an OccupancyGrid before it
    // is published. It synthesizes sampled free cells inside the configured
    // footprint even when the upstream self-filter removed lidar returns there.
    size_t clearFootprintInGrid(nav_msgs::msg::OccupancyGrid &grid) const {
        if (!footprint_output_clearing_enabled_ || footprint_clear_polygon_.size() < 3 ||
            grid.info.resolution <= 0.0 || grid.info.width == 0 || grid.info.height == 0 ||
            grid.data.size() != static_cast<size_t>(grid.info.width) * grid.info.height) {
            return 0;
        }

        const bool robot_centered_base_grid =
            use_robot_centered_origin_ && map_frame_ == base_frame_;
        if (!robot_centered_base_grid && !has_robot_pose_) {
            return 0;
        }

        const float yaw = robot_centered_base_grid ? 0.0f : robot_yaw_;
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &vertex : footprint_clear_polygon_) {
            const float map_x = robotPoint.x + cos_yaw * vertex[0] - sin_yaw * vertex[1];
            const float map_y = robotPoint.y + sin_yaw * vertex[0] + cos_yaw * vertex[1];
            min_x = std::min(min_x, map_x);
            min_y = std::min(min_y, map_y);
            max_x = std::max(max_x, map_x);
            max_y = std::max(max_y, map_y);
        }

        const float origin_x = static_cast<float>(grid.info.origin.position.x);
        const float origin_y = static_cast<float>(grid.info.origin.position.y);
        const float resolution = grid.info.resolution;
        const int width = static_cast<int>(grid.info.width);
        const int height = static_cast<int>(grid.info.height);
        const int min_cell_x = std::max(0, static_cast<int>(std::floor((min_x - origin_x) / resolution)) - 1);
        const int min_cell_y = std::max(0, static_cast<int>(std::floor((min_y - origin_y) / resolution)) - 1);
        const int max_cell_x = std::min(width - 1, static_cast<int>(std::floor((max_x - origin_x) / resolution)) + 1);
        const int max_cell_y = std::min(height - 1, static_cast<int>(std::floor((max_y - origin_y) / resolution)) + 1);

        size_t cleared_cells = 0;
        for (int y = min_cell_y; y <= max_cell_y; ++y) {
            for (int x = min_cell_x; x <= max_cell_x; ++x) {
                PointType point;
                point.x = origin_x + (static_cast<float>(x) + 0.5f) * resolution;
                point.y = origin_y + (static_cast<float>(y) + 0.5f) * resolution;
                point.z = 0.0f;
                if (!isFootprintObservationCell(point) || !isInRobotFootprint(point)) {
                    continue;
                }

                grid.data[static_cast<size_t>(x) + static_cast<size_t>(y) * grid.info.width] = 0;
                ++cleared_cells;
            }
        }
        return cleared_cells;
    }

    void publishTraversabilityMap(){
        if (pubElevationCloud->get_subscription_count() == 0)
            return;

        int robotGridX, robotGridY;
        if (!getGridIndexFromPoint(robotPoint, robotGridX, robotGridY)) {
            return;
        }

        const int visualRadiusCells = static_cast<int>(std::ceil(visualizationRadius / grid_resolution_m_));
        const float visualRadiusSq = visualizationRadius * visualizationRadius;
        laserCloudElevation->clear();
        
        for (int dx = -visualRadiusCells; dx <= visualRadiusCells; ++dx){
            for (int dy = -visualRadiusCells; dy <= visualRadiusCells; ++dy){
                const float distSq = static_cast<float>(dx * dx + dy * dy) * grid_resolution_m_ * grid_resolution_m_;
                if (distSq > visualRadiusSq) {
                    continue;
                }

                mapCell_t *cell = getCellAt(robotGridX + dx, robotGridY + dy);
                if (cell != nullptr && cell->observeTimes > 0){
                    PointType p = *(cell->xyz);
                    p.intensity = cell->occupancy;
                    laserCloudElevation->push_back(p);
                }
            }
        }

        // Publish elevation cloud
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = map_frame_;
        laserCloudTemp.header.stamp = this->get_clock()->now();
        pubElevationCloud->publish(laserCloudTemp);
    }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    auto tMapping = std::make_shared<TraversabilityMapping>();

    std::thread predictionThread(&TraversabilityMapping::TraversabilityThread, tMapping.get());

    RCLCPP_INFO(tMapping->get_logger(), "Traversability Mapping Started.");

    rclcpp::spin(tMapping);

    rclcpp::shutdown();

    if (predictionThread.joinable()) {
    predictionThread.join();
    }

    return 0;
}
