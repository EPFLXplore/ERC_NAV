#include "utility.h"
#include <unordered_map>

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

    int pubCount;
    
    // Map Arrays
    int mapArrayCount;
    int **mapArrayInd; // it saves the index of this submap in vector mapArray
    int **predictionArrayFlag;
    vector<childMap_t*> mapArray;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    vector<mapCell_t*> observingList1; // thread 1: save new observed cells
    vector<mapCell_t*> observingList2; // thread 2: calculate traversability of new observed cells

    // Inflation Parameters
    int cost;
    int cost_inflated;
    float cost_kernel;
    std::string pointcloud_topic_;
    std::string output_local_topic_;
    std::string output_local_inflated_topic_;
    std::string output_elevation_topic_;
    std::string map_frame_;
    std::string base_frame_;
    bool use_robot_centered_origin_;
    double fixed_origin_x_;
    double fixed_origin_y_;
    uint64_t current_scan_id_;
    std::unordered_map<mapCell_t*, uint64_t> last_observed_scan_;
    int radius_inflation = this->declare_parameter<int>("inflation_radius", 5); // in cells
    float alpha_inflation = this->declare_parameter<float>("inflation_factor", 1.0f); // inflation factor, can be tuned based on how much we want to inflate
    float sigmoid_k = this->declare_parameter<float>("sigmoid_k", 0.5f); // weight for slope in occupancy calculation
    float sigmoid_x0 = this->declare_parameter<float>("sigmoid_x0", 0.5f); // threshold for slope in occupancy calculation
    
    
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
        pubCount(1),
        mapArrayCount(0),
        current_scan_id_(0) {

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/cloud_pcd");
        output_local_topic_ = this->declare_parameter<std::string>("output_local_topic", "/occupancy_map_local");
        output_local_inflated_topic_ = this->declare_parameter<std::string>("output_local_inflated_topic", "/occupancy_map_local_inflated");
        output_elevation_topic_ = this->declare_parameter<std::string>("output_elevation_topic", "/elevation_pointcloud");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        use_robot_centered_origin_ = this->declare_parameter<bool>("use_robot_centered_origin", true);
        fixed_origin_x_ = this->declare_parameter<double>("fixed_origin_x", -localMapLength / 2.0);
        fixed_origin_y_ = this->declare_parameter<double>("fixed_origin_y", -localMapLength / 2.0);

        subFilteredGroundCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&TraversabilityMapping::cloudHandler, this, std::placeholders::_1));
        
        // CHANGE ME:
        // 1. If you want to use the gazebo pointcloud use /filtered_pointcloud
        // 2. If you want to use the marsyard pcd file, use /cloud_pcd

        pubOccupancyMapLocal = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_local_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        pubOccupancyMapLocalInflated = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_local_inflated_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        // pubOccupancyMapLocalHeight = this->create_publisher<elevation_msgs::msg::OccupancyElevation>("/occupancy_map_local_height", 10);
        pubElevationCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_elevation_topic_, 5);

        allocateMemory();
    }

    ~TraversabilityMapping(){}

    bool isCellFreshForOutput(const mapCell_t *cell) const {
        if (cell == nullptr || cell->observeTimes <= 0) {
            return false;
        }

        // In global mode, keep long-term map memory.
        if (!use_robot_centered_origin_) {
            return true;
        }

        auto it = last_observed_scan_.find(const_cast<mapCell_t*>(cell));
        if (it == last_observed_scan_.end()) {
            return false;
        }

        // In local mode we only publish values from the current measurement cycle.
        return it->second == current_scan_id_;
    }

    void allocateMemory(){
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());

        mapArrayInd = new int*[mapArrayLength];
        predictionArrayFlag = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i){
            mapArrayInd[i] = new int[mapArrayLength];
            predictionArrayFlag[i] = new int[mapArrayLength];
        }

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                mapArrayInd[i][j] = -1;

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                predictionArrayFlag[i][j] = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        auto start = std::chrono::high_resolution_clock::now();
        ++current_scan_id_;

        // Limit the speed of the callback
        if (std::chrono::duration_cast<std::chrono::milliseconds>(start - last_time).count() < cloudHandlerRate)
        {
            // RCLCPP_WARN(this->get_logger(), "TOO EARLY");
            return;
        }
        last_time = start;
        
        // Lock the the processes to prevent new data from interrupting old data
        std::lock_guard<std::mutex> lock(mtx);
        
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
        updateOccupancyGrid();

        auto t5 = std::chrono::high_resolution_clock::now();
        publishMap();
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
        // 1. Get submap index
        int cubeX, cubeY;
        getPointCubeIndex(&cubeX, &cubeY, point);
        if (cubeX < 0 || cubeX >= mapArrayLength || cubeY < 0 || cubeY >= mapArrayLength)
            return;

        // 2. Assign submap ID
        if (mapArrayInd[cubeX][cubeY] == -1){
            childMap_t *childMap = new childMap_t(mapArrayCount, cubeX, cubeY);
            mapArrayCount++;
            mapArray.push_back(childMap);
            mapArrayInd[cubeX][cubeY] = childMap->subInd;
        }

        // 3. Get submap that the point belongs to
        childMap_t *subMap = mapArray[mapArrayInd[cubeX][cubeY]];

        // 4. Get grid index
        int gridX = (point->x - subMap->originX) / mapResolution;
        int gridY = (point->y - subMap->originY) / mapResolution;
        if (gridX < 0 || gridY < 0 || gridX >= mapCubeArrayLength || gridY >= mapCubeArrayLength)
            return;

        // 5. Update cell
        updateOccupancyCell(subMap->cellArray[gridX][gridY], point);
    }

    void updateOccupancyCell(mapCell_t *cell, PointType *point){
        // if (point->intensity == 100) // obstacle
        //     RCLCPP_ERROR(this->get_logger(), "OBSTACLE");
        //     // updateOccupancyBel(cell, true);      #### THIS IS BECAUSE I DON't USE INTENSITY
        // else if (point->intensity == 0) // free
        // updateOccupancyBel(cell, false);
        
        cell->observeTimes++;
        last_observed_scan_[cell] = current_scan_id_;
        
        updateElevationBGK(cell, point);    // fuses elevation from all points within a cell
        
        observingList1.push_back(cell);
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
        occupancyMap2DInflated.info.resolution = mapResolution;
        occupancyMap2DInflated.info.width = localMapArrayLength;
        occupancyMap2DInflated.info.height = localMapArrayLength;
        
        // Set origin
        if (use_robot_centered_origin_) {
            occupancyMap2DInflated.info.origin.position.x = robotPoint.x - localMapLength/2.0;
            occupancyMap2DInflated.info.origin.position.y = robotPoint.y - localMapLength/2.0;
        } else {
            occupancyMap2DInflated.info.origin.position.x = fixed_origin_x_;
            occupancyMap2DInflated.info.origin.position.y = fixed_origin_y_;
        }
        occupancyMap2DInflated.info.origin.position.z = 0.0;
        // Fill data
        occupancyMap2DInflated.data.clear();
        occupancyMap2DInflated.data.resize(localMapArrayLength * localMapArrayLength, -1);


        for (int i = 0; i < localMapArrayLength; ++i) {
            for (int j = 0; j < localMapArrayLength; ++j) {
                PointType point;
                point.x = occupancyMap2DInflated.info.origin.position.x + i * mapResolution + mapResolution/2.0;
                point.y = occupancyMap2DInflated.info.origin.position.y + j * mapResolution + mapResolution/2.0;
                
                mapCell_t *cell = getCellFromPoint(&point);
                for (int m = -radius_inflation; m <= radius_inflation; ++m) {
                    for (int n = -radius_inflation; n <= radius_inflation; ++n) {
                        int x = i + m;
                        int y = j + n;
                        if (x < 0 || x >= localMapArrayLength || y < 0 || y >= localMapArrayLength)
                            continue;
                        if (isCellFreshForOutput(cell)) {
                            int cell_idx = i + j * localMapArrayLength;
                            int kernel_idx = x + y * localMapArrayLength;
                            // Value contained between 0 and 100, represent probability. 

                            // Update through sigmoid function to make the gradient sharper, can be tuned by changing sigmoid_k and sigmoid_x0
                            cost = int(applySigmoidToOccupancy(cell->occupancy) * 100);
                            // Kernel is the influence of the cell on its neighbors

                            // Kernel with distances
                            // cost_kernel = 1.0f - sqrt(m*m + n*n) / (radius_inflation+1); // simple linear kernel, can be changed to more complex ones
                            // cost_kernel = std::max(cost_kernel, 0.0f); // limit kernel value to -1, cost kernel between 0 and -1, where 0 means no inflation and -1 means full inflation
                            // cost_inflated = int(cost * (1 + alpha_inflation*cost_kernel));

                            // Kernel with exponential decay on distances
                            cost_kernel = std::exp(- alpha_inflation * sqrt(m*m + n*n) / (radius_inflation+1)); // exponential decay kernel, can be changed to more complex ones
                            cost_inflated = int(cost * (1 + cost_kernel));
                            // Keep and map the cost inflated between 0 and 100
                            cost_inflated = std::min(cost_inflated, 100);
                            if (occupancyMap2DInflated.data[kernel_idx] < cost_inflated) {
                                occupancyMap2DInflated.data[kernel_idx] = cost_inflated;
                            }
                        }
                    }
                }
            }
        }
    }

    void getPointCubeIndex(int *cubeX, int *cubeY, PointType *point){
        // Convert world coordinates (meters) to submap grid indices
        // Each submap is mapCubeLength × mapCubeLength (10m × 10m)
        // rootCubeIndex shifts the coordinate system to handle negative coordinates
        
        // Calculate which submap grid cell the point belongs to
        // Add mapCubeLength/2.0 to handle points centered on grid boundaries
        // Divide by mapCubeLength to get grid index
        // Add rootCubeIndex to offset from origin (allows negative world coordinates)
        *cubeX = int((point->x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        *cubeY = int((point->y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;

        // Handle negative coordinates correctly
        // Integer division rounds toward zero, but we need floor behavior for negative numbers
        // If point is in negative territory, decrement the index by 1
        if (point->x + mapCubeLength/2.0 < 0)  --*cubeX;
        if (point->y + mapCubeLength/2.0 < 0)  --*cubeY;
    }

    void TraversabilityThread(){
        rclcpp::Rate rate(10); // Hz

        while (rclcpp::ok()){
            traversabilityMapCalculation();
            rate.sleep();
        }
    }

    void traversabilityMapCalculation(){
        // Copy data with lock

        vector<mapCell_t*> cellsToProcess;
        {
            std::lock_guard<std::mutex> lock(mtx);

            if (observingList1.empty())
            {
                return;
            
            } 
            cellsToProcess.swap(observingList1);
        }
        
            // Process without lock
            for (auto cell : cellsToProcess) {
                calculateTraversability(cell);
            }
            RCLCPP_WARN(this->get_logger(), "Calculating traversability");
    }

    void calculateTraversability(mapCell_t *cell)
    {
        // BGK-based traversability analysis
        vector<PointType> neighborPoints;
        getNeighborCells(cell, neighborPoints);

        if (neighborPoints.size() < 3)
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
        
        // ===== Height Pre-Check =====
        float minElevation = matPoints.col(2).minCoeff();
        float maxElevation = matPoints.col(2).maxCoeff();
        float maxDifference = maxElevation - minElevation;

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
        slopeCost = slopeAngle / filterAngleLimit;
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
        roughness = roughness / filterMaxRoughness;
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
        // Get neighboring cells within a radius for analysis
        float searchRadius = 0.6; // meters
        int searchGrids = searchRadius / mapResolution;

        grid_t grid = cell->grid;
        childMap_t *subMap = mapArray[grid.mapID];

        for (int i = -searchGrids; i <= searchGrids; ++i) {
            for (int j = -searchGrids; j <= searchGrids; ++j) {
                int x = grid.gridX + i;
                int y = grid.gridY + j;
                
                if (x < 0 || x >= mapCubeArrayLength || y < 0 || y >= mapCubeArrayLength)
                    continue;
                    
                mapCell_t *neighborCell = subMap->cellArray[x][y];
                if (neighborCell->observeTimes > 0) {
                    PointType p;
                    p.x = neighborCell->xyz->x;
                    p.y = neighborCell->xyz->y;
                    p.z = neighborCell->elevation;
                    neighborPoints.push_back(p);
                }
            }
        }
    }

    bool getRobotPosition(){
        try{
            auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            
            robotPoint.x = transform.transform.translation.x;
            robotPoint.y = transform.transform.translation.y;
            robotPoint.z = transform.transform.translation.z;

            return true;
        }
        catch (tf2::TransformException& ex){ 
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return false;
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
        occupancyMap2D.info.resolution = mapResolution;
        occupancyMap2D.info.width = localMapArrayLength;
        occupancyMap2D.info.height = localMapArrayLength;
        
        // Set origin
        if (use_robot_centered_origin_) {
            occupancyMap2D.info.origin.position.x = robotPoint.x - localMapLength/2.0;
            occupancyMap2D.info.origin.position.y = robotPoint.y - localMapLength/2.0;
        } else {
            occupancyMap2D.info.origin.position.x = fixed_origin_x_;
            occupancyMap2D.info.origin.position.y = fixed_origin_y_;
        }
        occupancyMap2D.info.origin.position.z = 0.0;
        
        // Fill data
        occupancyMap2D.data.clear();
        occupancyMap2D.data.resize(localMapArrayLength * localMapArrayLength, -1);

        for (int i = 0; i < localMapArrayLength; ++i) {
            for (int j = 0; j < localMapArrayLength; ++j) {
                PointType point;
                point.x = occupancyMap2D.info.origin.position.x + i * mapResolution + mapResolution/2.0;
                point.y = occupancyMap2D.info.origin.position.y + j * mapResolution + mapResolution/2.0;
                
                mapCell_t *cell = getCellFromPoint(&point);
                if (isCellFreshForOutput(cell)) {
                    int index = i + j * localMapArrayLength;
                    occupancyMap2D.data[index] = int(cell->occupancy * 100);
                }
            }
        }

        pubOccupancyMapLocal->publish(occupancyMap2D);
    }

    void publishLocalOccupancyGridInflated(){
        

        pubOccupancyMapLocalInflated->publish(occupancyMap2DInflated);
    }

    mapCell_t* getCellFromPoint(PointType *point){
        int cubeX, cubeY;
        getPointCubeIndex(&cubeX, &cubeY, point);
        
        if (cubeX < 0 || cubeX >= mapArrayLength || cubeY < 0 || cubeY >= mapArrayLength)
            return NULL;
            
        if (mapArrayInd[cubeX][cubeY] == -1)
            return NULL;
            
        childMap_t *subMap = mapArray[mapArrayInd[cubeX][cubeY]];
        
        int gridX = (point->x - subMap->originX) / mapResolution;
        int gridY = (point->y - subMap->originY) / mapResolution;
        
        if (gridX < 0 || gridY < 0 || gridX >= mapCubeArrayLength || gridY >= mapCubeArrayLength)
            return NULL;
            
        return subMap->cellArray[gridX][gridY];
    }

    void publishTraversabilityMap(){
        if (pubElevationCloud->get_subscription_count() == 0)
            return;

        // 1. Find robot current cube index
        int currentCubeX, currentCubeY;
        getPointCubeIndex(&currentCubeX, &currentCubeY, &robotPoint);
        
        // 2. Loop through all the sub-maps that are nearby
        int visualLength = int(visualizationRadius / mapCubeLength);
        laserCloudElevation->clear();
        
        for (int i = -visualLength; i <= visualLength; ++i){
            for (int j = -visualLength; j <= visualLength; ++j){

                if (sqrt(float(i*i+j*j)) >= visualLength) continue;

                int idx = i + currentCubeX;
                int idy = j + currentCubeY;

                if (idx < 0 || idx >= mapArrayLength ||  idy < 0 || idy >= mapArrayLength) continue;

                if (mapArrayInd[idx][idy] == -1) continue;

                childMap_t *subMap = mapArray[mapArrayInd[idx][idy]];

                for (int m = 0; m < mapCubeArrayLength; ++m){
                    for (int n = 0; n < mapCubeArrayLength; ++n){
                        if (subMap->cellArray[m][n]->observeTimes > 0){
                            PointType p = *(subMap->cellArray[m][n]->xyz);
                            p.intensity = subMap->cellArray[m][n]->occupancy;
                            laserCloudElevation->push_back(p);
                        }
                    }
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