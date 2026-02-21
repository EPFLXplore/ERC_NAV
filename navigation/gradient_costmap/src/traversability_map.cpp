#include "utility.h"

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
    // rclcpp::Publisher<elevation_msgs::msg::OccupancyElevation>::SharedPtr pubOccupancyMapLocalHeight;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubElevationCloud;
    // Point Cloud Pointer
    pcl::PointCloud<PointType>::Ptr laserCloud; // save input filtered laser cloud for mapping
    pcl::PointCloud<PointType>::Ptr laserCloudElevation; // a cloud for publishing elevation map
    // Occupancy Grid Map
    nav_msgs::msg::OccupancyGrid occupancyMap2D; // local occupancy grid map
    // elevation_msgs::msg::OccupancyElevation occupancyMap2DHeight; // customized message that includes occupancy map and elevation info

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

public:
    TraversabilityMapping() : Node("traversability_mapping"),
        pubCount(1),
        mapArrayCount(0) {

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subFilteredGroundCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_pointcloud", 10, std::bind(&TraversabilityMapping::cloudHandler, this, std::placeholders::_1));

        // /filtered_pointcloud

        pubOccupancyMapLocal = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_map_local", 10);
        // pubOccupancyMapLocalHeight = this->create_publisher<elevation_msgs::msg::OccupancyElevation>("/occupancy_map_local_height", 10);
        pubElevationCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/elevation_pointcloud", 5);

        allocateMemory();
    }

    ~TraversabilityMapping(){}

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
        
        std::lock_guard<std::mutex> lock(mtx);
        
        auto t1 = std::chrono::high_resolution_clock::now();
        if (getRobotPosition() == false) return;
        
        auto t2 = std::chrono::high_resolution_clock::now();
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);
        
        auto t3 = std::chrono::high_resolution_clock::now();
        updateElevationMap();
        
        auto t4 = std::chrono::high_resolution_clock::now();
        publishMap();
        
        auto end = std::chrono::high_resolution_clock::now();
        
        // Print timing breakdown
        auto lock_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - start).count();
        auto tf_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto convert_time = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto update_time = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto publish_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - t4).count();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        
        if (total_time > 100) {  // If processing takes > 100ms
            RCLCPP_WARN(this->get_logger(), 
                "Slow processing: total=%ldms (lock=%ld, tf=%ld, convert=%ld, update=%ld, publish=%ld)",
                total_time, lock_time, tf_time, convert_time, update_time, publish_time);
        }
    }

    void updateElevationMap(){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            // laserCloud->points[i].z -= 0.2; // TODO: CHANGEME, commented for visualization
            updateElevationMap(&laserCloud->points[i]);
        }
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
        // Skip updating elevation if we have observed this cell enough times
        // if (cell->observeTimes > traversabilityObserveTimeTh)
        //     return;

        float z = point->z;
        float var = 0.01; // measurement noise

        float mu = cell->elevation;
        float sigma = cell->elevationVar;

        if (cell->observeTimes <= 1){
            cell->updateElevation(z, var);
        } else {
            float K = sigma / (sigma + var);
            float mu_new = mu + K * (z - mu);
            float sigma_new = (1 - K) * sigma;
            cell->updateElevation(mu_new, sigma_new);
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
                return;
            
            cellsToProcess.swap(observingList1);  // Fast swap
        } 
        
        // Process without lock
        for (auto cell : cellsToProcess) {
            calculateTraversability(cell);
        }
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

        // if (maxDifference > filterHeightLimit) {  // filterHeightLimit = 0.5m
        //     RCLCPP_ERROR(this->get_logger(), "Large height difference: %.2f m", maxDifference);
        //     cell->updateOccupancy(1.0);
        //     return;
        // }

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
        float searchRadius = 0.4; // meters
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
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
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
        if (pubOccupancyMapLocal->get_subscription_count() == 0)
            // pubOccupancyMapLocalHeight->get_subscription_count() == 0)
            return;

        publishLocalOccupancyGrid();
        // publishLocalOccupancyGridWithHeight();
        publishTraversabilityMap();
    }

    void publishLocalOccupancyGrid(){
        // Publish standard 2D occupancy grid
        occupancyMap2D.header.frame_id = "map";
        occupancyMap2D.header.stamp = this->get_clock()->now();
        
        // Set map parameters
        occupancyMap2D.info.resolution = mapResolution;
        occupancyMap2D.info.width = localMapArrayLength;
        occupancyMap2D.info.height = localMapArrayLength;
        
        // Set origin
        occupancyMap2D.info.origin.position.x = robotPoint.x - localMapLength/2.0;
        occupancyMap2D.info.origin.position.y = robotPoint.y - localMapLength/2.0;
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
                if (cell != NULL && cell->observeTimes > 0) {
                    int index = i + j * localMapArrayLength;
                    occupancyMap2D.data[index] = int(cell->occupancy * 100);
                }
            }
        }

        pubOccupancyMapLocal->publish(occupancyMap2D);
    }

    // void publishLocalOccupancyGridWithHeight(){
    //     if (pubOccupancyMapLocalHeight->get_subscription_count() == 0)
    //         return;

    //     // Copy occupancy grid structure
    //     occupancyMap2DHeight.occupancy = occupancyMap2D;
        
    //     // Reset and populate elevation data
    //     occupancyMap2DHeight.elevation.clear();
    //     occupancyMap2DHeight.elevation.resize(mapArrayLength * mapArrayLength, 0.0);

    //     for (int i = 0; i < mapArrayLength; ++i){
    //         for (int j = 0; j < mapArrayLength; ++j){
    //             PointType point;
    //             point.x = occupancyMap2DHeight.occupancy.info.origin.position.x + i * mapResolution + mapResolution/2.0;
    //             point.y = occupancyMap2DHeight.occupancy.info.origin.position.y + j * mapResolution + mapResolution/2.0;
                
    //             mapCell_t *cell = getCellFromPoint(&point);
    //             if (cell != NULL && cell->observeTimes > 0) {
    //                 int index = i + j * mapArrayLength;
    //                 occupancyMap2DHeight.elevation[index] = cell->elevation;
    //             }
    //         }
    //     }

    //     pubOccupancyMapLocalHeight->publish(occupancyMap2DHeight);
    // }

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
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = this->get_clock()->now();
        pubElevationCloud->publish(laserCloudTemp);
    }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    auto tMapping = std::make_shared<TraversabilityMapping>();

    std::thread predictionThread(&TraversabilityMapping::TraversabilityThread, tMapping.get());

    RCLCPP_INFO(tMapping->get_logger(), "Traversability Mapping Started.");
    RCLCPP_INFO(tMapping->get_logger(), "Traversability Mapping Scenario: %s", 
        urbanMapping ? "Urban" : "Terrain");

    rclcpp::spin(tMapping);

    rclcpp::shutdown();

    return 0;
}