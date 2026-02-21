#include "utility.h"
#include <cmath>

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
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
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
    bool **obstFlag;
    bool **initFlag;
    int cloudWidth = 0;
    int cloudHeight = 0;


public:
    TraversabilityFilter() : Node("traversability_filter") {
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster_points", 10, std::bind(&TraversabilityFilter::cloudHandler, this, std::placeholders::_1));

        pubCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
        // pubCloudVisualHiRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_high_res", 5);
        // pubCloudVisualLowRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_low_res", 5);
        pubLaserScan = this->create_publisher<sensor_msgs::msg::LaserScan>("/pointcloud_2_laserscan", 5);

        allocateMemory();

        pointcloud2laserscanInitialization();
    }

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32SC1, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32FC1, cv::Scalar::all(FLT_MAX));

        minHeight = new float*[filterHeightMapArrayLength];
        maxHeight = new float*[filterHeightMapArrayLength];
        obstFlag = new bool*[filterHeightMapArrayLength];
        initFlag = new bool*[filterHeightMapArrayLength];

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            minHeight[i] = new float[filterHeightMapArrayLength];
            maxHeight[i] = new float[filterHeightMapArrayLength];
            obstFlag[i] = new bool[filterHeightMapArrayLength];
            initFlag[i] = new bool[filterHeightMapArrayLength];
        }
    }

    void resetParameters(){
        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();
        
        for (int i = 0; i < N_SCAN; ++i)
            fill(laserCloudMatrix[i].begin(), laserCloudMatrix[i].end(), PointType());
        
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            fill(minHeight[i], minHeight[i] + filterHeightMapArrayLength, FLT_MAX);
            fill(maxHeight[i], maxHeight[i] + filterHeightMapArrayLength, -FLT_MAX);
            fill(obstFlag[i], obstFlag[i] + filterHeightMapArrayLength, false);
            fill(initFlag[i], initFlag[i] + filterHeightMapArrayLength, false);
        }
    }

    ~TraversabilityFilter(){}


    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        // RCLCPP_INFO(this->get_logger(), "=== Starting cloudHandler ===");
        // RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", 
                    // laserCloudMsg->width * laserCloudMsg->height);

        cloudWidth = laserCloudMsg->width;
        cloudHeight = laserCloudMsg->height;

        // Step 1: Convert ROS message to PCL and extract basic range info
        extractRawCloud(laserCloudMsg);         
        // RCLCPP_INFO(this->get_logger(), "✓ Raw cloud extracted");
        
        // Step 2: Transform point cloud from sensor frame to map frame
        if (transformCloud() == false) {
            RCLCPP_ERROR(this->get_logger(), "✗ Transform failed, skipping frame");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "✓ Cloud transformed");
        
        // Step 3: Organize points into scan line matrix format
        cloud2Matrix();
        // RCLCPP_INFO(this->get_logger(), "✓ Cloud converted to matrix");
        
        // // Step 4: Apply obstacle detection filters (curbs, slopes, distance)
        // applyFilter(); // Not used since we are using a gradient, so no obstacles defined here
        // RCLCPP_INFO(this->get_logger(), "✓ Filters applied");
        
        // Step 5: Extract filtered points with obstacle labels
        // extractFilteredCloud();
        // RCLCPP_INFO(this->get_logger(), "✓ Filtered cloud extracted");
        
        // Step 6: Convert to regular grid and downsample
        downsampleCloud();
        // RCLCPP_INFO(this->get_logger(), "✓ Cloud downsampled");
        
        // Step 7: Use Gaussian Process to predict missing areas
        predictCloudBGK();
        // RCLCPP_INFO(this->get_logger(), "✓ BGK prediction completed");
        
        // Step 8: Publish final point cloud
        publishCloud();
        // RCLCPP_INFO(this->get_logger(), "✓ Cloud published");
        
        // Step 9: Convert obstacles to 2D laser scan for navigatio -> not used as just using a gradiant based map
        // publishLaserScan();
        // RCLCPP_INFO(this->get_logger(), "✓ Laser scan published");
        
        // Step 10: Clean up for next iteration
        resetParameters();
        // RCLCPP_INFO(this->get_logger(), "=== cloudHandler completed ===\n");
    }

    void extractRawCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        // RCLCPP_INFO(this->get_logger(), "=== Starting extractRawCloud ===");
        
        // Check point cloud organization
        // RCLCPP_INFO(this->get_logger(), "Point cloud info:");
        // RCLCPP_INFO(this->get_logger(), "  Width: %d, Height: %d", laserCloudMsg->width, laserCloudMsg->height);
        // RCLCPP_INFO(this->get_logger(), "  Total points: %d", laserCloudMsg->width * laserCloudMsg->height);
        // RCLCPP_INFO(this->get_logger(), "  Is organized: %s", (laserCloudMsg->height > 1) ? "YES" : "NO");
        // RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", laserCloudMsg->header.frame_id.c_str());
        
        // RCLCPP_INFO(this->get_logger(), "Converting ROS message to PCL...");
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // RCLCPP_INFO(this->get_logger(), "PCL cloud size: %zu", laserCloudIn->points.size());
        
        // Initialize matrices
        // RCLCPP_INFO(this->get_logger(), "Initializing matrices...");
        obstacleMatrix.setTo(-1);  // -1 = invalid
        rangeMatrix.setTo(-1);     // -1 = invalid
        
        int valid_points = 0;
        int nan_points = 0;
        int out_of_bounds = 0;

        // DEBUG
        int num_printed = 0;
        int num_too_close = 0;

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

                // remove point if it's within the threshold range
                if (pointDepth2 < sensorMinRangeLimit)
                {
                    num_too_close++;
                    continue;
                }
                // otherwise, store it into the range matrix and reset obstacle status
                rangeMatrix.at<float>(i, j) = pointDepth2;
                obstacleMatrix.at<int>(i, j) = 0;
                valid_points++;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Out of bounds %d", out_of_bounds);
        // RCLCPP_INFO(this->get_logger(), "=== extractRawCloud Results ===");
        // RCLCPP_INFO(this->get_logger(), "Valid points processed: %d", valid_points);
        // RCLCPP_INFO(this->get_logger(), "Points that are too close: %d", num_too_close);
        // RCLCPP_INFO(this->get_logger(), "Expected total: %d (N_SCAN=%d x Horizon_SCAN=%d)", 
        //             N_SCAN * Horizon_SCAN, N_SCAN, Horizon_SCAN);

        // minDistFilter();
    }

    bool transformCloud(){
        try{
            // Get the actual frame_id from the incoming message
            std::string source_frame = "velodyne";  // or use laserCloudMsg->header.frame_id
            
            // Look up transform from sensor frame to map
            auto transform = tf_buffer_->lookupTransform("map", source_frame, tf2::TimePointZero);
            
            // Update robot position (you'll need base_link to map for this)
            auto robot_transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            robotPoint.x = robot_transform.transform.translation.x;
            robotPoint.y = robot_transform.transform.translation.y;
            robotPoint.z = robot_transform.transform.translation.z;
    
            // Transform point cloud from sensor frame to map frame
            sensor_msgs::msg::PointCloud2 cloud_in_msg, cloud_out_msg;
            pcl::toROSMsg(*laserCloudIn, cloud_in_msg);
            cloud_in_msg.header.frame_id = source_frame;  // Use actual sensor frame
            cloud_in_msg.header.stamp = this->get_clock()->now();
    
            // Apply transformation
            tf2::doTransform(cloud_in_msg, cloud_out_msg, transform);
            pcl::fromROSMsg(cloud_out_msg, *laserCloudIn);
            
            // RCLCPP_INFO(this->get_logger(), "Transformed from %s to map frame", source_frame.c_str());
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

    // void applyFilter(){

    //     // minDistFilter();
    //     // slopeFilter();
    // }

    // void slopeFilter(){
        
    //     for (int i = 0; i < scanNumSlopeFilter; ++i){
    //         for (int j = 0; j < Horizon_SCAN; ++j){
    //             // Point that has been verified by other filters
    //             if (obstacleMatrix.at<int>(i, j) == 1)
    //                 continue;
    //             // point without range value cannot be verified
    //             if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i+1, j) == -1)
    //                 continue;
    //             // point is too far away, skip comparison since it can be inaccurate
    //             if (rangeMatrix.at<float>(i, j) > sensorMaxRangeLimit)
    //                 continue;
    //             // Two range filters here:
    //             // 1. if a point's range is larger than scanNumSlopeFilter th ring point's range
    //             // 2. if a point's range is larger than the upper point's range
    //             // then this point is very likely on obstacle. i.e. a point under the car or on a pole
    //             // if (  (rangeMatrix.at<float>(scanNumSlopeFilter, j) != -1 && rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(scanNumSlopeFilter, j))
    //             //     || (rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(i+1, j)) ){
    //             //     obstacleMatrix.at<int>(i, j) = 1;
    //             //     continue;
    //             // }
    //             // Calculate slope angle
    //             float diffX = laserCloudMatrix[i+1][j].x - laserCloudMatrix[i][j].x;
    //             float diffY = laserCloudMatrix[i+1][j].y - laserCloudMatrix[i][j].y;
    //             float diffZ = laserCloudMatrix[i+1][j].z - laserCloudMatrix[i][j].z;
    //             float angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
    //             // Slope angle is larger than threashold, mark as obstacle point TODO: This may not be good cause slope is lower at higher elevations
    //             // if (angle < -filterAngleLimit || angle > filterAngleLimit){
    //             //     obstacleMatrix.at<int>(i, j) = 1;
    //             //     continue;
                
    //         }
    //     }
    // }

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

        // Publish laserCloudOut for visualization (before downsample and BGK prediction)
        if (pubCloudVisualHiRes->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            // ADD THIS DEBUG PRINT:
            // RCLCPP_INFO(this->get_logger(), "laserCloudTemp contains %d points", 
            // laserCloudTemp.width * laserCloudTemp.height);

            laserCloudTemp.header.stamp = this->get_clock()->now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualHiRes->publish(laserCloudTemp);
        }
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
            // // obstacle point (decided by curb or slope filter)
            // if (laserCloudOut->points[i].intensity == 100)
            //     obstFlag[idx][idy] = true;
            // save min and max height of a grid
            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
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
                thisPoint.z = maxHeight[i][j];

                // if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit){
                //     obstFlag[i][j] = true;
                //     thisPoint.intensity = 100; // obstacle
                //     laserCloudTemp->push_back(thisPoint);
                // }else{
                thisPoint.intensity = 0; // free
                laserCloudTemp->push_back(thisPoint);
                //}
            }
        }

        *laserCloudOut = *laserCloudTemp;

        // Publish laserCloudOut for visualization (after downsample but beforeBGK prediction)
        if (pubCloudVisualLowRes->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = this->get_clock()->now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualLowRes->publish(laserCloudTemp);
        }
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
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
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
        laserCloudTemp.header.frame_id = "map";
        pubCloud->publish(laserCloudTemp);
    }

    void publishLaserScan(){

        updateLaserScan();

        laserScan.header.stamp = this->get_clock()->now();
        pubLaserScan->publish(laserScan);
        // initialize laser scan for new scan
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
    }

    void updateLaserScan(){

        try{
            auto transform = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);

            laserCloudObstacles->header.frame_id = "map";
            laserCloudObstacles->header.stamp = 0;
            // transform obstacle cloud back to "base_link" frame
            pcl::PointCloud<PointType> laserCloudTemp;
            sensor_msgs::msg::PointCloud2 cloud_in_msg, cloud_out_msg;
            pcl::toROSMsg(*laserCloudObstacles, cloud_in_msg);
            tf2::doTransform(cloud_in_msg, cloud_out_msg, transform);
            pcl::fromROSMsg(cloud_out_msg, laserCloudTemp);

            //convert point to scan
            int cloudSize = laserCloudTemp.points.size();
            for (int i = 0; i < cloudSize; ++i){
                PointType *point = &laserCloudTemp.points[i];
                float x = point->x;
                float y = point->y;
                float range = std::sqrt(x*x + y*y);
                float angle = std::atan2(y, x);
                int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
                if (index >= 0 && index < laserScan.ranges.size())
                    laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
            } 
        }
        catch (tf2::TransformException& ex){
            RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
            return;
        }
    }

    void pointcloud2laserscanInitialization(){

        laserScan.header.frame_id = "base_link"; // assume laser has the same frame as the robot

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
