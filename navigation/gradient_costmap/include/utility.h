#ifndef _UTILITY_TM_H_
#define _UTILITY_TM_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

// #include <nav2_core/global_planner.hpp>
// #include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <array> // c++11
#include <thread> // c++11
#include <mutex> // c++11

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include "planner/kdtree.h"
// #include "planner/cubic_spline_interpolator.h" // Commented out due to ROS1 compatibility issues

// #include "elevation_msgs/msg/occupancy_elevation.hpp"

using namespace std;

typedef pcl::PointXYZI  PointType;

// cloudHandler rate
extern const int cloudHandlerRate = 250; // time in ms to wait before processing the next pointcloud

// VLP-16
extern const int N_SCAN = 32;   // vertical scan
extern const int Horizon_SCAN = 2048;        // horizontal scan lines

// Map Params
const int globalMapDim = 100;   // global map dimension in meters (represented as a square)
extern const float mapResolution = 0.1; // map resolution
extern const float mapCubeLength = 0.2; // the length of a sub-map (meters)
extern const int mapCubeArrayLength = mapCubeLength / mapResolution; // the grid dimension of a sub-map (mapCubeLength / mapResolution)
extern const int mapArrayLength = globalMapDim / mapCubeLength; // the sub-map dimension of 
extern const int rootCubeIndex = mapArrayLength / 2; // by default, robot is at the center of global map at the beginning

// Cost Params
constexpr float slopeCoeff = 1.0;           // how much the slope contributes to cost
constexpr float roughnessCoeff = 0;       // how much roughness contributes to cost

// Static assertion to ensure coefficients sum to 1.0
static_assert(
    (slopeCoeff + roughnessCoeff) >= 0.999 && (slopeCoeff + roughnessCoeff) <= 1.001,
    "slopeCoeff and roughnessCoeff must sum to 1.0"
);

// Filter Ring Params -> Not used
// extern const int scanNumCurbFilter = 8;             // Not needed    
// extern const int scanNumSlopeFilter = 20;           // the number of lower scan for slope analysis
// extern const int scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);

// Filter Threshold Params
extern const float maxObstacleHeight = 2.0;     // maximum obstacle height of 2 meters 
extern const float sensorMaxRangeLimit = 10; // only keep points with in a radius of x meters   
extern const float sensorMinRangeLimit = 0.6; // remove points within a radius of 0.6 meters of Lidar
extern const float filterAngleLimit = 35; // slope angle threshold     
extern const float filterMaxRoughness = 0.1;      
extern const int filterHeightMapArrayLength = sensorMaxRangeLimit*2 / mapResolution;    // size of the local height mat grid

// BGK Prediction Params
extern const bool predictionEnableFlag = false;
extern const float predictionKernalSize = 0.2; // predict elevation within x meters

// Occupancy Params -> not used
// extern const float p_occupied_when_laser = 0.9;
// extern const float p_occupied_when_no_laser = 0.2;
// extern const float large_log_odds = 100;
// extern const float max_log_odds_for_belief = 20;

// 2D Map Publish Params
extern const int localMapLength = 24; // 12m radius covers sensorMaxRangeLimit (10m) + margin; Nav2 local costmap is only 6x6m
extern const int localMapArrayLength = localMapLength / mapResolution;

// Visualization Params for visualizing the elevation pointcloud
extern const float visualizationRadius = 10;
extern const float visualizationFrequency = 2; // n, skip n scans then publish, n=0, visualize at each scan

// Robot Params
extern const float robotRadius = 0.4;

// Traversability Params
extern const int traversabilityObserveTimeTh = 10;

struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

/*
    This struct is used to send map from mapping package to prm package
    */
struct grid_t{
    int mapID;
    int cubeX;
    int cubeY;
    int gridX;
    int gridY;
    int gridIndex;
};

/*
    Cell Definition:
    a cell is a member of a grid in a sub-map
    a grid can have several cells in it. 
    a cell represent one height information
    */

struct mapCell_t{

    PointType *xyz; // it's a pointer to the corresponding point in the point cloud of submap

    grid_t grid;

    float log_odds;

    int observeTimes;
    
    float occupancy, occupancyVar;
    float elevation, elevationVar;

    bool needsUpdate;

    mapCell_t(){

        log_odds = 0.5;
        observeTimes = 0;

        elevation = -FLT_MAX;
        elevationVar = 1e3;

        occupancy = 0; // initialized as unkown
        occupancyVar = 1e3;
        needsUpdate = true;
    }

    void updatePoint(){
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }
    void updateElevation(float elevIn, float varIn){
        elevation = elevIn;
        elevationVar = varIn;
        updatePoint();
    }
    void updateOccupancy(float occupIn){
        occupancy = occupIn;
        updatePoint();
    }
};


/*
    Sub-map Definition:
    childMap_t is a small square. We call it "cellArray". 
    It composes the whole map
    */
struct childMap_t{

    vector<vector<mapCell_t*> > cellArray;
    int subInd; //sub-map's index in 1d mapArray
    int indX; // sub-map's x index in 2d array mapArrayInd
    int indY; // sub-map's y index in 2d array mapArrayInd
    float originX; // sub-map's x root coordinate
    float originY; // sub-map's y root coordinate
    pcl::PointCloud<PointType> cloud;

    childMap_t(int id, int indx, int indy){

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;

        // allocate and initialize each cell
        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;
        // allocate point cloud for visualization
        cloud.points.resize(mapCubeArrayLength*mapCubeArrayLength);

        // initialize cell pointer to cloud point
        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j*mapCubeArrayLength];

        // initialize each point in the point cloud, also each cell
        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                
                // point cloud initialization
                int index = i + j * mapCubeArrayLength;
                cloud.points[index].x = originX + i * mapResolution;
                cloud.points[index].y = originY + j * mapResolution;
                cloud.points[index].z = std::numeric_limits<float>::quiet_NaN();
                cloud.points[index].intensity = cellArray[i][j]->occupancy;

                // cell position in the array of submap
                cellArray[i][j]->grid.mapID = subInd;
                cellArray[i][j]->grid.cubeX = indX;
                cellArray[i][j]->grid.cubeY = indy;
                cellArray[i][j]->grid.gridX = i;
                cellArray[i][j]->grid.gridY = j;
                cellArray[i][j]->grid.gridIndex = index;
            }
        }
    }
};


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Some Functions    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

float pointDistance(PointType p1, PointType p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
