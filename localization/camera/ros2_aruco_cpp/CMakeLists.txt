cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---------- multiview_aruco_node ----------
add_executable(multiview_aruco_node src/multiview_aruco_node.cpp)
ament_target_dependencies(multiview_aruco_node
  rclcpp std_msgs sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  ros2_aruco_interfaces message_filters)
target_link_libraries(multiview_aruco_node ${OpenCV_LIBS} Eigen3::Eigen nlohmann_json::nlohmann_json)


# ---------- pose_estimator_lidar_node ----------
add_executable(pose_estimator_lidar_node src/pose_estimator_lidar_node.cpp)
ament_target_dependencies(pose_estimator_lidar_node
  rclcpp geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  ros2_aruco_interfaces)
target_link_libraries(pose_estimator_lidar_node Eigen3::Eigen)

find_library(ECOS_LIB ecos PATHS /usr/local/lib)
if(ECOS_LIB)
  message(STATUS "ECOS library found: ${ECOS_LIB}")
  target_link_libraries(pose_estimator_lidar_node ${ECOS_LIB} m)
  target_compile_definitions(pose_estimator_lidar_node PRIVATE HAS_ECOS)
  target_include_directories(pose_estimator_lidar_node PRIVATE /usr/local/include)
else()
  message(WARNING "ECOS library NOT found – n>=3 SOCP solver will be disabled at runtime.")
endif()

install(TARGETS multiview_aruco_node pose_estimator_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
