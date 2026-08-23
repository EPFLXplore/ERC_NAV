cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_cpp)

# This package currently gets Release only from a stale CMakeCache entry left by
# a past --cmake-args invocation. Pin it in-source so a clean build cannot
# silently drop -DNDEBUG.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# NOTE: the -O2 below lands after Release's -O3 on the command line, and GCC
# honours the last -O flag, so this package actually builds at -O2.
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
find_package(g2o REQUIRED)
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
target_link_libraries(pose_estimator_lidar_node
  Eigen3::Eigen
  g2o::core
  g2o::stuff
  g2o::solver_dense)

install(TARGETS multiview_aruco_node pose_estimator_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
