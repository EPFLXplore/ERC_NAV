cmake_minimum_required(VERSION 3.10)
project(realsense_pub)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# RealSense2 SDK
find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp)
find_library(REALSENSE2_LIBRARY NAMES realsense2)

# Executable
add_executable(realsense_pub_node src/realsense_pub_node.cpp)
target_include_directories(realsense_pub_node PUBLIC
  ${REALSENSE2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(realsense_pub_node
  ${REALSENSE2_LIBRARY}
  ${OpenCV_LIBS}
)

ament_target_dependencies(realsense_pub_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install target
install(TARGETS realsense_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
