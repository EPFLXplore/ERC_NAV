cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_with_lidar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  segmentation
  sample_consensus
  filters
)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Add other C++ dependencies here

# C++ executables
add_executable(lidar_phi_filter_node ros2_aruco_with_lidar/lidar_phi_filter_node.cpp)
add_executable(detect_cube ros2_aruco_with_lidar/detect_cube.cpp)
add_executable(aruco_lidar_recovery_manager ros2_aruco_with_lidar/aruco_lidar_recovery_manager.cpp)

ament_target_dependencies(lidar_phi_filter_node rclcpp sensor_msgs ros2_aruco_interfaces visualization_msgs pcl_conversions geometry_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(detect_cube rclcpp sensor_msgs std_msgs ros2_aruco_interfaces pcl_conversions visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(aruco_lidar_recovery_manager
  rclcpp std_msgs nav_msgs geometry_msgs ros2_aruco_interfaces
  tf2 tf2_ros tf2_geometry_msgs)

target_link_libraries(detect_cube
  ${PCL_SEGMENTATION_LIBRARIES}
  ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)

target_compile_features(lidar_phi_filter_node PUBLIC c_std_99 cxx_std_17)

# Install C++ executables
install(TARGETS
  lidar_phi_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  detect_cube
  aruco_lidar_recovery_manager
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  # Add your Python scripts here
  # scripts/my_python_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
