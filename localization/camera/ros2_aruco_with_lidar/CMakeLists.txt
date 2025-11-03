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
# Add other C++ dependencies here

# C++ executables
add_executable(lidar_phi_filter_node ros2_aruco_with_lidar/lidar_phi_filter_node.cpp)
ament_target_dependencies(lidar_phi_filter_node rclcpp sensor_msgs ros2_aruco_interfaces)
target_compile_features(lidar_phi_filter_node PUBLIC c_std_99 cxx_std_17)

# Install C++ executables
install(TARGETS
  lidar_phi_filter_node
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