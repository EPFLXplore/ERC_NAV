cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_lidar_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

add_executable(lidar_phi_filter_node src/lidar_phi_filter_node.cpp)
ament_target_dependencies(lidar_phi_filter_node rclcpp sensor_msgs ros2_aruco_interfaces)
target_compile_features(lidar_phi_filter_node PUBLIC c_std_99 cxx_std_17)

add_executable(detect_cube src/detect_cube.cpp)
ament_target_dependencies(detect_cube rclcpp sensor_msgs pcl_conversions)
target_link_libraries(detect_cube ${PCL_LIBRARIES})
target_include_directories(detect_cube PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_features(detect_cube PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  lidar_phi_filter_node
  detect_cube
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


