cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_opti)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(multiview_aruco_node src/multi_aruco_node_opti.cpp)
ament_target_dependencies(multiview_aruco_node
  cv_bridge
  geometry_msgs
  message_filters
  rclcpp
  ros2_aruco_interfaces
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(multiview_aruco_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(multiview_aruco_node Eigen3::Eigen ${OpenCV_LIBS})

add_executable(pose_estimation_node src/pose_estimation_node_opti.cpp)
ament_target_dependencies(pose_estimation_node
  geometry_msgs
  nav_msgs
  rclcpp
  ros2_aruco_interfaces
  tf2
  tf2_ros
)
target_link_libraries(pose_estimation_node Eigen3::Eigen)

add_executable(plot_arucos_node src/plot_arucos_node_opti.cpp)
ament_target_dependencies(plot_arucos_node
  rclcpp
  ros2_aruco_interfaces
  visualization_msgs
)

install(TARGETS
  multiview_aruco_node
  pose_estimation_node
  plot_arucos_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
