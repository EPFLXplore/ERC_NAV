cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_opti)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-g")
endif()

# ROS / ament
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(nav_msgs       REQUIRED)
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros        REQUIRED)


add_executable(aruco_pose_detector_node
  src/aruco_pose_estimation.cpp
)

# Link to ROS
ament_target_dependencies(aruco_pose_detector_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  tf2_ros
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
)

# Link Eigen and each OpenCV module explicitly
target_link_libraries(aruco_pose_detector_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
)


install(TARGETS
  aruco_pose_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY config launch
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
