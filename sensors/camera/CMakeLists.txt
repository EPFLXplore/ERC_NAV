cmake_minimum_required(VERSION 3.5)
project(front_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
# find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(stereo_msgs REQUIRED)

find_package(std_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)
# find_package(depth_image_proc REQUIRED)

set(dependencies
  camera_info_manager
  depthai_bridge
  depthai
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  pcl_conversions
  PCL
  OpenCV
)

include_directories(
  include ${ament_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)


# include_directories(
#   ../depth_image_proc/include
# )




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.xml")


# add_executable(front_camera_filter src/front_camera_filter.cpp)
# ament_target_dependencies(front_camera_filter rclcpp sensor_msgs pcl_conversions PCL cv_bridge OpenCV)

add_executable(front_camera_publisher src/stereo_publisher.cpp)
ament_target_dependencies(front_camera_publisher rclcpp std_msgs sensor_msgs stereo_msgs camera_info_manager depthai depthai_bridge)

install(TARGETS
  front_camera_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
