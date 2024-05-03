cmake_minimum_required(VERSION 3.5)
project(lidar_filter)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# find_package(stereo_msgs REQUIRED)

# find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  PCL
  # OpenCV
)

include_directories(
  include ${ament_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${cv_bridge_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

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

add_executable(NAV_lidar_filter_node src/lidar_filter.cpp)
ament_target_dependencies(NAV_lidar_filter_node rclcpp std_msgs sensor_msgs PCL pcl_conversions)

target_link_libraries(NAV_lidar_filter_node
  ${ament_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  NAV_lidar_filter_node
  DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(include)

ament_package()
