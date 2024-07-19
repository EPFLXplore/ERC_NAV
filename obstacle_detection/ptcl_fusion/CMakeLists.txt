cmake_minimum_required(VERSION 3.5)
project(ptcl_fusion)

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





find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(ptcl_concat src/ptcl_concat.cpp)


include_directories(
  include
  ${PCL_CONVERSIONS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  
)

target_link_libraries(ptcl_concat
  ${PCL_CONVERSIONS_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ptcl_concat
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)


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

install(TARGETS
  ptcl_concat
  DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
