cmake_minimum_required(VERSION 3.8)
project(gradient_costmap)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(elevation_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)   

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  cv_bridge
  image_transport
  #pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
#   elevation_msgs
  interactive_markers
  visualization_msgs
  pluginlib
  nav2_core
)

################################## PLUGIN LIBRARY #########################
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_library(gradient_layer SHARED
  src/gradient_layer_plugin.cpp
)

ament_target_dependencies(gradient_layer
  rclcpp
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

target_include_directories(gradient_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS gradient_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES gradient_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer_plugin.xml)

################################## END PLUGIN LIBRARY #########################

# Executables
add_executable(traversability_filter src/traversability_filter.cpp)
ament_target_dependencies(traversability_filter ${dependencies})
target_link_libraries(traversability_filter ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(traversability_map src/traversability_map.cpp)
ament_target_dependencies(traversability_map ${dependencies})
target_link_libraries(traversability_map ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Install executables
install(TARGETS
  traversability_filter
  traversability_map
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()