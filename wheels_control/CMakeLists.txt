cmake_minimum_required(VERSION 3.8)
project(wheels_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # needed for inline object variables
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_FLAGS_DEBUG "-g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)



include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${Eigen3_DIR}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.xml")



# install params directory
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)


# # install scripts directory
# install(DIRECTORY
#   scripts
#   DESTINATION share/${PROJECT_NAME}/
# )

set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/localization/wheels_odometry)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/fake_cs_gamepad.py
  scripts/fake_cs_shutdown.py
  scripts/backup_gamepad_pub.py
  DESTINATION lib/${PROJECT_NAME}
)




# add cpp executables
add_executable(NAV_gamepad_interface src/Gamepad_interface_node.cpp)
ament_target_dependencies(NAV_gamepad_interface rclcpp custom_msg geometry_msgs sensor_msgs) 

add_executable(NAV_cmd_vel_manager src/Cmd_vel_manager_node.cpp)
ament_target_dependencies(NAV_cmd_vel_manager rclcpp custom_msg geometry_msgs sensor_msgs) 

add_executable(NAV_odometry_node src/odometry_node.cpp)
ament_target_dependencies(NAV_odometry_node rclcpp nav_msgs tf2 tf2_geometry_msgs Eigen3 custom_msg)


add_executable(NAV_displacement_cmds 
  src/Displacement_cmds_node.cpp
  src/kinematic_models/normal_kinematic_model.cpp
  src/kinematic_models/lateral_kinematic_model.cpp
)

ament_target_dependencies(NAV_displacement_cmds rclcpp custom_msg geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs) 

add_executable(NAV_motor_cmds src/Motors_cmds_node_lifecycle.cpp src/motors.cpp)
ament_target_dependencies(NAV_motor_cmds rclcpp_lifecycle rclcpp custom_msg std_msgs std_srvs)


install(
  DIRECTORY include/
  DESTINATION include
)


# add epos libraries

target_link_libraries(NAV_motor_cmds /usr/local/lib/libftd2xx.so )
target_link_libraries(NAV_motor_cmds /usr/local/lib/libEposCmd.so )
target_link_libraries(NAV_odometry_node Eigen3::Eigen)
target_link_libraries(NAV_odometry_node 
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)


install(TARGETS
  NAV_gamepad_interface
  NAV_cmd_vel_manager
  NAV_displacement_cmds
  NAV_motor_cmds
  NAV_odometry_node
  
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
  
ament_package()
