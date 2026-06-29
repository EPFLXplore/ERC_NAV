cmake_minimum_required(VERSION 3.8)
project(local_nav_ekf)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)




# include headers
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(nav_ekf_node src/NavEKF.cpp)
ament_target_dependencies(nav_ekf_node
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    std_srvs
    custom_msg
)
target_link_libraries(nav_ekf_node Eigen3::Eigen)

add_executable(nav_ekf_3d_node src/NavEKF3D.cpp)
ament_target_dependencies(nav_ekf_3d_node
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    std_srvs
    std_msgs
    custom_msg
)
target_link_libraries(nav_ekf_3d_node Eigen3::Eigen)

# Install
install(TARGETS nav_ekf_node nav_ekf_3d_node
    DESTINATION lib/${PROJECT_NAME}
)

# (Optional) install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py" PATTERN "*.xml"
)

ament_package()
