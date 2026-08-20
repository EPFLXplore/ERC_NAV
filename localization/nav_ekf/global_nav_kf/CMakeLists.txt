cmake_minimum_required(VERSION 3.8)
project(global_nav_kf)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# include headers
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(global_nav_kf_2d_node src/GlobalNavKF2D.cpp)
ament_target_dependencies(global_nav_kf_2d_node
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)
target_link_libraries(global_nav_kf_2d_node Eigen3::Eigen)

# Install
install(TARGETS global_nav_kf_2d_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
