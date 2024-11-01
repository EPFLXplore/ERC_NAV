cmake_minimum_required(VERSION 3.8)
project(imu_pub_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the path to the custom find module
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Manually add include directories if LibSerial is installed in /usr/include/libserial
include_directories(/usr/include)

# Add executable
add_executable(imu_pub_raw src/imu_pub.cc)

# Link dependencies to the executable
target_link_libraries(imu_pub_raw /usr/lib/x86_64-linux-gnu/libserial.so)
ament_target_dependencies(imu_pub_raw rclcpp sensor_msgs)

# Install the executable
install(TARGETS imu_pub_raw
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
