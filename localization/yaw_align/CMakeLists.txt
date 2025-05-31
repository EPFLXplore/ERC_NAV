cmake_minimum_required(VERSION 3.5)
project(yaw_align)

# Default to C++17 (if not already set)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ament & ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# OpenCV (we need at least core, imgcodecs, aruco, imgproc)
find_package(OpenCV REQUIRED)

# If you do NOT use any cv::imshow() / HighGUI, you could restrict to:
#   find_package(OpenCV COMPONENTS core imgcodecs aruco imgproc REQUIRED)

add_executable(yaw_align src/yaw_align.cpp)

# Link against rclcpp, sensor_msgs, and OpenCV
ament_target_dependencies(yaw_align
  rclcpp
  sensor_msgs
  OpenCV
)

# Guarantee that OpenCV libraries are linked
target_link_libraries(yaw_align ${OpenCV_LIBS})

install(TARGETS
  yaw_align
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
