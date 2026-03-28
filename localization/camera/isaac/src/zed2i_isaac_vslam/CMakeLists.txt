cmake_minimum_required(VERSION 3.8)
project(zed2i_isaac_vslam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config 3
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vio_watcher
  DESTINATION lib/${PROJECT_NAME}
) 31

ament_package()
