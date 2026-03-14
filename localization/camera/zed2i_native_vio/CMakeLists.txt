cmake_minimum_required(VERSION 3.8)
project(zed2i_native_vio)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/native_vio_watcher
  scripts/run_zed2i_native_vio.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
