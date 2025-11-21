cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config PCD rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
