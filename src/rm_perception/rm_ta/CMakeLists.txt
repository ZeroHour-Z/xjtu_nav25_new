cmake_minimum_required(VERSION 3.5)
project(rm_ta)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(traversability_costmap_node src/traversability_costmap_node.cpp)

ament_target_dependencies(traversability_costmap_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(traversability_costmap_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(traversability_costmap_node
  ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

install(TARGETS traversability_costmap_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 