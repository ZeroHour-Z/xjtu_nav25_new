cmake_minimum_required(VERSION 3.8)
project(nav2_client_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(navigate_to_pose_node src/navigate_to_pose_node.cpp)
ament_target_dependencies(navigate_to_pose_node rclcpp rclcpp_action nav2_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node rclcpp rclcpp_action nav2_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros nav_msgs)

install(TARGETS navigate_to_pose_node path_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package() 