cmake_minimum_required(VERSION 3.5)
project(rviz_click_obstacles)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/click_obstacles_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core Qt5::Widgets
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  std_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 