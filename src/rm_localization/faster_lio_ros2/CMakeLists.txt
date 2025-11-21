cmake_minimum_required(VERSION 3.8)
project(faster_lio_ros2)

set(DEFAULT_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-pthread -fexceptions)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# add_subdirectory(thirdparty/livox_ros_driver2)
# add_subdirectory(app)

find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(engine_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

set(dependencies
  rclcpp
  # rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

rosidl_generate_interfaces(${PROJECT_NAME}
            "msg/Pose6D.msg"
            DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        include
)
# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

# add_subdirectory(src)
add_library(laser_mapping SHARED
        src/laser_mapping.cc
        src/pointcloud_preprocess.cc
        src/options.cc
        src/utils.cc
        )
# target_include_directories(laser_mapping PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )
target_link_libraries(laser_mapping
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        
        )
target_include_directories(laser_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
ament_target_dependencies(laser_mapping ${dependencies})
rosidl_target_interfaces(laser_mapping
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_subdirectory(app)
add_executable(run_mapping_online app/run_mapping_online.cc)
target_link_libraries(run_mapping_online
        laser_mapping 
        )
ament_target_dependencies(run_mapping_online ${dependencies})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
# install(TARGETS run_mapping_online laser_mapping
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(
#         TARGETS laser_mapping # 告诉ros2有这么个目标（可执行文件或者库）
#         EXPORT laser_mapping
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include
# )
# install(
#         TARGETS run_mapping_online # 告诉ros2有这么个目标（可执行文件或者库）
#         EXPORT run_mapping_online
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include
# )
install(
    TARGETS laser_mapping run_mapping_online
    DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
