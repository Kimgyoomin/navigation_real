cmake_minimum_required(VERSION 3.0.2)
project(pongbot_traversability)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  grid_map_msgs
  grid_map_ros
  grid_map_core
  pongbot_heightmap_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    grid_map_msgs
    grid_map_ros
    grid_map_core
    pongbot_heightmap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(support_patch_debug_node
  src/support_patch_debug_node.cpp
)

target_compile_features(support_patch_debug_node PUBLIC cxx_std_17)

add_dependencies(support_patch_debug_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(support_patch_debug_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)