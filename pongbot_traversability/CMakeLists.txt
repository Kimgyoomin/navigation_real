cmake_minimum_required(VERSION 3.0.2)
project(pongbot_traversability)

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
  INCLUDE_DIRS include
  LIBRARIES support_patch_evaluator
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

add_library(support_patch_evaluator
  src/support_patch_evaluator.cpp
)

target_compile_features(support_patch_evaluator PUBLIC cxx_std_14)

add_dependencies(support_patch_evaluator
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(support_patch_evaluator
  ${catkin_LIBRARIES}
)

add_executable(support_patch_debug_node
  src/support_patch_debug_node.cpp
)

target_compile_features(support_patch_debug_node PUBLIC cxx_std_14)

add_dependencies(support_patch_debug_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(support_patch_debug_node
  support_patch_evaluator
  ${catkin_LIBRARIES}
)

install(TARGETS support_patch_evaluator support_patch_debug_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)