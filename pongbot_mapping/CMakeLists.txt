cmake_minimum_required(VERSION 3.0.2)
project(pongbot_mapping)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  filters
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp filters grid_map_core grid_map_ros grid_map_filters grid_map_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(traversability_node src/nodes/traversability_node.cpp)
target_link_libraries(traversability_node ${catkin_LIBRARIES})

install(TARGETS traversability_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})