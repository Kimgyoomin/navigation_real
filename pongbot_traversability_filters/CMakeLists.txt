cmake_minimum_required(VERSION 3.0.2)
project(pongbot_traversability_filters)

find_package(catkin REQUIRED COMPONENTS
  filters
  grid_map_core
  grid_map_filters
  grid_map_msgs
  grid_map_ros
  grid_map_cv
  pluginlib
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pongbot_traversability_filters
  CATKIN_DEPENDS roscpp pluginlib filters grid_map_core grid_map_ros grid_map_filters grid_map_msgs grid_map_cv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pongbot_traversability_filters
  src/drop_filter.cpp
  src/variance_gate_filter.cpp
  # src/fuse_trav_filter.cpp
  src/morph_drop_filter.cpp
)
target_link_libraries(pongbot_traversability_filters ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS pongbot_traversability_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})