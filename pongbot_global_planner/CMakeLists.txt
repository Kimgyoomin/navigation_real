cmake_minimum_required(VERSION 3.0.2)
project(pongbot_global_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  pluginlib
  costmap_2d
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  LIBRARIES pongbot_global_planner
  CATKIN_DEPENDS roscpp nav_core pluginlib costmap_2d nav_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pongbot_global_planner
  src/astar_planner.cpp
  src/astar_ray_planner.cpp
  src/astar_sparse_planner.cpp
)
target_link_libraries(pongbot_global_planner ${catkin_LIBRARIES})
add_dependencies(pongbot_global_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS pongbot_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
