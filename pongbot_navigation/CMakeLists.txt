cmake_minimum_required(VERSION 3.8)
project(pongbot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz_cfg behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
