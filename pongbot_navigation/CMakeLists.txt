cmake_minimum_required(VERSION 3.8)
project(pongbot_navigation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rubi_heightmap_wavefront_planner REQUIRED)

add_library(pongbot_navigation_heightmap_wavefront_plugin SHARED
  src/heightmap_wavefront_nav2_planner.cpp
  src/fastdem_snapshot_adapter.cpp
)
target_compile_features(pongbot_navigation_heightmap_wavefront_plugin PUBLIC cxx_std_17)
target_include_directories(pongbot_navigation_heightmap_wavefront_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pongbot_navigation_heightmap_wavefront_plugin
  geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp
  rclcpp_lifecycle sensor_msgs tf2 tf2_geometry_msgs tf2_ros
)
target_link_libraries(pongbot_navigation_heightmap_wavefront_plugin
  rubi_heightmap_wavefront_planner::rubi_heightmap_wavefront_planner_core)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS pongbot_navigation_heightmap_wavefront_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch config maps rviz_cfg behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_fastdem_snapshot_adapter test/test_fastdem_snapshot_adapter.cpp)
  target_link_libraries(test_fastdem_snapshot_adapter
    pongbot_navigation_heightmap_wavefront_plugin)

  ament_add_gtest(test_heightmap_wavefront_plugin test/test_heightmap_wavefront_plugin.cpp)
  target_link_libraries(test_heightmap_wavefront_plugin
    pongbot_navigation_heightmap_wavefront_plugin)

  ament_add_pytest_test(
    test_heightmap_nav2_config_contract test/test_heightmap_nav2_config_contract.py)
  add_launch_test(test/test_heightmap_nav2_launch.py TIMEOUT 120)
  set_tests_properties(test_test_heightmap_nav2_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=203")
endif()

ament_package()
