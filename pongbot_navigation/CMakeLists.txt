cmake_minimum_required(VERSION 3.8)
project(pongbot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(sector_filter_core STATIC src/sector_filter.cpp)
target_compile_features(sector_filter_core PUBLIC cxx_std_17)
target_include_directories(sector_filter_core PUBLIC include)
ament_target_dependencies(sector_filter_core sensor_msgs)

add_executable(costmap_sector_filter src/costmap_sector_filter.cpp)
target_link_libraries(costmap_sector_filter sector_filter_core)
ament_target_dependencies(costmap_sector_filter rclcpp rcl_interfaces sensor_msgs)
install(TARGETS costmap_sector_filter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_sector_filter test/test_sector_filter.cpp)
  target_link_libraries(test_sector_filter sector_filter_core)
  ament_target_dependencies(test_sector_filter sensor_msgs)
  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)
endif()

install(DIRECTORY launch config maps rviz_cfg behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
