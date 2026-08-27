cmake_minimum_required(VERSION 3.8)
project(rubi_heightmap_step_wavefront_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/heightmap_snapshot.cpp
  src/step_evaluator.cpp
  src/graph/spatial_index_2d.cpp
  src/graph/wavefront_graph_builder.cpp
  src/search/astar_search.cpp
  src/planning/step_wavefront_planner.cpp
  src/planning/planning_fsm.cpp
  src/path_revalidation.cpp
  src/plan_lifecycle.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_visualization STATIC src/planner_visualization.cpp)
target_compile_features(${PROJECT_NAME}_visualization PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_visualization ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_visualization
  builtin_interfaces geometry_msgs std_msgs visualization_msgs)

add_library(${PROJECT_NAME}_ros_adapter STATIC src/ros/pointcloud2_heightmap_adapter.cpp)
target_compile_features(${PROJECT_NAME}_ros_adapter PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ros_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_ros_adapter ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros_adapter sensor_msgs)

add_executable(step_wavefront_planner_node src/planner_node.cpp)
target_compile_features(step_wavefront_planner_node PUBLIC cxx_std_17)
target_link_libraries(step_wavefront_planner_node
  ${PROJECT_NAME}_core ${PROJECT_NAME}_visualization ${PROJECT_NAME}_ros_adapter)
ament_target_dependencies(step_wavefront_planner_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs
  tf2_ros visualization_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core step_wavefront_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_executable(core_smoke test/core_smoke.cpp)
  target_link_libraries(core_smoke ${PROJECT_NAME}_core)
  add_test(NAME core_smoke COMMAND core_smoke)

  foreach(test_name
      test_heightmap_snapshot
      test_step_evaluator
      test_spatial_index_2d
      test_astar_search
      test_clearance_sweep
      test_step_wavefront_planner
      test_path_revalidation
      test_plan_lifecycle)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
  endforeach()

  ament_add_gtest(test_planning_fsm test/test_planning_fsm.cpp)
  target_link_libraries(test_planning_fsm ${PROJECT_NAME}_core)

  ament_add_gtest(test_pointcloud2_heightmap_adapter
    test/test_pointcloud2_heightmap_adapter.cpp)
  target_link_libraries(test_pointcloud2_heightmap_adapter ${PROJECT_NAME}_ros_adapter)
  ament_target_dependencies(test_pointcloud2_heightmap_adapter sensor_msgs)

  ament_add_gtest(test_planner_visualization test/test_planner_visualization.cpp)
  target_link_libraries(test_planner_visualization ${PROJECT_NAME}_visualization)

  ament_add_pytest_test(test_config_contract test/test_config_contract.py)
  add_launch_test(test/test_runtime_profile_launch.py TIMEOUT 60)
  add_launch_test(test/test_planner_node_launch.py TIMEOUT 90)
  set_tests_properties(test_test_runtime_profile_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=201")
  set_tests_properties(test_test_planner_node_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=202")
endif()

ament_package()
