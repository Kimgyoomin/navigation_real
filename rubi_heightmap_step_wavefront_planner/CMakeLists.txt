cmake_minimum_required(VERSION 3.8)
project(rubi_heightmap_step_wavefront_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/heightmap_snapshot.cpp
  src/map/costmap_snapshot.cpp
  src/terrain/height_evidence.cpp
  src/step_evaluator.cpp
  src/graph/spatial_index_2d.cpp
  src/graph/wavefront_graph_builder.cpp
  src/search/astar_search.cpp
  src/planning/step_wavefront_planner.cpp
  src/planning/planning_fsm.cpp
  src/planning/planning_query_resolver.cpp
  src/planning/step_grid_astar_planner.cpp
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

add_library(${PROJECT_NAME}_ros_adapter STATIC
  src/ros/pointcloud2_heightmap_adapter.cpp
  src/ros/nav2_costmap_adapter.cpp)
target_compile_features(${PROJECT_NAME}_ros_adapter PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ros_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_ros_adapter ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros_adapter nav2_msgs sensor_msgs)

add_executable(step_wavefront_planner_node src/planner_node.cpp)
target_compile_features(step_wavefront_planner_node PUBLIC cxx_std_17)
target_link_libraries(step_wavefront_planner_node
  ${PROJECT_NAME}_core ${PROJECT_NAME}_visualization ${PROJECT_NAME}_ros_adapter)
ament_target_dependencies(step_wavefront_planner_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs
  tf2_ros visualization_msgs)

add_executable(hybrid_planner_comparison_node src/hybrid_planner_comparison_node.cpp)
target_compile_features(hybrid_planner_comparison_node PUBLIC cxx_std_17)
target_link_libraries(hybrid_planner_comparison_node
  ${PROJECT_NAME}_core ${PROJECT_NAME}_visualization ${PROJECT_NAME}_ros_adapter)
ament_target_dependencies(hybrid_planner_comparison_node
  geometry_msgs nav2_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs
  tf2_ros visualization_msgs)

# Humble's nav2_costmap_2d executable hard-codes /costmap/costmap and ignores
# launch name/namespace remaps. This thin main selects the required stable FQN
# while using Nav2's unmodified Costmap2DROS implementation.
add_executable(hybrid_global_costmap_node src/hybrid_global_costmap_main.cpp)
target_compile_features(hybrid_global_costmap_node PUBLIC cxx_std_17)
ament_target_dependencies(hybrid_global_costmap_node nav2_costmap_2d rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core step_wavefront_planner_node
  hybrid_planner_comparison_node hybrid_global_costmap_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)
install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/run_grid_navigation.sh
  scripts/run_sampling_navigation.sh
  scripts/run_hybrid_comparison.sh
  scripts/check_hybrid_nav_inputs.sh
  DESTINATION lib/${PROJECT_NAME})
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
      test_planning_query_resolver
      test_costmap_snapshot
      test_height_evidence
      test_hybrid_step_evaluator
      test_step_grid_astar_planner
      test_trg_sampling_policy
      test_original_trg_construction
      test_grid_sampling_contract
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

  ament_add_gtest(test_nav2_costmap_adapter test/test_nav2_costmap_adapter.cpp)
  target_link_libraries(test_nav2_costmap_adapter ${PROJECT_NAME}_ros_adapter)
  ament_target_dependencies(test_nav2_costmap_adapter nav2_msgs)

  ament_add_gtest(test_planner_visualization test/test_planner_visualization.cpp)
  target_link_libraries(test_planner_visualization ${PROJECT_NAME}_visualization)

  ament_add_pytest_test(test_config_contract test/test_config_contract.py)
  ament_add_pytest_test(test_hybrid_map_install test/test_hybrid_map_install.py)
  ament_add_pytest_test(test_hybrid_costmap_config test/test_hybrid_costmap_config.py)
  add_launch_test(test/test_runtime_profile_launch.py TIMEOUT 60)
  add_launch_test(test/test_planner_node_launch.py TIMEOUT 90)
  add_launch_test(test/test_hybrid_comparison_launch.py TIMEOUT 90)
  add_launch_test(test/test_hybrid_costmap_bringup_launch.py TIMEOUT 120)
  set_tests_properties(test_test_runtime_profile_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=201")
  set_tests_properties(test_test_planner_node_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=202")
  set_tests_properties(test_test_hybrid_comparison_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=203")
  set_tests_properties(test_test_hybrid_costmap_bringup_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=204")
endif()

ament_package()
