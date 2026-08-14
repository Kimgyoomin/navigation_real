cmake_minimum_required(VERSION 3.8)
project(rubi_heightmap_wavefront_planner)

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
  src/terrain_snapshot.cpp
  src/terrain_evaluator.cpp
  src/path_revalidation.cpp
  src/plan_lifecycle.cpp
  src/wavefront_planner.cpp
  src/rrt_star_planner.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_visualization STATIC
  src/planner_visualization.cpp
)
target_compile_features(${PROJECT_NAME}_visualization PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_visualization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  ${PROJECT_NAME}_visualization
  ${PROJECT_NAME}_core
)
ament_target_dependencies(${PROJECT_NAME}_visualization
  builtin_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
)

add_executable(wavefront_planner_node
  src/planner_node.cpp
)
target_compile_features(wavefront_planner_node PUBLIC cxx_std_17)
target_link_libraries(wavefront_planner_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_visualization
)
ament_target_dependencies(wavefront_planner_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
  PATTERN "planner_visualization.hpp" EXCLUDE
)

install(
  TARGETS ${PROJECT_NAME}_core wavefront_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_executable(core_smoke
    test/core_smoke.cpp
  )
  target_link_libraries(core_smoke ${PROJECT_NAME}_core)
  add_test(NAME core_smoke COMMAND core_smoke)

  ament_add_gtest(test_terrain_evaluator
    test/test_terrain_evaluator.cpp
  )
  target_link_libraries(test_terrain_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_wavefront_planner
    test/test_wavefront_planner.cpp
  )
  target_link_libraries(test_wavefront_planner ${PROJECT_NAME}_core)

  ament_add_gtest(test_planner_visualization
    test/test_planner_visualization.cpp
  )
  target_link_libraries(
    test_planner_visualization
    ${PROJECT_NAME}_visualization
  )

  ament_add_gtest(test_rrt_star_planner
    test/test_rrt_star_planner.cpp
  )
  target_link_libraries(test_rrt_star_planner ${PROJECT_NAME}_core)

  ament_add_gtest(test_path_revalidation
    test/test_path_revalidation.cpp
  )
  target_link_libraries(test_path_revalidation ${PROJECT_NAME}_core)

  ament_add_gtest(test_plan_lifecycle
    test/test_plan_lifecycle.cpp
  )
  target_link_libraries(test_plan_lifecycle ${PROJECT_NAME}_core)

  ament_add_pytest_test(
    test_wavefront_config_contract
    test/test_wavefront_config_contract.py
  )

  add_launch_test(
    test/test_planner_node_launch.py
    TIMEOUT 90
  )

  add_launch_test(
    test/test_runtime_profile_launch.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_rrt_star_node_launch.py
    TIMEOUT 90
  )

  add_executable(benchmark_flat_map
    test/benchmark_flat_map.cpp
  )
  target_link_libraries(benchmark_flat_map ${PROJECT_NAME}_core)
endif()

ament_package()
