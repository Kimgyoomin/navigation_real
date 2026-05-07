cmake_minimum_required(VERSION 3.8)
project(pongbot_lcm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

set(LCM_TYPES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes)
file(GLOB LCM_TYPE_FILES "${LCM_TYPES_DIR}/*.lcm")

set(LCM_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/lcm_gen)
file(MAKE_DIRECTORY ${LCM_GEN_DIR})

add_custom_command(
  OUTPUT ${LCM_GEN_DIR}/.lcm_gen_stamp
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LCM_GEN_DIR}
  COMMAND lcm-gen -x ${LCM_TYPE_FILES}
  COMMAND ${CMAKE_COMMAND} -E touch ${LCM_GEN_DIR}/.lcm_gen_stamp
  WORKING_DIRECTORY ${LCM_GEN_DIR}
  DEPENDS ${LCM_TYPE_FILES}
  COMMENT "Generating LCM types"
)

add_custom_target(lcmtypes_gen ALL DEPENDS ${LCM_GEN_DIR}/.lcm_gen_stamp)

add_executable(cmdvel_to_lcm_node src/cmdvel_to_lcm_node.cpp)
add_dependencies(cmdvel_to_lcm_node lcmtypes_gen)
target_include_directories(cmdvel_to_lcm_node PRIVATE ${LCM_INCLUDE_DIRS} ${LCM_GEN_DIR})
ament_target_dependencies(cmdvel_to_lcm_node rclcpp geometry_msgs)
target_link_libraries(cmdvel_to_lcm_node ${LCM_LIBRARIES})

add_executable(lcm_test_publisher src/lcm_test_publisher.cpp)
add_dependencies(lcm_test_publisher lcmtypes_gen)
target_include_directories(lcm_test_publisher PRIVATE ${LCM_INCLUDE_DIRS} ${LCM_GEN_DIR})
target_link_libraries(lcm_test_publisher ${LCM_LIBRARIES})

add_executable(lcm_test_listener src/lcm_test_listener.cpp)
add_dependencies(lcm_test_listener lcmtypes_gen)
target_include_directories(lcm_test_listener PRIVATE ${LCM_INCLUDE_DIRS} ${LCM_GEN_DIR})
target_link_libraries(lcm_test_listener ${LCM_LIBRARIES})

add_executable(joy_lcm_node src/joy_lcm_node.cpp)
add_dependencies(joy_lcm_node lcmtypes_gen)
target_include_directories(joy_lcm_node PRIVATE ${LCM_INCLUDE_DIRS} ${LCM_GEN_DIR})
ament_target_dependencies(joy_lcm_node rclcpp sensor_msgs)
target_link_libraries(joy_lcm_node ${LCM_LIBRARIES})

install(TARGETS
  joy_lcm_node
  cmdvel_to_lcm_node
  lcm_test_publisher
  lcm_test_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY lcmtypes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()