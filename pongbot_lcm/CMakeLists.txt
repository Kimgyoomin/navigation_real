cmake_minimum_required(VERSION 3.0.2)
project(pongbot_lcm)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes   # robotlcm/*.hpp가 여기 아래이기 때문에 추가
)

# ---- LCM type generation (.lcm -> .hpp) ----
set(LCM_TYPES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes)
file(GLOB LCM_TYPE_FILE "${LCM_TYPES_DIR}/*.lcm")

set(LCM_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/lcm_gen)
# 중요: configure 단계에서 디렉토리 생성 (make 단계에서 cd 실패 방지)
file(MAKE_DIRECTORY ${LCM_GEN_DIR})

# lcm-gen은 한 번 호출로 여러 타입 생성 가능
# (WORKING_DIRECTORY를 LCM_GEN_DIR로 두면 robotlcm/ 아래 생성됨)
add_custom_command(
  OUTPUT ${LCM_GEN_DIR}/.lcm_gen_stamp
  COMMAND lcm-gen -x ${LCM_TYPE_FILES}
  COMMAND ${CMAKE_COMMAND} -E touch ${LCM_GEN_DIR}/.lcm_gen_stamp
  WORKING_DIRECTORY ${LCM_GEN_DIR}
  DEPENDS ${LCM_TYPE_FILES}
  COMMENT "Generating LCM types from ${LCM_TYPES_DIR}"
)

# add_custom_target(lcmtypes_gen ALL
#   DEPENDS ${LCM_GEN_HEADER}
# )
add_custom_target(lcmtypes_gen ALL
  DEPENDS ${LCM_GEN_DIR}/.lcm_gen_stamp
)

include_directories(${LCM_GEN_DIR})

# set(LCM_TYPE_FILE ${LCM_TYPES_DIR}/robot_data_t.lcm)


# set(LCM_GEN_HEADER ${LCM_GEN_DIR}/robotlcm/robot_data_t.hpp)




# add_custom_command(
#   OUTPUT ${LCM_GEN_HEADER}
#   COMMAND lcm-gen -x ${LCM_TYPE_FILE}
#   WORKING_DIRECTORY ${LCM_GEN_DIR}
#   DEPENDS ${LCM_TYPE_FILE}
#   COMMENT "Generating LCM types from ${LCM_TYPE_FILE}"
# )




# ---- executables ----
add_executable(lcm_server src/lcm_server.cpp)
add_executable(lcm_client src/lcm_client.cpp)

add_dependencies(lcm_server lcmtypes_gen)
add_dependencies(lcm_client lcmtypes_gen)

target_link_libraries(lcm_server
  ${catkin_LIBRARIES}
  ${LCM_LIBRARIES}
)

target_link_libraries(lcm_client
  ${catkin_LIBRARIES}
  ${LCM_LIBRARIES}
)

add_executable(ros_cmd_vel_to_lcm src/ros_cmd_vel_to_lcm.cpp)
add_executable(lcm_cmdvel_listener src/lcm_cmdvel_listener.cpp)

add_dependencies(ros_cmd_vel_to_lcm lcmtypes_gen)
add_dependencies(lcm_cmdvel_listener lcmtypes_gen)

target_link_libraries(ros_cmd_vel_to_lcm
  ${catkin_LIBRARIES}
  ${LCM_LIBRARIES}
)

target_link_libraries(lcm_cmdvel_listener
  ${LCM_LIBRARIES}
)
