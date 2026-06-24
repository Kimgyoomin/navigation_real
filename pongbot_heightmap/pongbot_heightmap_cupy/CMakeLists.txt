cmake_minimum_required(VERSION 3.0.2)
project(pongbot_heightmap_cupy)

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(OpenCV REQUIRED)

if(PYTHONLIBS_FOUND)
  message(STATUS "Using Python Libraries at: " ${PYTHON_LIBRARIES})
  message(STATUS "Using Python include directories at: " ${PYTHON_INCLUDE_DIRS})
else()
  message(WARNING "Could not find Python Libraries")
endif()

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED
  roscpp
  rospy
  tf
  tf_conversions
  sensor_msgs
  std_msgs
  geometry_msgs
  grid_map_msgs
  grid_map_ros      
  image_transport
  pcl_ros
  pybind11_catkin
  pongbot_heightmap_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    roscpp 
    rospy
    tf
    tf_conversions
    sensor_msgs 
    std_msgs
    geometry_msgs
    grid_map_ros
    image_transport
    pcl_ros           
    pybind11_catkin
    pongbot_heightmap_msgs
)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(pongbot_heightmap_ros
    src/pongbot_heightmap_wrapper.cpp    
    src/pongbot_heightmap_ros.cpp
    src/Utilities/Utilities.cpp 
)

target_link_libraries(pongbot_heightmap_ros 
  ${PYTHON_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  lcm
)

add_dependencies(pongbot_heightmap_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(pongbot_heightmap_node src/pongbot_heightmap_node.cpp)
target_link_libraries(pongbot_heightmap_node pongbot_heightmap_ros)

catkin_python_setup()

install(
  TARGETS pongbot_heightmap_ros pongbot_heightmap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
