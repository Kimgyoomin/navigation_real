cmake_minimum_required(VERSION 3.0.2)
project(pongbot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   amcl
#   map_server
#   move_base
#   roscpp
#   rospy
#   tf
# )
find_package(catkin REQUIRED)
# find_package(LCM REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pongbot_navigation
#  CATKIN_DEPENDS amcl map_server move_base roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
install(DIRECTORY launch/ config/ maps/ rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${LCM_INCLUDE_DIRS}
)

# target_link_libraries(${LCM_LIBRARIES})


