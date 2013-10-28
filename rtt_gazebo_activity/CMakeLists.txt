cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_activity)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include)

orocos_library(rtt_gazebo_activity src/gazebo_activity.cpp)
target_link_libraries(rtt_gazebo_activity ${USE_OROCOS_LIBRARIES})

orocos_use_package(rtt_rosgraph_msgs)
orocos_plugin(rtt_gazebo_service src/gazebo_service.cpp)
target_link_libraries(rtt_gazebo_service rtt_gazebo_activity ${USE_OROCOS_LIBRARIES})

#############
## Install ##
#############

orocos_generate_package()
