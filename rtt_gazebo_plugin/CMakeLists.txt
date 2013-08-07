cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(SDF REQUIRED)
find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Orocos-KDL)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS rtt
  )

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )

## Gazebo RTT plugin
add_library(${PROJECT_NAME} src/rtt_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SDFormat_LIBRARIES} orocos-kdl)

## Default component
orocos_plugin(default_gazebo_component src/default_gazebo_component.cpp)
target_link_libraries(default_gazebo_component orocos-kdl)
