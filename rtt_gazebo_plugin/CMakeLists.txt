cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_plugin)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(Orocos_KDL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  DEPENDS rtt gazebo eigen
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

## Gazebo RTT plugin
add_library(${PROJECT_NAME} src/rtt_plugin.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

#set(CMAKE_BUILD_TYPE Debug)

## Default component
orocos_component(default_gazebo_component src/default_gazebo_component.cpp)
set_target_properties(default_gazebo_component PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(default_gazebo_component ${orocos_kdl_LIBRARIES} ${GAZEBO_LIBRARIES})

orocos_generate_package()
