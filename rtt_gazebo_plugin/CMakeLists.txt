cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(PkgConfig REQUIRED)
#pkg_check_modules(TAO_PortableServer REQUIRED TAO_PortableServer)
#pkg_check_modules(TAO_CosNaming REQUIRED TAO_CosNaming)

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
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-transport-corba )
orocos_use_package( rtt-typekit )
orocos_use_package( rtt-scripting )

## Gazebo RTT plugin
add_library(${PROJECT_NAME} src/rtt_plugin.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES} 
  ${OROCOS-RTT_LIBRARIES}
  boost_thread
  orocos-kdl)

set(CMAKE_BUILD_TYPE Debug)

## Default component
orocos_component(default_gazebo_component src/default_gazebo_component.cpp)
set_target_properties(default_gazebo_component PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(default_gazebo_component orocos-kdl ${GAZEBO_LIBRARIES})

orocos_generate_package()
