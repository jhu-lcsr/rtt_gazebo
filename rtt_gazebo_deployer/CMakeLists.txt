cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_deployer)

## Find catkin macros and  braries
find_package(catkin REQUIRED)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo REQUIRED)

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

## Gazebo RTT plugin
add_library(${PROJECT_NAME} 
  src/gazebo_deployer_model_plugin.cpp)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set(CMAKE_BUILD_TYPE Debug)

orocos_generate_package()
