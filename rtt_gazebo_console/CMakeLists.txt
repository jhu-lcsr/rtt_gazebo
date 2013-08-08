cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_console)

find_package(catkin REQUIRED)
catkin_package( DEPENDS rtt)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )

add_executable(console src/console.cpp)
#target_link_libraries(console orocos-rtt-corba-${OROCOS_TARGET})
#target_link_libraries(console ${OROCOS-RTT_RTT_CORBA_LIBRARIES})
orocos_generate_package()
