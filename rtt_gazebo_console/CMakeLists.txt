cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_console)

find_package(catkin REQUIRED)
catkin_package( DEPENDS rtt)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(PkgConfig REQUIRED)
pkg_check_modules(TAO_PortableServer REQUIRED TAO_PortableServer)
pkg_check_modules(TAO_CosNaming REQUIRED TAO_CosNaming)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-transport-corba )
orocos_use_package( rtt-typekit )

add_definitions(-DRTT_STATIC)
orocos_executable(console src/console.cpp)

orocos_generate_package()
