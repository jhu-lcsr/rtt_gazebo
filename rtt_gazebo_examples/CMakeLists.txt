cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_examples)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules)


# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

# Add c++11 for Gazebo >= 6.0
if(${GAZEBO_VERSION} GREATER 6)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
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

#set(CMAKE_BUILD_TYPE Debug)

## Default component
orocos_component(default_gazebo_component src/default_gazebo_component.cpp)
set_target_properties(default_gazebo_component PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(default_gazebo_component ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GAZEBO_LIBRARIES})

orocos_generate_package()
