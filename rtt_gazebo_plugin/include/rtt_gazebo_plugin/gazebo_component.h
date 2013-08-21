#ifndef __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT
#define __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

namespace rtt_gazebo_plugin {

  bool add_gazebo_interfaces(
      RTT::TaskContext *component,
      bool (RTT::TaskContext::*gazebo_configure_hook)(gazebo::physics::ModelPtr),
      void (RTT::TaskContext::*gazebo_update_hook)(gazebo::physics::ModelPtr))
  {
    component->provides("gazebo")->addOperation("configure",gazebo_configure_hook,component,RTT::ClientThread);
    component->provides("gazebo")->addOperation("update",gazebo_update_hook,component,RTT::ClientThread);
    return true;
  }

}

#endif // ifndef __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT
