
#include <cstdlib>

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


#include <rtt/os/startstop.h>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

// RTT/ROS Simulation Clock Activity
#include <rtt_rosclock/rtt_rosclock.h>

#include "rtt_system_plugin.h"

using namespace rtt_gazebo_world;

GZ_REGISTER_WORLD_PLUGIN(rtt_gazebo_world::RTTSystemPlugin)

RTTSystemPlugin::RTTSystemPlugin() 
{
}

void RTTSystemPlugin::initialize()
{
  // Args for init functions
  // TODO: Get these from SDF
  int argc = 0;
  char **argv = NULL;

  // Initialize RTT
  __os_init(argc, argv);

  // Setup TaskContext server if necessary
  if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    // Initialize orb
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    // Propcess orb requests in a thread
    RTT::corba::TaskContextServer::ThreadOrb();
  }
}

void RTTSystemPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  // Run RTT initialization
  this->initialize();

  // Connect the RTT system to the world update (unless it already has been, in
  // which case this is a noop)
  this->connectWorld(world);

  // Initialize and enable the simulation clock 
  rtt_rosclock::use_manual_clock();
  rtt_rosclock::enable_sim();
}

RTTSystemPlugin::~RTTSystemPlugin() 
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);

  // Stop the Orb thread
  if(!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    RTT::corba::TaskContextServer::CleanupServers();
  }
}

void RTTSystemPlugin::connectWorld(gazebo::physics::WorldPtr world) 
{
  // Only set the world if it hasn't been set yet
  if(world_.get() == NULL) {
    // Store the world
    world_ = world;
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = 
      gazebo::event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RTTSystemPlugin::updateClock, this));
  }
}

void RTTSystemPlugin::updateClock()
{
  // Make sure the world isn't an illusion
  if(world_.get() == NULL) {
    return;
  }

  // Get the simulation time
  gazebo::common::Time gz_time = world_->GetSimTime();
  RTT::os::TimeService::Seconds gz_secs = gz_time.Double();

  // Update the clock from the simulation time and execute the SimClockActivities
  rtt_rosclock::update_sim_clock(ros::Time(gz_time.sec, gz_time.nsec));
}
