
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

#include "rtt_system.h"

using namespace rtt_gazebo_deployer;

boost::shared_ptr<RTTSystem> RTTSystem::Instance() 
{
  // Create a new instance, if necessary
  boost::shared_ptr<RTTSystem> shared = singleton.lock();
  if(singleton.expired()) {
    shared.reset(new RTTSystem());
    singleton = shared;
  }

  return shared;
}

boost::shared_ptr<RTTSystem> RTTSystem::GetInstance() 
{
  return singleton.lock();
}

RTTSystem::RTTSystem() 
{
  // Args for init functions
  // TODO: Get these from SDF
  int argc = 0;
  char **argv = NULL;

  // Initialize RTT
  __os_init(argc, argv);

  // Initialize and enable the simulation clock 
  rtt_rosclock::use_manual_clock();
  rtt_rosclock::enable_sim();

  // Setup TaskContext server if necessary
  if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    // Initialize orb
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    // Propcess orb requests in a thread
    RTT::corba::TaskContextServer::ThreadOrb();
  }
}

RTTSystem::~RTTSystem() 
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

void RTTSystem::connectWorld(gazebo::physics::WorldPtr world) 
{
  // Only set the world if it hasn't been set yet
  if(world_.get() == NULL) {
    // Store the world
    world_ = world;
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = 
      gazebo::event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RTTSystem::updateClock, this));
  }
}

void RTTSystem::updateClock()
{
  // Make sure the world isn't an illusion
  if(world_.get() == NULL) {
    return;
  }

  // Get the simulation time
  gazebo::common::Time gz_time = world_->GetSimTime();
  RTT::os::TimeService::Seconds gz_secs = 
    (RTT::Seconds)gz_time.sec +
    ((RTT::Seconds)gz_time.nsec)*1E-9;

  // Update the clock from the simulation time and execute the SimClockActivities
  rtt_rosclock::update_sim_clock(gz_secs);
}
