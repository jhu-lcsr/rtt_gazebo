
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

using namespace rtt_gazebo_system;

void RTTSystemPlugin::Load(int argc, char **argv)
{
  // Initialize RTT
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(gzerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  // Setup TaskContext server if necessary
  if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    // Initialize orb
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    // Propcess orb requests in a thread
    RTT::corba::TaskContextServer::ThreadOrb();
  }
}

void RTTSystemPlugin::Init()
{
  // Initialize and enable the simulation clock 
  rtt_rosclock::use_manual_clock();
  rtt_rosclock::enable_sim();
  
  update_connection_ = 
    gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RTTSystemPlugin::updateClock, this));
}

RTTSystemPlugin::~RTTSystemPlugin() 
{
  // Stop the Orb thread
  if(!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    RTT::corba::TaskContextServer::CleanupServers();
  }
}

void RTTSystemPlugin::updateClock()
{
  // Get the simulation time
  gazebo::common::Time gz_time = gazebo::physics::get_world()->GetSimTime();

  // Update the clock from the simulation time and execute the SimClockActivities
  rtt_rosclock::update_sim_clock(ros::Time(gz_time.sec, gz_time.nsec));
}

GZ_REGISTER_SYSTEM_PLUGIN(rtt_gazebo_system::RTTSystemPlugin)
