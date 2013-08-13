#include <iostream>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/CorbaDeploymentComponent.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

using namespace RTT::corba;
using namespace RTT;

int ORO_main(int argc, char** argv)
{
  // Setup Corba:
  TaskContextProxy::InitOrb(argc, argv);

  // Get a pointer to the component above
  OCL::CorbaDeploymentComponent console_deployer("console_deployer");
  console_deployer.loadComponent("gazebo","CORBA");
  console_deployer.import("kdl_typekit");

  // Interface it:
  OCL::TaskBrowser browse( &console_deployer );
  browse.loop();

  // Cleanup Corba:
  TaskContextProxy::DestroyOrb();
  return 0;
} 

