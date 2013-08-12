#include <iostream>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/CorbaDeploymentComponent.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

using namespace RTT::corba;
using namespace RTT;

int ORO_main(int argc, char** argv)
{
  std::string RTT_COMPONENT_PATH;

  RTT_COMPONENT_PATH = std::string(getenv("RTT_COMPONENT_PATH"));
  RTT::log(RTT::Error) << "RTT_COMPONENT_PATH: " << RTT_COMPONENT_PATH <<std::endl;

  RTT::ComponentLoader::Instance()->setComponentPath(RTT_COMPONENT_PATH);
  // Setup Corba:
  TaskContextServer::InitOrb(argc, argv);

  // Setup a thread to handle call-backs to our components.
  TaskContextServer::ThreadOrb();

  // Get a pointer to the component above
  TaskContext* component = TaskContextProxy::Create( "gazebo" );
  OCL::DeploymentComponent cdc("console_deployer");
  cdc.path(RTT_COMPONENT_PATH);
  cdc.addPeer(component);

  // Interface it:
  OCL::TaskBrowser browse( &cdc );
  browse.loop();

  // Stop ORB thread:
  TaskContextServer::ShutdownOrb();
  // Cleanup Corba:
  TaskContextServer::DestroyOrb();
  return 0;
} 

