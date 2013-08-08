#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

using namespace RTT::corba;
using namespace RTT;

int ORO_main(int argc, char** argv)
{
  // Setup Corba:
  TaskContextServer::InitOrb(argc, argv);

  // Setup a thread to handle call-backs to our components.
  TaskContextServer::ThreadOrb();

  // Get a pointer to the component above
  TaskContext* component = TaskContextProxy::Create( "gazebo" );

  // Interface it:
  OCL::TaskBrowser browse( component );
  browse.loop();

  // Stop ORB thread:
  TaskContextServer::ShutdownOrb();
  // Cleanup Corba:
  TaskContextServer::DestroyOrb();
  return 0;
} 

