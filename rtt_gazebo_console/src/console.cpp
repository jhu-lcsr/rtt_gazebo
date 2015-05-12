#include <iostream>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/CorbaDeploymentComponent.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>
#include <boost/program_options.hpp>

using namespace RTT::corba;
using namespace RTT;
namespace po = boost::program_options;

int ORO_main(int argc, char** argv)
{
  // Add parser for program options
  po::options_description description("RTT Gazebo Console Usage");
  description.add_options()
    ("help,h", "Display this help message")
    ("script,s",po::value<std::string>(), "Run an ops script");
    
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
  po::notify(vm);
  
  // Setup Corba:
  TaskContextProxy::InitOrb(argc, argv);

  // Get a pointer to the component above
  OCL::CorbaDeploymentComponent console_deployer("console_deployer");
  console_deployer.loadComponent("gazebo","CORBA");
  
  // Run an ops script if option is provided
  if(vm.count("script"))
      console_deployer.runScript(vm["script"].as<std::string>());
  
  // Interface it:
  OCL::TaskBrowser browse( &console_deployer );
  browse.loop();

  // Cleanup Corba:
  TaskContextProxy::DestroyOrb();
  return 0;
} 

