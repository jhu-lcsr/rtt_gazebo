/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jonathan Bohren
   Desc:   Gazebo plugin for running OROCOS RTT components */

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


class RTTPlugin : public gazebo::ModelPlugin
{
public:

  virtual ~RTTPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Load in seperate thread from Gazebo in case something blocks
  void loadThread();

  // Called by the world update start event
  void Update();

private:
  // deferred load in case something blocks
  boost::thread deferred_load_thread_;

  // Gazebo structures
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;
  gazebo::event::ConnectionPtr update_connection_;
  std::string deployer_name_;

  // Orocos Structures
  static boost::shared_ptr<OCL::DeploymentComponent> gazebo_deployer;
  static RTT::corba::TaskContextServer * taskcontext_server;
  static std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > deployers;

  RTT::TaskContext* gazebo_component_;

  RTT::OperationCaller<void(void)> gazebo_update_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RTTPlugin);

// Static imeplementations
boost::shared_ptr<OCL::DeploymentComponent> RTTPlugin::gazebo_deployer;
RTT::corba::TaskContextServer * RTTPlugin::taskcontext_server;
std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > RTTPlugin::deployers;


RTTPlugin::~RTTPlugin()
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

// Overloaded Gazebo entry point
void RTTPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  // Set orocos environment variables
  std::string RTT_COMPONENT_PATH;

  RTT_COMPONENT_PATH = std::string(getenv("RTT_COMPONENT_PATH"));
  gzwarn << "RTT_COMPONENT_PATH: " << RTT_COMPONENT_PATH <<std::endl;

  RTT::ComponentLoader::Instance()->setComponentPath(RTT_COMPONENT_PATH);

  // Disable the RTT system clock so Gazebo can manipulate time
  RTT::os::TimeService::Instance()->enableSystemClock(false);

  // Create main gazebo deployer if necessary
  if(gazebo_deployer.get() == NULL) {

    int argc = 0;
    char **argv = NULL;

    __os_init(argc, argv);

    // Setup TaskContext server if necessary
    if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
      // Initialize orb
      RTT::corba::TaskContextServer::InitOrb(argc, argv);
      // Propcess orb requests in a thread
      RTT::corba::TaskContextServer::ThreadOrb();
    }

    // Create the gazebo deployer
    gazebo_deployer = boost::make_shared<OCL::DeploymentComponent>("gazebo");

    // Import the kdl typekit
    gazebo_deployer->import("kdl_typekit");

    // Attach the taskcontext server to this component
    taskcontext_server = 
      RTT::corba::TaskContextServer::Create( gazebo_deployer.get() );
  }

  // Check if this deployer should have a custom name
  if(sdf_->HasElement("deployer")) {
    deployer_name_ = sdf_->GetElement("deployer")->Get<std::string>();
  } else {
    deployer_name_ = "deployer"; 
  }

  // Create component deployer if necessary
  if(deployers.find(deployer_name_) == deployers.end()) {
    deployers[deployer_name_] = boost::make_shared<OCL::DeploymentComponent>(deployer_name_);
    deployers[deployer_name_]->connectPeers(gazebo_deployer.get());
    RTT::corba::TaskContextServer::Create( deployers[deployer_name_].get() );
  }

  deferred_load_thread_ = boost::thread(boost::bind(&RTTPlugin::loadThread, this));
}

// Load in seperate thread from Gazebo in case something blocks
void RTTPlugin::loadThread()
{
  // Error message if the model couldn't be found
  if (!parent_model_) {
    return;
  }

  // Get the orocos ops script
  if(sdf_->HasElement("opsScriptFile")) {
    std::string ops_script_file;
    ops_script_file = sdf_->GetElement("opsScriptFile")->Get<std::string>();
    if(!deployers[deployer_name_]->runScript(ops_script_file)) {
      return;
    }
  }
  
  // Get the orocos ops script
  if(sdf_->HasElement("opsScript")) {
    std::string ops_script;
    ops_script = sdf_->GetElement("opsScript")->Get<std::string>();
    if(!deployers[deployer_name_]->getProvider<RTT::Scripting>("scripting")->eval(ops_script)) {
      return;
    }
  }

  // Initialize gazebo component
  gazebo_component_ = NULL;

  // Check if there is a special gazebo component
  if(sdf_->HasElement("component")) {
    // Get the component name
    std::string gazebo_component_name = sdf_->GetElement("component")->Get<std::string>();

    // Get the gazebo component from the deployer by name
    if(deployers[deployer_name_]->hasPeer(gazebo_component_name)) {
      gazebo_component_ = deployers[deployer_name_]->getPeer(gazebo_component_name);
    } else {
      return;
    }
  } else {
    // Import the default component
    try {
      deployers[deployer_name_]->import("rtt_gazebo_plugin");
    } catch(std::runtime_error &err) {
      gzerr << "Could not load rtt_gazebo_plugin: " << err.what() <<std::endl;
      return;
    }

    deployers[deployer_name_]->loadComponent(parent_model_->GetName(),"DefaultGazeboComponent");
    // Create a gazebo component with the same name as the model
    gazebo_component_ = deployers[deployer_name_]->getPeer(parent_model_->GetName());
  }

  // Make sure the component has the required interfaces
  if( gazebo_component_ == NULL ||
      !gazebo_component_->provides()->hasService("gazebo") ||
      !gazebo_component_->provides("gazebo")->hasOperation("setModel") ||
      !gazebo_component_->provides("gazebo")->hasOperation("getModel") ||
      !gazebo_component_->provides("gazebo")->hasOperation("update")) 
  {
    return;
  }

  RTT::corba::TaskContextServer::Create( gazebo_component_ );

  // Set the component's gazebo model
  RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_set_model = 
    gazebo_component_->provides("gazebo")->getOperation("setModel");

  gazebo_set_model(parent_model_);

  // Get gazebo update function
  gazebo_update_ = 
    gazebo_component_->provides("gazebo")->getOperation("update");

  // Configure the gazebo component
  if(!gazebo_component_->configure()) {
    return;
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = 
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RTTPlugin::Update, this));

}

// Called by the world update start event
void RTTPlugin::Update()
{
  // Get the simulation time
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();

  // Update the RTT time to match the gazebo time
  using namespace RTT::os;
  TimeService *rtt_time = TimeService::Instance();
  TimeService::Seconds dt = std::max(0.0,TimeService::Seconds((gz_time_now.sec*1E6 + gz_time_now.nsec) - rtt_time->getNSecs()));
  rtt_time->secondsChange(dt);

  if(gazebo_component_ == NULL) {
    return;
  }

  // Call orocos gazebo component gazeboUpdate()
  gazebo_update_();
}
