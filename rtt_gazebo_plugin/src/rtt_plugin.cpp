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

class RTTSystem
{
public:
  //! Get an instance of the singleton
  static boost::shared_ptr<RTTSystem> Instance() 
  {
    // Cache the singleton with a weak pointer
    static boost::weak_ptr<RTTSystem> instance;

    // Create a new instance, if necessary
    boost::shared_ptr<RTTSystem> shared = instance.lock();
    if(instance.expired()) {
      shared.reset(new RTTSystem());
      instance = shared;
    }
    
    return shared;
  }

  //! Set the world time source
  void connectWorld(gazebo::physics::WorldPtr world) 
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

  //! Update the RTT clock from the gazebo clock
  void updateClock()
  {
    // Make sure the world isn't an illusion
    if(world_.get() == NULL) {
      return;
    }

    // Update the RTT time to match the gazebo time
    using namespace RTT::os;
    TimeService *rtt_time = TimeService::Instance();
    TimeService::ticks rtt_ticks = rtt_time->getTicks();
    TimeService::Seconds rtt_secs = TimeService::ticks2nsecs(rtt_ticks)*1E-9;

    // Get the simulation time
    gazebo::common::Time gz_time = world_->GetSimTime();
    TimeService::Seconds gz_secs = (TimeService::Seconds)gz_time.sec + ((TimeService::Seconds)gz_time.nsec)*1E-9;

    // Compute the time update
    TimeService::Seconds dt = gz_secs-rtt_secs;

    // Check if time went backwards
    if(dt < 0) {
      gzwarn << "Time went backwards by "<<dt<<" seconds!" << std::endl;
    }

    // Update the RTT clock
    // TODO: Interpolate over a higher-resolution range?
    rtt_time->secondsChange(dt);
  }

  ~RTTSystem() {
    // Disconnect from gazebo events
    gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);

    // Stop the Orb thread
    if(!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
      RTT::corba::TaskContextServer::ShutdownOrb();
      RTT::corba::TaskContextServer::DestroyOrb();
      RTT::corba::TaskContextServer::CleanupServers();
    }
  }

private:
  // Singleton constraints
  RTTSystem() {
    // Args for init functions
    // TODO: Get these from SDF
    int argc = 0;
    char **argv = NULL;

    // Initialize RTT
    __os_init(argc, argv);

    // Disable the RTT system clock so Gazebo can manipulate time
    RTT::os::TimeService::Instance()->enableSystemClock(false);

    // Setup TaskContext server if necessary
    if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
      // Initialize orb
      RTT::corba::TaskContextServer::InitOrb(argc, argv);
      // Propcess orb requests in a thread
      RTT::corba::TaskContextServer::ThreadOrb();
    }
  }
  RTTSystem(RTTSystem const&);
  void operator=(RTTSystem const&);

  // Gazebo world to get time from
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
};

class RTTPlugin : public gazebo::ModelPlugin
{
public:

  /* \brief Constructor
   * Gets an instance of \ref RTTSystem
   */
  RTTPlugin();

  //! Destructor, disconnects update
  virtual ~RTTPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Load in seperate thread from Gazebo in case something blocks
  void loadThread();

  // Called by the world update start event
  void gazeboUpdate();

private:

  // Shared pointer to the RTTSystem singleton
  boost::shared_ptr<RTTSystem> rtt_system_;

  // deferred load in case something blocks
  boost::thread deferred_load_thread_;

  // Gazebo structures
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;
  gazebo::event::ConnectionPtr update_connection_;
  std::string deployer_name_;

  // Orocos Structures
  static RTT::corba::TaskContextServer * taskcontext_server;
  static std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > deployers;

  RTT::TaskContext* model_component_;

  // Operation for polling the
  RTT::OperationCaller<void(gazebo::physics::ModelPtr)> gazebo_update_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RTTPlugin);

// Static imeplementations
RTT::corba::TaskContextServer * RTTPlugin::taskcontext_server;
std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > RTTPlugin::deployers;

RTTPlugin::RTTPlugin() : 
  gazebo::ModelPlugin(), 
  rtt_system_(RTTSystem::Instance())
{
}

RTTPlugin::~RTTPlugin()
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

// Overloaded Gazebo entry point
void RTTPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;
  
  // Initialize the time synchronizer
  rtt_system_->connectWorld(parent->GetWorld());

  // Create main gazebo deployer if necessary
  if(deployers.find("gazebo") == deployers.end()) {
    // Create the gazebo deployer
    deployers["gazebo"] = boost::make_shared<OCL::DeploymentComponent>("gazebo");
    deployers["gazebo"]->import("kdl_typekit");

    // Attach the taskcontext server to this component
    taskcontext_server = RTT::corba::TaskContextServer::Create(deployers["gazebo"].get());
  }

  // Check if this deployer should have a custom name
  if(sdf_->HasElement("isolated")) {
    deployer_name_ = parent_model_->GetName()+std::string("__deployer__");
  } else {
    deployer_name_ = "gazebo"; 
  }

  // Create component deployer if necessary
  if(deployers.find(deployer_name_) == deployers.end()) {
    deployers[deployer_name_] = boost::make_shared<OCL::DeploymentComponent>(deployer_name_);
    deployers[deployer_name_]->connectPeers(deployers["gazebo"].get());
    RTT::corba::TaskContextServer::Create(deployers[deployer_name_].get());
  }

  // Perform the rest of the asynchronous loading
  //deferred_load_thread_ = boost::thread(boost::bind(&RTTPlugin::loadThread, this));
  this->loadThread();
}

// Load in seperate thread from Gazebo in case something blocks
void RTTPlugin::loadThread()
{
  // Error message if the model couldn't be found
  if (!parent_model_) {
    return;
  }

  // Initialize gazebo component
  model_component_ = NULL;

  if(!sdf_->HasElement("component")) {
    // Import the default component
    try {
      deployers[deployer_name_]->import("rtt_gazebo_plugin");
    } catch(std::runtime_error &err) {
      gzerr << "Could not load rtt_gazebo_plugin: " << err.what() <<std::endl;
      return;
    }
    deployers[deployer_name_]->loadComponent(parent_model_->GetName(),"DefaultGazeboComponent");
    // Create a gazebo component with the same name as the model
    model_component_ = deployers[deployer_name_]->getPeer(parent_model_->GetName());
  }

  // Get the orocos ops script
  if(sdf_->HasElement("opsScriptFile")) {
    std::string ops_script_file;
    ops_script_file = sdf_->GetElement("opsScriptFile")->Get<std::string>();
    if(!deployers[deployer_name_]->runScript(ops_script_file)) {
      gzerr << "Could not run ops script file "<<ops_script_file<<"!" << std::endl;
      return;
    }
  }
  
  // Get the orocos ops script
  if(sdf_->HasElement("opsScript")) {
    std::string ops_script;
    ops_script = sdf_->GetElement("opsScript")->Get<std::string>();
    if(!deployers[deployer_name_]->getProvider<RTT::Scripting>("scripting")->eval(ops_script)) {
      gzerr << "Could not run inline ops script!" << std::endl;
      return;
    }
  }

  // Check if there is a special gazebo component that should be loaded
  if(sdf_->HasElement("component")) {
    // Get the component name
    std::string model_component_name = sdf_->GetElement("component")->Get<std::string>();

    // Get the model component from the deployer by name
    if(deployers[deployer_name_]->hasPeer(model_component_name)) {
      model_component_ = deployers[deployer_name_]->getPeer(model_component_name);
    } else {
      gzerr << "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""<<model_component_name<<"\", but there is no peer by that name." <<std::endl;
      return;
    }
  }

  // Make sure the component has the required interfaces
  if( model_component_ == NULL ) {
    gzerr << "RTT model component was not properly created." << std::endl; return; }
  if( !model_component_->provides()->hasService("gazebo") ) {
    gzerr << "RTT model component does not have required \"gazebo\" service." << std::endl; return; }
  if( !model_component_->provides("gazebo")->hasOperation("configure") ) {
    gzerr << "RTT model component does not have required \"gazebo.configure\" operation." << std::endl; return; }
  if( !model_component_->provides("gazebo")->hasOperation("update") ) {
    gzerr << "RTT model component does not have required \"gazebo.update\" operation." << std::endl; return; }

  // Configure the component with the parent model
  RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure = model_component_->provides("gazebo")->getOperation("configure");
  // Make sure the operation is ready
  if(gazebo_configure.ready()) {
    if(!gazebo_configure(parent_model_)){
      gzerr <<"RTT model component's \"gazebo.configure\" operation returned false." << std::endl;
      return;
    }
  } else {
    gzerr <<"RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature." << std::endl;
    return;
  }

  // Configure the model component
  if(!model_component_->configure()) {
    gzerr <<"Could not configure the RTT model component." << std::endl;
    return;
  }

  // Start the model component
  if(!model_component_->start()) {
    gzerr <<"Could not start the RTT model component." << std::endl;
    return;
  }

  // Get gazebo update function
  gazebo_update_ = model_component_->provides("gazebo")->getOperation("update");
  if(!gazebo_update_.ready()) {
    gzerr <<"RTT model component's \"gazebo.update\" operation could not be connected. Check its signature." << std::endl;
    return;
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&RTTPlugin::gazeboUpdate, this));
}

// Called by the world update start event
void RTTPlugin::gazeboUpdate()
{
  // Make sure the gazebo component exists
  if(model_component_ == NULL) {
    return;
  }

  // Call orocos RTT model component gazebo.update()
  gazebo_update_(parent_model_);
}
