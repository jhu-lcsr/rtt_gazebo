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

// RTT/ROS Simulation Clock Activity

#include "rtt_system.h"
#include "gazebo_deployer_model_plugin.h"

using namespace rtt_gazebo_deployer;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboDeployerModelPlugin);

// Static definitions
RTT::corba::TaskContextServer * GazeboDeployerModelPlugin::taskcontext_server;

std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > GazeboDeployerModelPlugin::deployers;

boost::weak_ptr<RTTSystem> RTTSystem::singleton;

GazeboDeployerModelPlugin::GazeboDeployerModelPlugin() : 
  gazebo::ModelPlugin()
{
}

GazeboDeployerModelPlugin::~GazeboDeployerModelPlugin()
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

// Overloaded Gazebo entry point
void GazeboDeployerModelPlugin::Load(
    gazebo::physics::ModelPtr parent, 
    sdf::ElementPtr sdf)
{
  // Get an instance to the RTT subsystem
  rtt_system_ = RTTSystem::Instance();

  // Save pointer to the model
  parent_model_ = parent;
  
  // Save the SDF source
  sdf_ = sdf;
  
  // Connect the RTT system to the world update (unless it already has been, in which case this is a noop)
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
  //deferred_load_thread_ = boost::thread(boost::bind(&GazeboDeployerModelPlugin::loadThread, this));
  this->loadThread();
}

// Load in seperate thread from Gazebo in case something blocks
void GazeboDeployerModelPlugin::loadThread()
{
  // Error message if the model couldn't be found
  if (!parent_model_) {
    return;
  }

  // Initialize gazebo component
  model_component_ = NULL;

  // Get a pointer to this model's deployer
  boost::shared_ptr<OCL::DeploymentComponent> deployer = deployers[deployer_name_];

  // Get the orocos ops script to run in the deployer
  if(sdf_->HasElement("opsScriptFile")) 
  {
    std::string ops_script_file;
    ops_script_file = sdf_->GetElement("opsScriptFile")->Get<std::string>();
    if(!deployer->runScript(ops_script_file)) {
      gzerr << "Could not run ops script file "<<ops_script_file<<"!" << std::endl;
      return;
    }
  }
  else if(sdf_->HasElement("opsScript")) 
  {
    std::string ops_script;
    ops_script = sdf_->GetElement("opsScript")->Get<std::string>();
    if(!deployer->getProvider<RTT::Scripting>("scripting")->eval(ops_script)) {
      gzerr << "Could not run inline ops script!" << std::endl;
      return;
    }
  }

  // Check if there is a special gazebo component that should be connected to the world
  if(sdf_->HasElement("component")) {
    // Get the component name
    std::string model_component_name = sdf_->GetElement("component")->Get<std::string>();

    // Get the model component from the deployer by name
    if(deployer->hasPeer(model_component_name)) {
      model_component_ = deployer->getPeer(model_component_name);
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
  RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure = 
    model_component_->provides("gazebo")->getOperation("configure");

  // Make sure the operation is ready
  if(!gazebo_configure.ready()) {
    gzerr <<"RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature." << std::endl;
    return;
  }

  if(!gazebo_configure(parent_model_)){
    gzerr <<"RTT model component's \"gazebo.configure\" operation returned false." << std::endl;
    return;
  }

  // Get gazebo update function
  gazebo_update_ = model_component_->provides("gazebo")->getOperation("update");
  if(!gazebo_update_.ready()) {
    gzerr <<"RTT model component's \"gazebo.update\" operation could not be connected. Check its signature." << std::endl;
    return;
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboDeployerModelPlugin::gazeboUpdate, this));
}

// Called by the world update start event
void GazeboDeployerModelPlugin::gazeboUpdate()
{
  // Make sure the gazebo component exists
  if(model_component_ == NULL) {
    return;
  }

  // Call orocos RTT model component gazebo.update()
  gazebo_update_(parent_model_);
}
