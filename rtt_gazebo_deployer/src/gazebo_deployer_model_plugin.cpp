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

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

#include <rtt_ros/rtt_ros.h>

// RTT/ROS Simulation Clock Activity

#include "gazebo_deployer_model_plugin.h"

using namespace rtt_gazebo_deployer;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboDeployerModelPlugin);

// Static definitions
RTT::corba::TaskContextServer * GazeboDeployerModelPlugin::taskcontext_server;

std::map<std::string,boost::shared_ptr<OCL::DeploymentComponent> > GazeboDeployerModelPlugin::deployers;

GazeboDeployerModelPlugin::GazeboDeployerModelPlugin() : 
  gazebo::ModelPlugin()
{
}

GazeboDeployerModelPlugin::~GazeboDeployerModelPlugin()
{
  // Disconnect from gazebo events
  for(std::vector<gazebo::event::ConnectionPtr>::iterator it = update_connections_.begin();
      it != update_connections_.end();
      ++it)
  {
    gazebo::event::Events::DisconnectWorldUpdateBegin(*it);
  }
}

// Overloaded Gazebo entry point
void GazeboDeployerModelPlugin::Load(
    gazebo::physics::ModelPtr parent, 
    sdf::ElementPtr sdf)
{
  // Save pointer to the model
  parent_model_ = parent;
  
  // Save the SDF source
  sdf_ = sdf;
  
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
  deferred_load_thread_ = boost::thread(boost::bind(&GazeboDeployerModelPlugin::loadThread, this));
}

// Load in seperate thread from Gazebo in case something blocks
void GazeboDeployerModelPlugin::loadThread()
{
  RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::loadThread");

  // Error message if the model couldn't be found
  if (!parent_model_) {
    return;
  }

  // Get a pointer to this model's deployer
  boost::shared_ptr<OCL::DeploymentComponent> deployer = deployers[deployer_name_];

  // Check if there is a special gazebo component that should be connected to the world
  if(sdf_->HasElement("component")) 
  {
    RTT::log(RTT::Info) << "Loading Gazebo RTT components..." << RTT::endlog();

    sdf::ElementPtr component_elem = sdf_->GetElement("component");

    while(component_elem && component_elem->GetName() == "component") 
    {
      // Initialize gazebo component
      RTT::TaskContext* new_model_component = NULL;

      if(!component_elem->HasElement("package") ||
         !component_elem->HasElement("type") ||
         !component_elem->HasElement("name"))
      {
        gzerr << "SDF rtt_gazebo plugin <component> tag is missing a required field!" << std::endl;
        return;
      }
      // Get the component name
      RTT::log(RTT::Info) << "Getting gazebo RTT component information..." << RTT::endlog();
      std::string model_component_package = component_elem->GetElement("package")->Get<std::string>();
      std::string model_component_type = component_elem->GetElement("type")->Get<std::string>();
      std::string model_component_name = component_elem->GetElement("name")->Get<std::string>();

      RTT::log(RTT::Info) << "Loading gazebo RTT component package \"" << model_component_package <<"\""<<RTT::endlog();

      // Import the package
      if(!rtt_ros::import(model_component_package)) {
        gzerr << "Could not import rtt_gazebo model component package: \"" << model_component_package << "\"" <<std::endl;
        return;
      }

      // Load the component
      if(!deployer->loadComponent(model_component_name, model_component_type)) {
        gzerr << "Could not load rtt_gazebo model component: \"" << model_component_type << "\"" <<std::endl;
        return;
      }

      // Get the model component from the deployer by name
      if(deployer->hasPeer(model_component_name)) {
        new_model_component = deployer->getPeer(model_component_name);
      } else {
        gzerr << "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""<<model_component_name<<"\", but there is no peer by that name." <<std::endl;
        return;
      }

      // Make sure the component has the required interfaces
      if( new_model_component == NULL ) {
        gzerr << "RTT model component was not properly created." << std::endl; return; }
      if( !new_model_component->provides()->hasService("gazebo") ) {
        gzerr << "RTT model component does not have required \"gazebo\" service." << std::endl; return; }
      if( !new_model_component->provides("gazebo")->hasOperation("configure") ) {
        gzerr << "RTT model component does not have required \"gazebo.configure\" operation." << std::endl; return; }
      if( !new_model_component->provides("gazebo")->hasOperation("update") ) {
        gzerr << "RTT model component does not have required \"gazebo.update\" operation." << std::endl; return; }

      // Configure the component with the parent model
      RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure = 
        new_model_component->provides("gazebo")->getOperation("configure");

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
      GazeboUpdateCaller gazebo_update_caller = new_model_component->provides("gazebo")->getOperation("update");

      if(!gazebo_update_caller.ready()) {
        gzerr <<"RTT model component's \"gazebo.update\" operation could not be connected. Check its signature." << std::endl;
        return;
      }

      model_components_.push_back(new_model_component);
      gazebo_update_callers_.push_back(gazebo_update_caller);

      // Listen to the update event. This event is broadcast every simulation iteration.
      update_connections_.push_back(gazebo::event::Events::ConnectWorldUpdateBegin(
              boost::bind(&GazeboDeployerModelPlugin::gazeboUpdate, this)));

      // Get the next element
      component_elem = component_elem->GetNextElement("component");
    }
  } else {
    RTT::log(RTT::Warning) << "No RTT component defined for Gazebo hooks." << RTT::endlog();
    return;
  }

  if(model_components_.empty()) {
    gzerr << "Could not load any RTT components!" << std::endl;
    return;
  }

  // Get the orocos ops script to run in the deployer
  if(sdf_->HasElement("orocosScript")) 
  {
    sdf::ElementPtr script_elem = sdf_->GetElement("orocosScript");

    while(script_elem && script_elem->GetName() == "orocosScript") 
    {
      if(script_elem->HasElement("filename")) {
        std::string ops_script_file = script_elem->GetElement("filename")->Get<std::string>();
        gzlog << "Running orocos ops script file "<<ops_script_file<<"..." << std::endl;
        if(!deployer->runScript(ops_script_file)) {
          gzerr << "Could not run ops script file "<<ops_script_file<<"!" << std::endl;
          return;
        }
      } else if(script_elem->HasElement("inline")) {
        gzlog << "Running inline orocos ops script..." << std::endl;
        std::string ops_script = script_elem->Get<std::string>();
        if(!deployer->getProvider<RTT::Scripting>("scripting")->eval(ops_script)) {
          gzerr << "Could not run inline ops script!" << std::endl;
          return;
        }
      }

      script_elem = script_elem->GetNextElement("orocosScript");
    }
  }
}

// Called by the world update start event
void GazeboDeployerModelPlugin::gazeboUpdate()
{
  // Call orocos RTT model component gazebo.update() operations
  for(std::vector<GazeboUpdateCaller>::iterator caller = gazebo_update_callers_.begin();
      caller != gazebo_update_callers_.end();
      ++caller)
  {
    (*caller)(parent_model_);
  }
}
