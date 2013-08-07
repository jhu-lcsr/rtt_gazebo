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
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


class RTTPlugin : public gazebo::ModelPlugin
{
public:

  virtual ~RosControlPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)

  // Load in seperate thread from Gazebo in case ROS is blocking
  void loadThread()

  // Called by the world update start event
  void Update()

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const

  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF()

private:
  // deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  // Node Handles
  ros::NodeHandle nh_; // no namespace
  ros::NodeHandle model_nh_; // namespaces to robot name

  std::string robot_namespace_;
  std::string robot_description_;

  // Gazebo structures
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;
  gazebo::event::ConnectionPtr update_connection_;

  // Orocos Structures
  static std::vector<OCL::DeploymentComponent> dployers;

  RTT::TaskContext* gazebo_component_;
  boost::scoped_ptr<RTT::TaskContext> gazebo_component_collector_;

  RTT::OperationCaller<void(void)> gazebo_update_; = 
    gazebo_component_->provides("gazebo")->getOperation("update");

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RTTPlugin);

RTTPlugin::~RosControlPlugin()
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

// Overloaded Gazebo entry point
void RTTPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  ROS_INFO_STREAM("Loading ros_control_gazebo_plugin...");
  
  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  // Disable the RTT system clock so Gazebo can manipulate time
  RTT::os::TimeService::Instance()->enableSystemClock(false);

  // Check if this deployer should have a custom name
  if(sdf_->HasElement("deployer")) {
    deployer_name_ = sdf_->GetElement("deployer")->Get<std::string>();
  } else {
    deployer_name_ = "deployer"; 
  }

  // Create deployer if necessary
  if(deployers.find(deployer_name_) == deployers.end()) {
    deployers[deployer_name_] = OCL::DeploymentComponent(deployer_name_);

    // Setup TaskContext server
    if(CORBA::is_nil(RTT::TaskContextServer::orb)) {
      // Initialize orb
      RTT::TaskContextServer::InitOrb(argc, argv);
      // Propcess orb requests in a thread
      RTT::TaskContext::ThreadOrb();
    }

    // Attach the taskcontext server to this component
    TaskContextServer * tc_server = RTT::TaskContextServer::Create( &deployers[deployer_name_] );
  }

  // ros callback queue for processing subscription
  deferred_load_thread_ = boost::thread(boost::bind(&RosControlPlugin::loadThread, this));
}

// Load in seperate thread from Gazebo in case ROS is blocking
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
    if(!deployers[deployer_name_].runScript(opsScriptFile)) {
      return;
    }
  }
  
  // Get the orocos ops script
  if(sdf_->HasElement("opsScript")) {
    std::string ops_script;
    ops_script = sdf_->GetElement("opsScript")->Get<std::string>();
    if(!deployers[deployer_name_].getProvider<Scripting>("scripting")->eval(ops_script)) {
      return;
    }
  }

  // Initialize gazebo component
  gazebo_component_ = NULL;

  // Check if there is a special gazebo component
  if(sdf_->HasElement("component")) {
    std::string gazebo_component_name = sdf_->GetElement("component")->Get<std::string>();

    // Get the gazebo component from the deployer
    if(deployers[deployer_name_].hasPeer(gazebo_component_name)) {
      gazebo_component_ = deployers[deployer_name_].getPeer(gazebo_component_name);
    } else {
      return;
    }

  } else {
    // Create a gazebo component with the same name as the model
    gazebo_component_ = new rtt_gazebo_plugin::DefaultGazeboComponent(parent->getName());
    // Store a scoped ptr to clean this up later
    gazebo_component_collector_.reset(gazebo_component_);
    // Connect the gazebo component to the deployer
    gazebo_component_->connectPeers(&deployers[deployer_name_]);
  }

  // Make sure the component has the required interfaces
  if( !gazebo_component_->provides()->hasService("gazebo") ||
      !gazebo_component_->provides("gazebo")->hasOperation("setModel") ||
      !gazebo_component_->provides("gazebo")->hasOperation("getModel") ||
      !gazebo_component_->provides("gazebo")->hasOperation("update") ||
      ) 
  {
    return;
  }

  // Set the component's gazebo model
  RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_set_model = 
    gazebo_component_->provides("gazebo")->getOperation("set<odel");

  gazebo_set_model(parent_model_);

  // Get gazebo update function
  gazebo_update_ = 
    gazebo_component_->provides("gazebo")->getOperation("update");

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
  TimeService::Seconds dt = std::max(0.0,TimeService::Seconds((gz_time_now.sec*1E6 + gz_time_now.nsec) - rtt_time->getNSsecs()));
  rtt_time->secondsChange(dt);

  // Call orocos gazebo component gazeboUpdate()
  gazebo_update_();
}
