
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <rtt_gazebo_plugin/gazebo_component.h>

class DefaultGazeboComponent : public rtt_gazebo_plugin::GazeboComponent
{
public:

  DefaultGazeboComponent(std::string const& name) : 
    GazeboComponent(name)
  {
    this->ports()->addPort("test", port_test);
    this->addPort("names", port_state_names);
    this->addPort("posvel", port_state_posvel);
    this->addPort("effort", port_state_effort);

    this->provides("joint_command")->addPort("position", port_cmd_position);
    this->provides("joint_command")->addPort("velocity", port_cmd_velocity);
    this->provides("joint_command")->addPort("effort", port_cmd_effort);
  }

  virtual bool configureHook()
  {
    state_posvel_.q.resize(gazebo_joints_.size());
    state_posvel_.qdot.resize(gazebo_joints_.size());
    state_effort_.resize(gazebo_joints_.size());
    return true;
  }

  //! Called from Gazebo
  virtual void gazeboUpdateHook() 
  {
    // Synchronize with update()
    RTT::os::MutexLock lock(gazebo_mutex_);

    port_test.write(gazebo_joints_.size());

    // Write command
    KDL::JntArray cmd;
    if(port_cmd_effort.readNewest(cmd) == RTT::NewData) {
      for(unsigned int j=0; j < gazebo_joints_.size() && j < cmd.rows(); j++) {
        gazebo_joints_[j]->SetForce(0,cmd(j));
      }
    } else if(port_cmd_velocity.readNewest(cmd) == RTT::NewData) {
      for(unsigned int j=0; j < gazebo_joints_.size() && j < cmd.rows(); j++) {
        gazebo_joints_[j]->SetVelocity(0,cmd(j));
      }
    } else if(port_cmd_position.readNewest(cmd) == RTT::NewData) {
      for(unsigned int j=0; j < gazebo_joints_.size() && j < cmd.rows(); j++) {
        gazebo_joints_[j]->SetAngle(0,cmd(j));
      }
    }
    
    // Get state
    for(unsigned j=0; j < gazebo_joints_.size(); j++) {
      state_posvel_.q(j) = gazebo_joints_[j]->GetAngle(0).Radian();
      state_posvel_.qdot(j) = gazebo_joints_[j]->GetVelocity(0);
      state_effort_(j) = gazebo_joints_[j]->GetForce(0u);
    }

    port_state_posvel.write(state_posvel_);
    port_state_effort.write(state_effort_);
  }

  virtual void updateHook()
  {
    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);
  }

protected:

  KDL::JntArrayVel state_posvel_;
  KDL::JntArray state_effort_;

  RTT::OutputPort<double> port_test;

  RTT::OutputPort<std::vector<std::string> > port_state_names;
  RTT::OutputPort<KDL::JntArrayVel> port_state_posvel;
  RTT::OutputPort<KDL::JntArray> port_state_effort;

  RTT::InputPort<KDL::JntArray> port_cmd_position;
  RTT::InputPort<KDL::JntArray> port_cmd_velocity;
  RTT::InputPort<KDL::JntArray> port_cmd_effort;
};

ORO_LIST_COMPONENT_TYPE(DefaultGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
