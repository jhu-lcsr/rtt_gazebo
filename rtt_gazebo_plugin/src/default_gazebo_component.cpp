
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

class DefaultGazeboComponent : public RTT::TaskContext
{
public:

  DefaultGazeboComponent(std::string const& name) : 
    RTT::TaskContext(name)
  {
    // Add required gazebo interfaces
    this->provides("gazebo")->addOperation("configure",&DefaultGazeboComponent::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&DefaultGazeboComponent::gazeboUpdateHook,this,RTT::ClientThread);

    this->provides("joint_state")->addPort("names", port_state_names);
    this->provides("joint_state")->addPort("posvel", port_state_posvel);
    this->provides("joint_state")->addPort("effort", port_state_effort);

    this->provides("joint_command")->addPort("position", port_cmd_position);
    this->provides("joint_command")->addPort("velocity", port_cmd_velocity);
    this->provides("joint_command")->addPort("effort", port_cmd_effort);

    this->provides("debug")->addProperty("time_rtt",rtt_time_);
    this->provides("debug")->addProperty("time_gz",gz_time_);
  }

  //! Called from gazebo
  virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
  {
    // Get the joints
    gazebo_joints_ = model_->GetJoints();

    // Get the joint names
    for(std::vector<gazebo::physics::JointPtr>::iterator it=gazebo_joints_.begin();
        it != gazebo_joints_.end();
        ++it)
    {
      joint_names_.push_back((**it).GetName());
    }

    return true;
  }

  //! Called from Gazebo
  virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model) 
  {
    // Synchronize with update()
    RTT::os::MutexLock lock(gazebo_mutex_);

    // Get the RTT and gazebo time for debugging purposes
    rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
    gazebo::common::Time gz_time = model_->GetWorld()->GetSimTime();
    gz_time_ = (double)gz_time.sec + ((double)gz_time.nsec)*1E-9;

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


  virtual bool configureHook()
  {
    if(model_.get() == NULL) {
      return false;
    }

    state_posvel_.q.resize(gazebo_joints_.size());
    state_posvel_.qdot.resize(gazebo_joints_.size());
    state_effort_.resize(gazebo_joints_.size());
    return true;
  }

  virtual void updateHook()
  {
    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);
  }

protected:

  //! Synchronization
  RTT::os::MutexRecursive gazebo_mutex_;
  
  //! The Gazebo Model
  gazebo::physics::ModelPtr model_;
  //! The gazebo
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;
  std::vector<std::string> joint_names_;

  KDL::JntArrayVel state_posvel_;
  KDL::JntArray state_effort_;

  RTT::OutputPort<std::vector<std::string> > port_state_names;
  RTT::OutputPort<KDL::JntArrayVel> port_state_posvel;
  RTT::OutputPort<KDL::JntArray> port_state_effort;

  RTT::InputPort<KDL::JntArray> port_cmd_position;
  RTT::InputPort<KDL::JntArray> port_cmd_velocity;
  RTT::InputPort<KDL::JntArray> port_cmd_effort;

  //! RTT time for debugging
  double rtt_time_;
  //! Gazebo time for debugging
  double gz_time_;
};

ORO_LIST_COMPONENT_TYPE(DefaultGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
