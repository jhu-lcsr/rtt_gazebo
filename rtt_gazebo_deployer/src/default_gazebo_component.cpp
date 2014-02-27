
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
  enum CommandType {
    EFFORT,
    VELOCITY,
    POSITION
  };

  DefaultGazeboComponent(std::string const& name) : 
    RTT::TaskContext(name),
    kp_(1.0),
    kd_(0.1),
    steps_rtt_(0),
    steps_gz_(0),
    n_joints_(0)
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

    this->provides("debug")->addAttribute("time_wall",wall_time_);
    this->provides("debug")->addAttribute("time_rtt",rtt_time_);
    this->provides("debug")->addAttribute("time_gz",gz_time_);
    this->provides("debug")->addAttribute("period_sim",period_sim_);
    this->provides("debug")->addAttribute("period_wall",period_wall_);
    this->provides("debug")->addAttribute("steps_rtt",steps_rtt_);
    this->provides("debug")->addAttribute("steps_gz",steps_gz_);
    this->provides("debug")->addAttribute("n_joints",n_joints_);
    this->provides("debug")->addAttribute("joint_pos",state_pos_);
    this->provides("debug")->addAttribute("joint_command",command_);

    this->addProperty("kp",kp_);
    this->addProperty("kd",kd_);
  }

  //! Called from gazebo
  virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
  {
    if(model.get() == NULL) {return false;}

    // Get the joints
    gazebo_joints_ = model->GetJoints();
    n_joints_ = gazebo_joints_.size();

    // Get the joint names
    for(std::vector<gazebo::physics::JointPtr>::iterator it=gazebo_joints_.begin();
        it != gazebo_joints_.end();
        ++it)
    {
      joint_names_.push_back((**it).GetName());
    }

    state_posvel_.q.resize(n_joints_);
    state_posvel_.qdot.resize(n_joints_);
    state_effort_.resize(n_joints_);
    command_.resize(n_joints_);
    state_pos_.resize(n_joints_);

    return true;
  }

  //! Called from Gazebo
  virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model) 
  {
    if(model.get() == NULL) {return;}

    // Synchronize with update()
    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(trylock.isSuccessful()) {

      // Increment simulation step counter (debugging)
      steps_gz_++;

      // Get the RTT and gazebo time for debugging purposes
      rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
      gazebo::common::Time gz_time = model->GetWorld()->GetSimTime();
      gz_time_ = (double)gz_time.sec + ((double)gz_time.nsec)*1E-9;

      // Get the wall time
      gazebo::common::Time gz_wall_time = gazebo::common::Time::GetWallTime();
      wall_time_ = (double)gz_wall_time.sec + ((double)gz_wall_time.nsec)*1E-9;

      // Get state
      for(unsigned j=0; j < gazebo_joints_.size(); j++) {
        state_pos_[j] = gazebo_joints_[j]->GetAngle(0).Radian();
        state_posvel_.q(j) =  state_pos_[j];
        state_posvel_.qdot(j) = gazebo_joints_[j]->GetVelocity(0);
        state_effort_(j) = gazebo_joints_[j]->GetForce(0u);
      }

      // Write command
      switch(command_type_) {
        case EFFORT:
          for(unsigned int j=0; j < gazebo_joints_.size() && j < command_.size(); j++)
            gazebo_joints_[j]->SetForce(0,command_[j]);
          break;
        case VELOCITY:
          for(unsigned int j=0; j < gazebo_joints_.size() && j < command_.size(); j++)
            gazebo_joints_[j]->SetVelocity(0,command_[j]);
          break;
        case POSITION:
          for(unsigned int j=0; j < gazebo_joints_.size() && j < command_.size(); j++)
            gazebo_joints_[j]->SetAngle(0,command_[j]);
          break;
      };
    }
    
  }


  virtual bool configureHook()
  {
    return true;
  }

  virtual void updateHook()
  {
    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);

    // Compute period in simulation clock
    static double last_update_time_sim;
    period_sim_ = rtt_time_ - last_update_time_sim;
    last_update_time_sim = rtt_time_;

    // Compute period in wall clock
    static double last_update_time_wall;
    period_wall_ = wall_time_ - last_update_time_wall;
    last_update_time_wall = wall_time_;

    // Increment simulation step counter (debugging)
    steps_rtt_++;

    // Get command from ports
    KDL::JntArray cmd;
    if(port_cmd_effort.readNewest(cmd) == RTT::NewData) {
      command_type_ = EFFORT;
    } else if(port_cmd_velocity.readNewest(cmd) == RTT::NewData) {
      command_type_ = VELOCITY;
    } else if(port_cmd_position.readNewest(cmd) == RTT::NewData) {
      command_type_ = POSITION;
    }

    // Set the command from the input port or from simple PD control
    if(cmd.rows() > 0) {
      for(unsigned int j=0; j < gazebo_joints_.size() && j < cmd.rows(); j++) {
        command_[j] = cmd(j);
      }
    } else {
      command_type_ = EFFORT;
      for(unsigned int j=0; j < gazebo_joints_.size(); j++) {
        command_[j] = -kp_*state_pos_[j] - kd_*state_pos_[j];
      }
    }

    // Write state to ports
    port_state_posvel.write(state_posvel_);
    port_state_effort.write(state_effort_);
  }

protected:

  //! Synchronization
  RTT::os::MutexRecursive gazebo_mutex_;
  
  //! The Gazebo Model
  //! The gazebo
  std::vector<gazebo::physics::JointPtr> gazebo_joints_;
  std::vector<std::string> joint_names_;

  KDL::JntArrayVel state_posvel_;
  KDL::JntArray state_effort_;
  std::vector<double> state_pos_;
  std::vector<double> command_;

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
  double wall_time_;

  //! Control gains
  double kp_,kd_;

  int steps_gz_;
  int steps_rtt_;
  int n_joints_;

  double period_sim_;
  double period_wall_;

  CommandType command_type_;
};

ORO_LIST_COMPONENT_TYPE(DefaultGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
