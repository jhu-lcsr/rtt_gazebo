#ifndef __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT
#define __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

namespace rtt_gazebo_plugin {

  class GazeboComponent : public RTT::TaskContext {
  public:

    GazeboComponent(std::string const& name) : 
      RTT::TaskContext(name)
    {
      this->provides("gazebo")->addOperation("setModel",&GazeboComponent::gazeboSetModel,this,RTT::ClientThread);
      this->provides("gazebo")->addOperation("getModel",&GazeboComponent::gazeboGetModel,this,RTT::ClientThread);
      this->provides("gazebo")->addOperation("update",&GazeboComponent::gazeboUpdate,this,RTT::ClientThread);
      this->provides("gazebo")->provides("debug")->addProperty("time_rtt",rtt_time_);
      this->provides("gazebo")->provides("debug")->addProperty("time_gz",gz_time_);
    }

    //! Set the Gazebo model
    bool gazeboSetModel(gazebo::physics::ModelPtr model) 
    {
      // Store the model
      if(model.get() == NULL) { return false; }
      model_ = model;

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

    //! Get the Gazebo model
    gazebo::physics::ModelPtr gazeboGetModel() 
    {
      return model_;
    }

    //! Called from Gazebo
    void gazeboUpdate() 
    {
      // Synchronize with update()
      RTT::os::MutexLock lock(gazebo_mutex_);
      rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());

      gazebo::common::Time gz_time = model_->GetWorld()->GetSimTime();
      gz_time_ = (double)gz_time.sec + ((double)gz_time.nsec)*1E-9;

      if(this->getTaskState() == RTT::TaskContext::Running) {
        this->gazeboUpdateHook();
      }
    }

    //! Implemented by subclasses
    virtual void gazeboUpdateHook() = 0;

  protected:

    double rtt_time_;
    double gz_time_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    //! The Gazebo Model
    gazebo::physics::ModelPtr model_;
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;
    std::vector<std::string> joint_names_;
  };

}

#endif // ifndef __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT
