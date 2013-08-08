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
      if(this->getTaskState() == RTT::TaskContext::Running) {
        this->gazeboUpdateHook();
      }
    }

    //! Implemented by subclasses
    virtual void gazeboUpdateHook() = 0;

  protected:

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    //! The Gazebo Model
    gazebo::physics::ModelPtr model_;
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;
    std::vector<std::string> joint_names_;
  };

}

#endif // ifndef __RTT_GAZEBO_PLUGIN_GAZEBO_COMPONENT
