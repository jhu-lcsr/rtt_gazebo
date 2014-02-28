#ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
#define __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H

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

#include <rtt_rosclock/rtt_rosclock.h>

namespace rtt_gazebo_deployer {

  class RTTSystem
  {
  public:
    //! Get or create an instance of the singleton
    static boost::shared_ptr<RTTSystem> Instance();
    //! Get an instance of the singleton or NULL if not created
    static boost::shared_ptr<RTTSystem> GetInstance();

    //! Set the gazebo world (as a time/trigger source)
    void connectWorld(gazebo::physics::WorldPtr world);

    /**
     * \brief Update the RTT clock from the gazebo clock
     *
     * This queries the Gazebo time, and then uses the rtt_rosclock Orocos
     * plugin to both set the RTT::os::TimeService time and trigger any
     * periodic sim clock components using SimClockActivity.
     */
    void updateClock();

    //! Disconnect the world event and cleanup CORBA
    ~RTTSystem();

  private:
    //! Cache the singleton with a weak pointer
    static boost::weak_ptr<RTTSystem> singleton;

    //! Initialize RTT (__os_init), rtt_rosclock sim clock, and CORBA
    RTTSystem();

    //! Singleton constraints
    RTTSystem(RTTSystem const&);
    void operator=(RTTSystem const&);

    //! Gazebo world to get time from
    gazebo::physics::WorldPtr world_;

    //! Event connection to the world update
    gazebo::event::ConnectionPtr update_connection_;
  };

}

#endif // ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
