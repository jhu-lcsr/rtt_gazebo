/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Meyer, TU Darmstadt
 *  Copyright (c) 2013, Intermodalics BVBA
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
 *   * Neither the name of TU Darmstadt and Intermodalics BVBA
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

#include <rtt_gazebo_activity/gazebo_service.hpp>
#include <rtt_gazebo_activity/gazebo_activity.hpp>

#include <rtt/TaskContext.hpp>

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

namespace rtt_gazebo_activity {

GazeboService::GazeboService(const std::string& name, RTT::TaskContext* owner)
    : RTT::Service(name, owner)
    , RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0, name)
    , mtime_service(RTT::os::TimeService::Instance())
{
    this->addOperation("start", &GazeboService::start, this);
    this->addOperation("stop", &GazeboService::stop, this);

    this->addOperation("setGazeboActivity", &GazeboService::setGazeboActivity, this);
    setGazeboActivity();
}

GazeboService::~GazeboService()
{
    stop();
}

void GazeboService::clockCallback(const rosgraph_msgs::ClockConstPtr& clock)
{
    // Get the simulation time
    using namespace RTT::os;
    TimeService::Seconds clock_secs = (TimeService::Seconds)clock->clock.sec + ((TimeService::Seconds)clock->clock.nsec)*1E-9;

    // Update the RTT clock
    updateClock(clock_secs);
}

void GazeboService::updateClock(RTT::os::TimeService::Seconds clock_secs)
{
    // Update the RTT time to match the gazebo time
    using namespace RTT::os;
    TimeService::ticks rtt_ticks = mtime_service->getTicks();
    TimeService::Seconds rtt_secs = RTT::nsecs_to_Seconds(TimeService::ticks2nsecs(rtt_ticks));

    // Compute the time update
    TimeService::Seconds dt = clock_secs - rtt_secs;

    // Check if time went backwards
    if(dt < 0) {
        RTT::log(RTT::Warning) << "Time went backwards by " << dt << " seconds!" << RTT::endlog();
    }

    // Update the RTT clock
    // TODO: Interpolate over a higher-resolution range?
    mtime_service->secondsChange(dt);

    // trigger all GazeboActivities
    boost::shared_ptr<GazeboActivityManager> manager = GazeboActivityManager::GetInstance();
    if (manager) {
        manager->setSimulationPeriod(dt);
        manager->update();
    }
}

RTT::os::ThreadInterface* GazeboService::thread()
{
    return this;
}

void GazeboService::loop()
{
    static const ros::WallDuration timeout(0.1);
    while(!mbreak_loop)
        mcallback_queue.callAvailable(timeout);
}

bool GazeboService::breakLoop()
{
    mbreak_loop = true;
    return true;
}

bool GazeboService::initialize()
{
    if (getOwner()) {
        RTT::log(RTT::Error) << "The gazebo service thread can only be started in the global gazebo service." << RTT::endlog();
        return false;
    }

    // Disable the RTT system clock so Gazebo can manipulate time
    mtime_service->enableSystemClock(false);

    // Subscribe the /clock topic (simulation time, e.g. published by Gazebo)
    ros::NodeHandle nh;
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<rosgraph_msgs::Clock>(
                "/clock", 1, boost::bind(&GazeboService::clockCallback, this, _1),
                ros::VoidConstPtr(), &mcallback_queue);
    msubscriber = nh.subscribe(ops);

    RTT::log(RTT::Info) << "Using gazebo time on topic " << msubscriber.getTopic() << "." << RTT::endlog();
    mbreak_loop = false;
    return true;
}

void GazeboService::finalize()
{
    msubscriber = ros::Subscriber();
}

bool GazeboService::setGazeboActivity(RTT::TaskContext *t)
{
    if (!t) t = getOwner();
    if (!t) return false;
    return t->setActivity(new GazeboActivity(t->getPeriod()));
}

} // namespace rtt_gazebo_activity


#include <rtt/plugin/Plugin.hpp>
#include <rtt/internal/GlobalService.hpp>

/**
 * Instructs this plugin to load itself into the process or a component.
 * This function will first be called with \a t being equal to zero, giving
 * the plugin the opportunity to load something in the whole process.
 * Implement in this function any startup code your plugin requires.
 * This function should not throw.
 *
 * @param t The optional TaskContext which is loading this plugin.
 * Is zero when the plugin is loaded into the process, non-zero
 * when loaded in a TaskContext. A plugin may choose to load only
 * in the process and not in a TaskContext (typekits for example). If
 * a plugin only wants to load in a TaskContext, it must return true when
 * t is zero, such that the plugin remains loaded in the process.
 * @return true if the loading succeeded, false otherwise.
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    boost::shared_ptr<rtt_gazebo_activity::GazeboService> service(new rtt_gazebo_activity::GazeboService("gazebo", t));
    if (t == 0) {
        RTT::internal::GlobalService::Instance()->addService(service);
    } else {
        t->provides()->addService(service);
    }
    return true;
}

/**
 * Return the unique name of this plugin. No two plugins with
 * the same name will be allowed to live in a single process.
 */
std::string getRTTPluginName()
{
    return "gazebo";
}

/**
 * Returns the target name for which this plugin was built.
 *
 * @return The name as set by OROCOS_TARGET. When the empty
 * string is returned, it is assumed that the loadRTTPlugin
 * function will check if this plugin may be loaded or not.
 */
std::string getRTTTargetName()
{
    return OROCOS_TARGET_NAME;
}
