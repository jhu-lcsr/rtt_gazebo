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

#ifndef RTT_GAZEBO_ACTIVITY_CLOCK_SERVICE_HPP
#define RTT_GAZEBO_ACTIVITY_CLOCK_SERVICE_HPP

#include <rtt/Service.hpp>
#include <rtt/os/Thread.hpp>
#include <rtt/os/TimeService.hpp>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <rosgraph_msgs/Clock.h>

namespace rtt_gazebo_activity {

class GazeboService : public RTT::Service, public RTT::os::Thread
{
public:
    GazeboService(const std::string& name, RTT::TaskContext* owner = 0);
    virtual ~GazeboService();

    virtual void updateClock(RTT::os::TimeService::Seconds clock_secs);
    virtual RTT::os::ThreadInterface* thread();

    virtual bool setGazeboActivity(RTT::TaskContext *t = 0);

protected:
    virtual void clockCallback(const rosgraph_msgs::ClockConstPtr& clock);

    virtual void loop();
    virtual bool breakLoop();
    virtual bool initialize();
    virtual void finalize();

private:
    RTT::os::TimeService *mtime_service;

    ros::Subscriber msubscriber;
    ros::CallbackQueue mcallback_queue;

    bool mbreak_loop;
};

} // namespace rtt_gazebo_activity

#endif // RTT_GAZEBO_ACTIVITY_CLOCK_SERVICE_HPP
