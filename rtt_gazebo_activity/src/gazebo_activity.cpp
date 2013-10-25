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

#include <rtt_gazebo_activity/gazebo_activity.hpp>

#include <rtt/base/RunnableInterface.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <boost/weak_ptr.hpp>

namespace rtt_gazebo_activity {

using namespace RTT::base;

boost::weak_ptr<GazeboActivityManager> GazeboActivityManager::sinstance;

boost::shared_ptr<GazeboActivityManager> GazeboActivityManager::GetInstance()
{
    return sinstance.lock();
}

boost::shared_ptr<GazeboActivityManager> GazeboActivityManager::Instance()
{
    // Create a new instance, if necessary
    boost::shared_ptr<GazeboActivityManager> shared = sinstance.lock();
    if(sinstance.expired()) {
        shared.reset(new GazeboActivityManager());
        sinstance = shared;
    }

    return shared;
}

GazeboActivityManager::~GazeboActivityManager()
{
}

RTT::Seconds GazeboActivityManager::getSimulationPeriod() const
{
    return msimulation_period;
}

void GazeboActivityManager::setSimulationPeriod(RTT::Seconds s)
{
    msimulation_period = s;
}

void GazeboActivityManager::update()
{
    RTT::os::MutexLock lock(mmutex);
    RTT::os::TimeService::ticks now = RTT::os::TimeService::Instance()->getTicks();

    for(std::list<GazeboActivity *>::const_iterator it = mactivities.begin(); it != mactivities.end(); it++)
    {
        GazeboActivity *activity = *it;
        if (RTT::os::TimeService::ticks2nsecs(now - activity->getLastExecutionTicks()) * 1e9 >= activity->getPeriod())
        {
            if (activity->execute())
                activity->setLastExecutionTicks(now);
        }
    }
}

void GazeboActivityManager::add(GazeboActivity *activity)
{
    RTT::os::MutexLock lock(mmutex);
    std::list<GazeboActivity *>::iterator it = std::find(mactivities.begin(), mactivities.end(), activity);
    if (it == mactivities.end()) {
        mactivities.push_back(activity);
    }
}

void GazeboActivityManager::remove(GazeboActivity *activity)
{
    RTT::os::MutexLock lock(mmutex);
    std::list<GazeboActivity *>::iterator it = std::find(mactivities.begin(), mactivities.end(), activity);
    if (it != mactivities.end()) {
        mactivities.erase(it);
    }
}

GazeboActivity::GazeboActivity(RunnableInterface* run, const std::string& name)
    : ActivityInterface(run), mname(name), mrunning(false), mactive(false), mmanager(GazeboActivityManager::Instance())
{
    mmanager->add(this);
}

GazeboActivity::GazeboActivity(RTT::Seconds period, RunnableInterface* run, const std::string& name)
    : ActivityInterface(run), mname(name), mperiod(period), mrunning(false), mactive(false), mmanager(GazeboActivityManager::Instance())
{
    mmanager->add(this);
}

GazeboActivity::~GazeboActivity()
{
    stop();
    mmanager->remove(this);
}

RTT::Seconds GazeboActivity::getPeriod() const
{
    if (mperiod > 0.0)
        return mperiod;
    else
        return mmanager->getSimulationPeriod();
}

bool GazeboActivity::setPeriod(RTT::Seconds s)
{
    mperiod = s;
    return true;
}

unsigned GazeboActivity::getCpuAffinity() const
{
    return ~0;
}

bool GazeboActivity::setCpuAffinity(unsigned cpu)
{
    return false;
}

RTT::os::ThreadInterface* GazeboActivity::thread()
{
    return 0;
}

bool GazeboActivity::initialize()
{
    return true;
}

void GazeboActivity::step()
{
}

void GazeboActivity::loop()
{
    this->step();
}

bool GazeboActivity::breakLoop()
{
    return false;
}


void GazeboActivity::finalize()
{
}

bool GazeboActivity::start()
{
    if ( mactive == true )
    {
        RTT::log(RTT::Error) << "Unable to start slave as it is already started" << RTT::endlog();
        return false;
    }

    mactive = true;

    if ( runner ? runner->initialize() : this->initialize() ) {
        mrunning = true;
    } else {
        mactive = false;
    }

    return mactive;
}

bool GazeboActivity::stop()
{
    if ( !mactive )
        return false;

    mrunning = false;
    if (runner)
        runner->finalize();
    else
        this->finalize();
    mactive = false;
    return true;
}

bool GazeboActivity::isRunning() const
{
    return mrunning;
}

bool GazeboActivity::isPeriodic() const
{
    return true;
}

bool GazeboActivity::isActive() const
{
    return mactive;
}

bool GazeboActivity::trigger()
{
    return false;
}

bool GazeboActivity::execute()
{
    if (!mrunning) return false;
    if (runner) runner->step(); else this->step();
    return true;
}

RTT::os::TimeService::ticks GazeboActivity::getLastExecutionTicks() const
{
    return mlast;
}

void GazeboActivity::setLastExecutionTicks(RTT::os::TimeService::ticks ticks)
{
    mlast = ticks;
}

} // namespace rtt_gazebo_activity
