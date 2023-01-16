#pragma once

#include "../JoystickState.h"

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

namespace mc_plugin
{

struct ROSUpdater
{
  ROSUpdater(const std::string & name, const mc_rtc::Configuration & profile);

  inline void update(mc_joystick::State & state)
  {
    if(nh_.ok())
    {
      queue_.callAvailable();
      state = state_;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  mc_joystick::State state_;
};

} // namespace mc_plugin
