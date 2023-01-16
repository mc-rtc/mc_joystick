#include "ROSUpdater.h"

namespace mc_plugin
{

namespace
{

struct ROSButtonMapping
{
  size_t index;
  mc_joystick::Button button;
};

struct ROSAxisMapping
{
  size_t index;
  mc_joystick::Axis axis;
  double dir;
};

struct ROSAxisToButtonMapping
{
  size_t index;
  std::array<mc_joystick::Button, 2> buttons;
};

static std::vector<ROSButtonMapping> get_buttons_mapping(const std::map<std::string, mc_joystick::Button> & map)
{
  std::vector<ROSButtonMapping> out;
  out.reserve(map.size());
  for(const auto & m : map)
  {
    out.push_back({std::stoul(m.first), m.second});
  }
  return out;
}

static void get_axes_mapping(const std::map<std::string, mc_rtc::Configuration> & map,
                             const std::vector<mc_joystick::Axis> & inverted,
                             std::vector<ROSAxisMapping> & axis_mapping,
                             std::vector<ROSAxisToButtonMapping> & axis_to_button_mapping)
{
  axis_mapping.reserve(map.size());
  for(const auto & m : map)
  {
    if(m.second.size())
    {
      axis_to_button_mapping.push_back({std::stoul(m.first), m.second});
    }
    else
    {
      double dir = 1.0;
      mc_joystick::Axis axis = m.second;
      if(std::find(inverted.begin(), inverted.end(), axis) != inverted.end())
      {
        dir = -1.0;
      }
      axis_mapping.push_back({std::stoul(m.first), axis, dir});
    }
  }
}

} // namespace

ROSUpdater::ROSUpdater(const std::string & name, const mc_rtc::Configuration & profile)
{
  if(!nh_.ok())
  {
    mc_rtc::log::critical("[Joystick] {} should be connected with ROS but ROS is not available", name);
    return;
  }
  nh_.setCallbackQueue(&queue_);
  std::string topic = profile("topic", std::string{"/joy"});
  auto buttons_mapping = get_buttons_mapping(profile("ros_buttons"));
  std::vector<ROSAxisMapping> axes_mapping;
  std::vector<ROSAxisToButtonMapping> axes_buttons_mapping;
  get_axes_mapping(profile("ros_axes"), profile("inverted_axes"), axes_mapping, axes_buttons_mapping);
  sub_ = nh_.subscribe<sensor_msgs::Joy>(
      topic, 100,
      [this, buttons_mapping, axes_mapping, axes_buttons_mapping](const sensor_msgs::JoyConstPtr & msg_ptr) {
        const auto & msg = *msg_ptr;
        for(const auto & bm : buttons_mapping)
        {
          if(bm.index >= msg.buttons.size())
          {
            continue;
          }
          state_.buttons[static_cast<size_t>(bm.button)] = msg.buttons[bm.index];
        }
        for(const auto & am : axes_mapping)
        {
          if(am.index >= msg.axes.size())
          {
            continue;
          }
          state_.axes[static_cast<size_t>(am.axis)] = am.dir * msg.axes[am.index];
        }
        for(const auto & am : axes_buttons_mapping)
        {
          if(am.index >= msg.axes.size())
          {
            continue;
          }
          state_.buttons[static_cast<size_t>(am.buttons[0])] = msg.axes[am.index] == -1.0;
          state_.buttons[static_cast<size_t>(am.buttons[1])] = msg.axes[am.index] == 1.0;
        }
      });
}

} // namespace mc_plugin
