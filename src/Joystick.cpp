#include "Joystick.h"

#include "JoystickState.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/ros.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

#include <cstring>

namespace mc_plugin
{

static const mc_joystick::State & get_default_joystick_state()
{
  static mc_joystick::State default_state = []() {
    mc_joystick::State state;
    for(auto & b : state.buttons)
    {
      b = false;
    }
    for(auto & a : state.axes)
    {
      a = 0.0;
    }
    state.axes[static_cast<size_t>(mc_joystick::Axis::L2)] = 1.0;
    state.axes[static_cast<size_t>(mc_joystick::Axis::R2)] = 1.0;
    return state;
  }();
  return default_state;
}

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

} // namespace

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

struct JoystickUpdater
{
  JoystickUpdater(const std::string & name, const mc_rtc::Configuration & profile)
  : name_(name), update_state_(get_default_joystick_state()), publish_state_(get_default_joystick_state())
  {
    std::string type = profile("type", std::string(""));
    if(type == "ROS")
    {
      auto nh = mc_rtc::ROSBridge::get_node_handle();
      if(!nh)
      {
        mc_rtc::log::error("[Joystick] {} should be connected with ROS but ROS is not available", name);
        return;
      }
      std::string topic = profile("topic", std::string{"/joy"});
      auto buttons_mapping = get_buttons_mapping(profile("ros_buttons"));
      std::vector<ROSAxisMapping> axes_mapping;
      std::vector<ROSAxisToButtonMapping> axes_buttons_mapping;
      get_axes_mapping(profile("ros_axes"), profile("inverted_axes"), axes_mapping, axes_buttons_mapping);
      sub_ = nh->subscribe<sensor_msgs::Joy>(
          topic, 100,
          [this, buttons_mapping, axes_mapping, axes_buttons_mapping](const sensor_msgs::JoyConstPtr & msg_ptr) {
            const auto & msg = *msg_ptr;
            for(const auto & bm : buttons_mapping)
            {
              if(bm.index >= msg.buttons.size())
              {
                continue;
              }
              update_state_.buttons[static_cast<size_t>(bm.button)] = msg.buttons[bm.index];
            }
            for(const auto & am : axes_mapping)
            {
              if(am.index >= msg.axes.size())
              {
                continue;
              }
              update_state_.axes[static_cast<size_t>(am.axis)] = am.dir * msg.axes[am.index];
            }
            for(const auto & am : axes_buttons_mapping)
            {
              if(am.index >= msg.axes.size())
              {
                continue;
              }
              update_state_.buttons[static_cast<size_t>(am.buttons[0])] = msg.axes[am.index] == -1.0;
              update_state_.buttons[static_cast<size_t>(am.buttons[1])] = msg.axes[am.index] == 1.0;
            }
            publish_state_ = update_state_;
          });
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[Joystick] Profile {} is using the unknown type specifier: \"{}\"", name, type);
    }
  }

  JoystickUpdater(const JoystickUpdater &) = delete;
  JoystickUpdater & operator=(const JoystickUpdater &) = delete;
  JoystickUpdater(JoystickUpdater &&) = delete;
  JoystickUpdater & operator=(JoystickUpdater &&) = delete;

  void add_to_datastore(mc_rtc::DataStore & store)
  {
    store.make<const mc_joystick::State *>(name_ + "::state", &publish_state_);
  }

  void run_callbacks() {}

  inline const std::string & name() const noexcept
  {
    return name_;
  }

private:
  std::string name_;
  mc_joystick::State update_state_;
  mc_joystick::State publish_state_;
  ros::Subscriber sub_;
};

Joystick::~Joystick() = default;

static mc_rtc::Configuration get_profile(const mc_rtc::Configuration & config, const std::string & profile)
{
  if(!config.has(profile))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[Joystick] Unknown profile requested {}", profile);
  }
  auto profile_c = config(profile);
  if(profile_c.has("base"))
  {
    auto base = get_profile(config, profile_c("base"));
    base.load(profile_c);
    return base;
  }
  else
  {
    return profile_c;
  }
}

void Joystick::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto profiles = config("profiles", mc_rtc::Configuration{});
  auto connect = config("connect", std::vector<std::string>{});
  for(const auto & p : connect)
  {
    auto joy = std::unique_ptr<JoystickUpdater>(new JoystickUpdater(p, get_profile(profiles, p)));
    joysticks_.push_back(std::move(joy));
  }
  reset(controller);
}

void Joystick::reset(mc_control::MCGlobalController & gc)
{
  std::vector<std::string> connected;
  for(auto & joy : joysticks_)
  {
    connected.push_back(joy->name());
    joy->add_to_datastore(gc.controller().datastore());
  }
  gc.controller().datastore().make<decltype(connected)>("Joystick::connected", connected);
}

void Joystick::before(mc_control::MCGlobalController &)
{
  ros::spinOnce();
  for(auto & joy : joysticks_)
  {
    joy->run_callbacks();
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration Joystick::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("Joystick", mc_plugin::Joystick)
