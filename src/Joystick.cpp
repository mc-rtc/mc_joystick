#include "Joystick.h"

#include "updaters/ROSUpdater.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/ros.h>

#include <cstring>

namespace mc_plugin
{

struct JoystickUpdater
{
  JoystickUpdater(const std::string & name, const mc_rtc::Configuration & profile) : name_(name)
  {
    std::string type = profile("type", std::string(""));
    if(type == "ROS")
    {
      auto updater = std::make_shared<ROSUpdater>(name, profile);
      do_update_ = [updater = std::move(updater)](mc_joystick::State & state) { updater->update(state); };
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
    store.make<const mc_joystick::State *>(name_ + "::state", &state_);
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  void update()
  {
    do_update_(state_);
  }

private:
  std::string name_;
  mc_joystick::State state_;
  std::function<void(mc_joystick::State &)> do_update_;
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
  for(auto & joy : joysticks_)
  {
    joy->update();
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
