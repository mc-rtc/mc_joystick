/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

namespace mc_plugin
{

struct JoystickUpdater;
using JoystickUpdaterPtr = std::unique_ptr<JoystickUpdater>;

struct Joystick : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController &) override {}

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~Joystick() override;

private:
  std::vector<JoystickUpdaterPtr> joysticks_;
};

} // namespace mc_plugin
