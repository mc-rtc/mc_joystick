#include "JoystickState.h"

namespace mc_joystick
{

State::State()
{
  for(auto & b : buttons)
  {
    b = false;
  }
  for(auto & a : axes)
  {
    a = 0.0;
  }
  axes[static_cast<size_t>(mc_joystick::Axis::L2)] = 1.0;
  axes[static_cast<size_t>(mc_joystick::Axis::R2)] = 1.0;
}

} // namespace mc_joystick
