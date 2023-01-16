mc-rtc joystick plugin
==

Provides a plugin that can provide a joystick state to mc-rtc components

Two types of joystick are supported:
- ROS-based joystick (subscriber to a `sensors_msgs/Joy` topic)
- GLFW-based joystick

These joysticks are abstracted to a common model, the following sample shows how you can read the data from the plugin:

```cpp
#include <mc_joystick/JoystickState.h>

// Show available controllers
const auto & available_joysticks = ctl.datastore().get<std::vector<std::string>>("Joystick::connected");
if(available_joysticks.size() == 0)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick connected, bye!", name());
}

// Choose the first joystick available
std::string joystick = available_joysticks[0] + "::state";

// Get the latest state
const auto & state = *ctl.datastore().get<const mc_joystick::State *>(joystick);

// Axis values are in the [-1, 1] range
// On *_LR axes: left is 1.0 and right is -1.0
// On *_UD axes: up is 1.0 and down is -1.0
// L2/R2: 1.0 is off, -1.0 is fully pressed
using Axis = mc_joystick::Axis;
double lr = state.axes[Axis::Left_LR];
double ud = state.axes[Axis::Left_UD];

// Buttons are true when pushed, false otherwise
using Button = mc_joystick::Button;
bool L1 = state.buttons[Button::L1];
bool A  = state.buttons[Button::A];
```

You can make sure the plugin is installed on your system:

```cmake
find_package(mc_rtc_joystick REQUIRED)
```

See [etc/Joystick.yaml](etc/Joystick.yaml) for an example configuration.
