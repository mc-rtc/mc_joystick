#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_joystick
{

/** Possible buttons on a joystick
 *
 * Uses XBox names for face button/layout (Y->B->A->X from the top in clockwise order) and Playstation for everything
 * else (i.e. L[1-3]/R[1-3], Select, Start, Home)
 *
 */
enum class Button : std::uint8_t
{
  Y = 0,
  B,
  A,
  X,
  L1,
  L3,
  R1,
  R3,
  Select,
  Start,
  Home,
  DPad_Up,
  DPad_Down,
  DPad_Left,
  DPad_Right,
  COUNT = DPad_Right + 1 // provide a count of Button in the library for internal use
};

/** Possible axes on a Joystick
 *
 * The following conventions are enforced by the plugin
 * - on *_LR axes: left is 1.0, right is -1.0
 * - on *_UD axes: up is 1.0, down is -1.0
 * - L2/R2: 1.0 is off, -1.0 is fully pressed
 *
 */
enum class Axis : std::uint8_t
{
  Left_LR = 0,
  Left_UD,
  Right_LR,
  Right_UD,
  L2,
  R2,
  COUNT = R2 + 1 // provide a count of Axis in the library for internal use
};

struct State
{
  static constexpr size_t ButtonCount = static_cast<size_t>(Button::COUNT);
  static constexpr size_t AxisCount = static_cast<size_t>(Axis::COUNT);

  struct ButtonArray : public std::array<bool, ButtonCount>
  {
    using std::array<bool, ButtonCount>::operator[];

    bool operator[](mc_joystick::Button b) const noexcept
    {
      return (*this)[static_cast<size_t>(b)];
    }

    bool & operator[](mc_joystick::Button b) noexcept
    {
      return (*this)[static_cast<size_t>(b)];
    }
  };

  struct AxisArray : public std::array<double, AxisCount>
  {
    using std::array<double, AxisCount>::operator[];

    double operator[](mc_joystick::Axis a) const noexcept
    {
      return (*this)[static_cast<size_t>(a)];
    }

    double & operator[](mc_joystick::Axis a) noexcept
    {
      return (*this)[static_cast<size_t>(a)];
    }
  };

  /** Buttons activation state, true if pushed, false otherwise */
  ButtonArray buttons;
  /** Axis state, the values are in the [-1.0, 1.0] range, \ref Axis for the meaning based on the axis */
  AxisArray axes;
};

} // namespace mc_joystick

namespace mc_rtc
{

template<>
struct ConfigurationLoader<mc_joystick::Button>
{
  inline static mc_joystick::Button load(const mc_rtc::Configuration & config)
  {
    std::string in = config;
#define LOAD_IT(BUTTON)                 \
  else if(in == #BUTTON)                \
  {                                     \
    return mc_joystick::Button::BUTTON; \
  }
    if(in == "YY")
    {
      return mc_joystick::Button::Y;
    }
    LOAD_IT(B)
    LOAD_IT(A)
    LOAD_IT(X)
    LOAD_IT(L1)
    LOAD_IT(L3)
    LOAD_IT(R1)
    LOAD_IT(R3)
    LOAD_IT(Select)
    LOAD_IT(Start)
    LOAD_IT(Home)
    LOAD_IT(DPad_Up)
    LOAD_IT(DPad_Down)
    LOAD_IT(DPad_Left)
    LOAD_IT(DPad_Right)
#undef LOAD_IT
    auto message = fmt::format("Unknown button {}", in);
    mc_rtc::log::critical(message);
    throw mc_rtc::Configuration::Exception(message, config);
  }

  inline mc_rtc::Configuration save(const mc_joystick::Button & button)
  {
    mc_rtc::Configuration config;
#define SAVE_CASE(BUTTON)           \
  case mc_joystick::Button::BUTTON: \
    return #BUTTON
    switch(button)
    {
      case mc_joystick::Button::Y:
        return "YY";
        SAVE_CASE(B);
        SAVE_CASE(A);
        SAVE_CASE(X);
        SAVE_CASE(L1);
        SAVE_CASE(L3);
        SAVE_CASE(R1);
        SAVE_CASE(R3);
        SAVE_CASE(Select);
        SAVE_CASE(Start);
        SAVE_CASE(Home);
        SAVE_CASE(DPad_Up);
        SAVE_CASE(DPad_Down);
        SAVE_CASE(DPad_Left);
        SAVE_CASE(DPad_Right);
      default:
        config.add("b", "UNKNOWN");
        return config("b");
    }
#undef SAVE_CASE
  }
};

template<>
struct ConfigurationLoader<mc_joystick::Axis>
{
  inline static mc_joystick::Axis load(const mc_rtc::Configuration & config)
  {
    std::string in = config;
#define LOAD_IT(AXIS)               \
  else if(in == #AXIS)              \
  {                                 \
    return mc_joystick::Axis::AXIS; \
  }
    if(in == "Left_LR")
    {
      return mc_joystick::Axis::Left_LR;
    }
    LOAD_IT(Left_UD)
    LOAD_IT(Right_LR)
    LOAD_IT(Right_UD)
    LOAD_IT(L2)
    LOAD_IT(R2)
#undef LOAD_IT
    auto message = fmt::format("Unknown axis {}", in);
    mc_rtc::log::critical(message);
    throw mc_rtc::Configuration::Exception(message, config);
  }

  inline mc_rtc::Configuration save(const mc_joystick::Axis & button)
  {
    mc_rtc::Configuration config;
#define SAVE_CASE(AXIS)         \
  case mc_joystick::Axis::AXIS: \
    return #AXIS;
    switch(button)
    {
      SAVE_CASE(Left_LR)
      SAVE_CASE(Left_UD)
      SAVE_CASE(Right_LR)
      SAVE_CASE(Right_UD)
      SAVE_CASE(L2)
      SAVE_CASE(R2)
      default:
        config.add("a", "UNKNOWN");
        return config("a");
    }
#undef SAVE_CASE
  }
};

} // namespace mc_rtc
