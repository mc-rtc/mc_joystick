cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME mc_joystick)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/mc-rtc/mc_joystick)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(CXX_DISABLE_WERROR ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME})

add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugins is required for this plugin")
endif()

set(plugin_SRC
  src/Joystick.cpp
  src/JoystickState.cpp
  src/updaters/ROSUpdater.cpp
)

set(plugin_PUBLIC_HDR
  src/Joystick.h
  src/JoystickState.h
)

set(plugin_PRIVATE_HDR
  src/updaters/ROSUpdater.h
)

set(plugin_HDR
  ${plugin_PUBLIC_HDR}
  ${plugin_PRIVATE_HDR}
)

add_plugin(Joystick ${plugin_SRC} ${plugin_HDR})
target_include_directories(Joystick PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(Joystick PUBLIC mc_rtc::mc_rtc_ros)

install(FILES ${plugin_PUBLIC_HDR} DESTINATION include/mc_joystick)
