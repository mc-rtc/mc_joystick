cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME mc-rtc-joystick)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/mc-rtc/mc-rtc-joystick)
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

if(DEFINED BUILD_SHARED_LIBS)
  set(WAS_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
endif()
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libs" FORCE)
add_subdirectory(ext/glfw EXCLUDE_FROM_ALL)
if(DEFINED WAS_BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ${WAS_BUILD_SHARED_LIBS} CACHE BOOL "Build shared libs" FORCE)
endif()

set(plugin_SRC
  src/Joystick.cpp
)

set(plugin_HDR
  src/Joystick.h
  src/JoystickState.h
)

add_plugin(Joystick ${plugin_SRC} ${plugin_HDR})
target_include_directories(Joystick PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(Joystick PUBLIC glfw mc_rtc::mc_rtc_ros)

install(FILES ${plugin_HDR} DESTINATION include/mc_joystick)
