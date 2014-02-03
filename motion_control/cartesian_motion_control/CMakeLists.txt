#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(cartesian_motion_control)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(kinematic_converter src/kinematic_converter.hpp src/kinematic_converter.cpp)
orocos_component(CartesianControllerPos src/CartesianControllerPos.cpp src/CartesianControllerPos.hpp)
orocos_component(CartesianControllerVel src/CartesianControllerVel.cpp src/CartesianControllerVel.hpp)
orocos_component(CartesianControllerPosVel src/CartesianControllerPosVel.cpp src/CartesianControllerPosVel.hpp)
orocos_component(CartesianGeneratorPos  src/CartesianGeneratorPos.cpp src/CartesianGeneratorPos.hpp)
orocos_component(CartesianImpedanceController  src/CartesianImpedanceController.cpp src/CartesianImpedanceController.hpp)


#
# You may add multiple orocos_component statements.

#
# Building a typekit (recommended):
#
# Creates a typekit library libmotion_control-types-<target>.so
# and installs in the directory lib/orocos/motion_control/types/
#
#orocos_typegen_headers(motion_control-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libmotion_control-service-<target>.so or libmotion_control-plugin-<target>.so
# and installs in the directory lib/orocos/motion_control/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_service(motion_control-service motion_control-service.cpp) # ...only one service per library !
#orocos_plugin(motion_control-plugin motion_control-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin/orocos_service statements.


#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/motion_control/ directory
#
orocos_install_headers( src/CartesianControllerPos.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
