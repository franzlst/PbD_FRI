#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(naxes_motion_control)

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

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libnaxes_motion_control-<target>.so
# and installs in the directory lib/orocos/naxes_motion_control/
#
orocos_component(nAxesControllerPos src/nAxesControllerPos.cpp)
#orocos_component(nAxesControllerPosVelAcc src/nAxesControllerPosVel.cpp)
#orocos_component(nAxesControllerPosVelAcc src/nAxesControllerPosVelAcc.cpp)
#orocos_component(nAxesControllerVel src/nAxesControllerVel.cpp)
orocos_component(nAxesGeneratorPos src/nAxesGeneratorPos.cpp)
#orocos_component(nAxesGeneratorVel src/nAxesGeneratorVel.cpp)

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/naxes_motion_control/ directory
#
#orocos_install_headers( naxes_motion_control-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
