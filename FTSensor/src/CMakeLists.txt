
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libFTSensor-<target>.so
# and installs in the directory lib/orocos/FTSensor/
#


#common commands for building c++ executables and libraries

find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
include_directories(${RTNET_INCLUDE_DIR})
add_definitions(-DHAVE_RTNET)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#SET(CMAKE_VERBOSE_MAKEFILE on )


orocos_component(FTSensor FTSensor-component.hpp FTSensor-component.cpp) # ...you may add multiple source files
target_link_libraries(FTSensor ${XENO_NATIVE_LDFLAGS} -lrtdm)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/FTSensor/ directory
#
orocos_install_headers(FTSensor-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libFTSensor-plugin-<target>.so
# and installs in the directory lib/orocos/FTSensor/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(FTSensor-plugin FTSensor-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libFTSensor-service-<target>.so
# and installs in the directory lib/orocos/FTSensor/plugins/
#
#orocos_service(FTSensor-service FTSensor-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


