
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libfri_ros_publisher-<target>.so
# and installs in the directory lib/orocos/fri_ros_publisher/
#
orocos_component(fri_ros_publisher fri_ros_publisher-component.hpp fri_ros_publisher-component.cpp) # ...you may add multiple source files
orocos_component(fri_ros_receiver fri_ros_receiver-component.hpp fri_ros_receiver-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/fri_ros_publisher/ directory
#
orocos_install_headers(fri_ros_publisher-component.hpp fri_ros_receiver-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libfri_ros_publisher-plugin-<target>.so
# and installs in the directory lib/orocos/fri_ros_publisher/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(fri_ros_publisher-plugin fri_ros_publisher-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libfri_ros_publisher-service-<target>.so
# and installs in the directory lib/orocos/fri_ros_publisher/plugins/
#
#orocos_service(fri_ros_publisher-service fri_ros_publisher-service.cpp) # ...only one service per library !
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


