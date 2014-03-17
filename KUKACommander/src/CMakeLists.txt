
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libKUKACommander-<target>.so
# and installs in the directory lib/orocos/KUKACommander/
#
orocos_component(KUKACommander KUKACommander.hpp KUKACommander.cpp) # ...you may add multiple source files
orocos_component(KUKACommanderROS KUKACommanderROS.hpp KUKACommanderROS.cpp) # ...you may add multiple source files

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/KUKACommander/ directory
#
orocos_install_headers(KUKACommander.hpp KUKACommanderROS.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libKUKACommander-plugin-<target>.so
# and installs in the directory lib/orocos/KUKACommander/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(KUKACommander-plugin KUKACommander-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libKUKACommander-service-<target>.so
# and installs in the directory lib/orocos/KUKACommander/plugins/
#
#orocos_service(KUKACommander-service KUKACommander-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


