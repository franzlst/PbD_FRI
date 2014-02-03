#ifndef OROCOS_BASIC_MOVES_COMPONENT_HPP
#define OROCOS_BASIC_MOVES_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Basic_moves : public RTT::TaskContext{
  public:
    Basic_moves(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
