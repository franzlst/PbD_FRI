#include "basic_moves-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Basic_moves::Basic_moves(std::string const& name) : TaskContext(name){
  std::cout << "Basic_moves constructed !" <<std::endl;
}

bool Basic_moves::configureHook(){
  std::cout << "Basic_moves configured !" <<std::endl;
  return true;
}

bool Basic_moves::startHook(){
  std::cout << "Basic_moves started !" <<std::endl;
  return true;
}

void Basic_moves::updateHook(){
  std::cout << "Basic_moves executes updateHook !" <<std::endl;
}

void Basic_moves::stopHook() {
  std::cout << "Basic_moves executes stopping !" <<std::endl;
}

void Basic_moves::cleanupHook() {
  std::cout << "Basic_moves cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Basic_moves)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Basic_moves)
