#include "basic_moves-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

namespace iros {

using namespace RTT;
using namespace std;

Basic_moves::Basic_moves(std::string const& name) : TaskContext(name){
	this->ports()->addEventPort( "JointsPort", joints_pos_port ).doc( "Retrieves joint positions and fires an event when receiving" );
	this->ports()->addPort("PosePort", pose_port).doc("Retrieved Cartesian position and orientation.");
	std::cout << "Basic_moves constructed !" <<std::endl;
}

Basic_moves::~Basic_moves()
{}

bool Basic_moves::configureHook(){
	if(!joints_pos_port.connected()) {
		std::cout << "Joints port not connected. Stopping..." << std::endl;
		return false;
	}

	if(!pose_port.connected()) {
		std::cout << "Pose port not connected. Stopping..." << std::endl;
		return false;
	}

	std::cout << "Basic_moves configured !" <<std::endl;
	return true;
}

bool Basic_moves::startHook(){
	std::cout << "Basic_moves started !" <<std::endl;
	return true;
}

void Basic_moves::updateHook(){

	if(joints_pos_port.read(current_joints_pos) == NewData)
		cout << "New Cartesian Pos Data!" << endl;

	//if(joints_pos_port.read(current_joints_pos) == NewData ||
	if(pose_port.read(current_pose) == NewData) {
		//std::cout << "Basic_moves received new data!" <<std::endl;
		cout.precision(5);
		cout << "Current Pos: " << current_pose.position.x << " - " << current_pose.position.y << " - "<< current_pose.position.z << endl;
		cout << "And Joints: " << current_joints_pos.position.at(0) << " - "
				<< current_joints_pos.position.at(1) << " - "
				<< current_joints_pos.position.at(2) << " - "
				<< current_joints_pos.position.at(3) << " - "
				<< current_joints_pos.position.at(4) << " - "
				<< current_joints_pos.position.at(5) << endl;
	}

	std::cout << "Basic_moves executes updateHook !" <<std::endl;
}

void Basic_moves::stopHook() {
	std::cout << "Basic_moves executes stopping !" <<std::endl;
}

void Basic_moves::cleanupHook() {
	std::cout << "Basic_moves cleaning up !" <<std::endl;
}

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
ORO_CREATE_COMPONENT(iros::Basic_moves)
