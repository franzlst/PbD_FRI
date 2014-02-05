#include "basic_moves-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <string>
#include <initializer_list>


namespace iros {

using namespace RTT;
using namespace std;

Basic_moves::Basic_moves(std::string const& name) : TaskContext(name), current_joints_pos_index(0), axes_generator_pos_peer(nullptr) {
	this->ports()->addPort("FRIState", fri_state_port).doc("Gets information about the FRI state");
	this->ports()->addPort("RobotState", robot_state_port).doc("Gets information about the robot state");

	this->ports()->addEventPort( "JointsPort", joints_pos_port ).doc( "Retrieves joint positions and fires an event when receiving" );
	this->ports()->addPort("PosePort", pose_port).doc("Retrieves Cartesian position and orientation.");
	this->ports()->addEventPort("AxesEventPort", axes_event_port).doc("Retrieves events when a position was reached");


	joints_pos_storage.push_back(vector<double>({0., 0., 0., 0., 0., 0., 0.}));
	joints_pos_storage.push_back(vector<double>({0.2, 0.3, 0.1, -0.1, -0.4, 0.23, 0.11}));
	joints_pos_storage.push_back(vector<double>({-0.3, -0.6, -0.1, 0.3, 0.3, -0.1, -0.2}));

	std::cout << "Basic_moves constructed !" <<std::endl;
}

Basic_moves::~Basic_moves()
{}

bool Basic_moves::configureHook(){
	axes_generator_pos_peer = getPeer("nAxesGeneratorPos");
	move_to_caller = axes_generator_pos_peer->getOperation("moveTo");
	axes_gen_running_caller = axes_generator_pos_peer->getOperation("isRunning");
	reset_position_caller = axes_generator_pos_peer->getOperation("resetPosition");

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

	if(!axes_gen_running_caller()) {
		log(Error) << "nAxesGeneratorPos not running" << endlog();
		return false;
	}

	return true;
}

void Basic_moves::updateHook(){
	Logger::In in((this->getName()));
	static int iterations = 0;
	static bool moving = false;
	static bool reset = false;

	fri_state_port.read(current_fri_state);
	robot_state_port.read(current_robot_state);

	if(current_robot_state.warning) {
		log(Warning) << "Robot state warning: " << current_robot_state.warning << endlog();
	}

	if(current_robot_state.error) {
		log(Error) << "Robot state error: " << current_robot_state.error << endlog();
	}

	if(current_fri_state.state == FRI_STATE_MON) {
		static auto last_monitor_out_time = current_fri_state.timestamp;
		if(current_fri_state.timestamp - last_monitor_out_time > 1) {
			log(Info) << "Still in monitor mode." << endlog();
			last_monitor_out_time = current_fri_state.timestamp;
		}
		return;
	}

	if(current_robot_state.power == 0) {
		if(moving) {
			log(Warning) << "Robot drives switched off while moving!" << endlog();
			reset_position_caller();
			reset = true;
			moving = false;
			return;
		} else {
			return;
		}
	}

	if(current_fri_state.quality <= FRI_QUALITY_OK) {
		switch(current_fri_state.quality) {
			case FRI_QUALITY_UNACCEPTABLE:
				log(Error) << "FRI quality unacceptable!" << endlog(); return;
			case FRI_QUALITY_BAD:
				log(Warning) << "FRI quality bad!" << endlog(); break;
			case FRI_QUALITY_OK:
				log(Info) << "FRI quality OK!" << endlog(); break;
		}
	}


	if(pose_port.read(current_pose) == NewData) {
		//cout << "New Cartesian Pos Data!" << endl;
		//if(iterations % 100)
		//	log(Info) << "Current Pos: " << current_pose.position.x << " - " << current_pose.position.y << " - "<< current_pose.position.z << endl;
	}

	if(joints_pos_port.read(current_joints_pos) == NewData){}
	/*	cout.precision(4);
		if(iterations % 20 == 0)
			log(Info) << "Joints: " << current_joints_pos.position.at(0) << " - "
				<< current_joints_pos.position.at(1) << " - "
				<< current_joints_pos.position.at(2) << " - "
				<< current_joints_pos.position.at(3) << " - "
				<< current_joints_pos.position.at(4) << " - "
				<< current_joints_pos.position.at(5) << " - "
				<< current_joints_pos.position.at(6) << endlog();*/
	//};

	//SendHandle<bool(const vector<double>&, double)> move_to_handle;
	string axes_event;
	FlowStatus axes_event_status = axes_event_port.read(axes_event);
	if(axes_event_status == NoData) { // We haven't send any axes positions yet
		if(iterations == 20) {
			log(Info) << "Sending initial pose" << endlog();
			//vector<double> cur = vector<double>(current_joints_pos.position);
			//cur[0] -= 0.2;
			//cur[1] -= 0.3;
			//move_to_handle = move_to_caller.send(cur, 0.0);
			reset_position_caller();
			//move_to_handle = move_to_caller.send(joints_pos_storage.at(0), 0.0);
			//bool res;
			//while (move_to_handle.collectIfDone(res) == SendNotReady )
			//      sleep(1);
			bool res = move_to_caller(joints_pos_storage.at(0), 0.0);
			if(res)
				log(Info) << "Initial pose sent" << endlog();
			else
				log(Error) << "Error while sending initial pose" << endlog();
		}
	} else if(axes_event_status == NewData || reset) {
		auto pos = axes_event.find_last_of("_") + 1; // extract event name, form is "e_"+name+"_move_started"
		axes_event = axes_event.substr(pos, 20);

		if(axes_event.compare("started") == 0 && ! reset) {
			moving = true;
			log(Info) << "Started moving..." << endlog();

		} else if(axes_event.compare("finished") == 0 || reset) {
			moving = false;
			reset = false;
			log(Info) << "Position reached, sending new position..." << endlog();
			reset_position_caller();
			current_joints_pos_index = current_joints_pos_index == joints_pos_storage.size() - 1 ? 0 : current_joints_pos_index + 1;
			move_to_caller.send(joints_pos_storage.at(current_joints_pos_index), 0.);
		}
	}
	iterations++;

	//if(joints_pos_port.read(current_joints_pos) == NewData ||
	/*{
		//std::cout << "Basic_moves received new data!" <<std::endl;
		cout.precision(5);
		cout << "And Joints: " << current_joints_pos.position.at(0) << " - "
				<< current_joints_pos.position.at(1) << " - "
				<< current_joints_pos.position.at(2) << " - "
				<< current_joints_pos.position.at(3) << " - "
				<< current_joints_pos.position.at(4) << " - "
				<< current_joints_pos.position.at(5) << endl;
	}*/
	//std::cout << "Basic_moves executes updateHook !" <<std::endl;
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
