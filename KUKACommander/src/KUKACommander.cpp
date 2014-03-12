#include "KUKACommander.hpp"
#include <rtt/Component.hpp>
#include <iostream>

namespace iros {

	using namespace RTT;
	using namespace std;

	KUKACommander::KUKACommander(std::string const& name) : TaskContext(name){
		Logger::In in((this->getName()));

		call_set_ctrl_mode = false;

		this->addPort("fromKRL", port_from_krl);
		this->addPort("toKRL", port_to_krl);

		this->addPort("RobotState", port_robot_state).doc("Port containing the status of the robot");
		this->addEventPort("FRIState", port_fri_state).doc("Port containing the status of the FRI communication");

		// Operations regarding the control of the KRC and grouped in the Commander service
		provides("Commander")->addOperation("getCurrentState", &KUKACommander::getCurrentState, this, OwnThread).doc("get the current FRI state");
		provides("Commander")->addOperation("getCurrentControlMode", &KUKACommander::getCurrentControlMode, this, OwnThread).doc("get the current control mode");
		provides("Commander")->addOperation("switchToMonitorState", &KUKACommander::switchToMonitorState, this, OwnThread).doc("switch to monitor state: friStop()");
		provides("Commander")->addOperation("switchToCommandState", &KUKACommander::switchToCommandState, this, OwnThread).doc("switch to command state: friStart()");
		provides("Commander")->addOperation("stopCommunication", &KUKACommander::stopCommunication, this, OwnThread).doc("stop communication: friClose");
		provides("Commander")->addOperation("setControlMode", &KUKACommander::setControlMode, this, OwnThread).doc("set control mode");

		// Initialize variables with 0
		data_to_krl.boolData = 0;
		for(uint8_t i = 0; i < FRI_USER_SIZE; i++) {
			data_to_krl.intData[i] = 0;
			data_to_krl.realData[i] = 0;
		}
		// Set default controller mode
		data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_POSITION * 10;

		log(Debug) << "KUKACommander constructed !" << endlog();
	}

	bool KUKACommander::configureHook(){
		Logger::In in((this->getName()));
		log(Debug) << "KUKACommander configured !" << endlog();
		return true;
	}

	bool KUKACommander::startHook(){
		Logger::In in((this->getName()));
		log(Debug) << "KUKACommander started !" << endlog();
		return true;
	}

	void KUKACommander::updateHook(){
		Logger::In in((this->getName()));

		port_fri_state.read(data_fri_state);
		port_robot_state.read(data_robot_state);

		port_from_krl.read(data_from_krl);


		if(call_set_ctrl_mode)
			//setControlMode();

		log(Debug) << "KUKACommander executes updateHook !" << endlog();
	}

	void KUKACommander::stopHook() {
		Logger::In in((this->getName()));
		log(Debug) << "KUKACommander executes stopping !" << endlog();
	}

	void KUKACommander::cleanupHook() {
		Logger::In in((this->getName()));
		log(Debug) << "KUKACommander cleaning up !" << endlog();
	}

	FRI_STATE KUKACommander::getCurrentState() {
		// Get the FRI state from the FRI variables, written by KRC program
		switch(data_from_krl.intData[FRI_FROM_KRL_STATE]) {
			case 10:
				return FRI_STATE_MON;
			case 20:
				return FRI_STATE_CMD;
			default:
				return FRI_STATE_OFF;
		}
	}

	FRI_CTRL KUKACommander::getCurrentControlMode() {
		return (FRI_CTRL) data_robot_state.control;
	}

	void KUKACommander::switchToMonitorState() {
		data_to_krl.intData[FRI_TO_KRL_MON_CMD] = 20;
		port_to_krl.write(data_to_krl);
	}

	void KUKACommander::switchToCommandState() {
		data_to_krl.intData[FRI_TO_KRL_MON_CMD] = 10;
		port_to_krl.write(data_to_krl);
	}

	void KUKACommander::stopCommunication() {
		// value must be copied, either 30 or 40
		data_to_krl.intData[FRI_TO_KRL_MON_CMD] = data_from_krl.intData[FRI_FROM_KRL_COND];
		port_to_krl.write(data_to_krl);
	}

	void KUKACommander::setControlMode(FRI_CTRL mode) {
		// Stores the state when being commanded to switch the control mode
		static FRI_STATE final_state = FRI_STATE_MON;
		// Stores the desired control mode
		static FRI_CTRL final_mode = FRI_CTRL_OTHER;

		// We are in the desired control mode
		if(mode == getCurrentControlMode() || final_mode == getCurrentControlMode()) {
			// We have previously been in another state, so to complete the action, switch to this state
			if(final_state != getCurrentState()) {
				log(Debug) << "Control mode set, will set state now..." << endlog();
				if(final_state == FRI_STATE_CMD)
					switchToCommandState();
				else
					switchToMonitorState();
				call_set_ctrl_mode = true;
				return;
			// Both state and command mode are the desired ones, we are done
			} else {
				log(Info) << "Set new control mode: " << mode << endlog();
				call_set_ctrl_mode = false;
				final_state = FRI_STATE_MON; // default value
				return;
			}
		// Store the desired mode for later use
		} else if(mode != FRI_CTRL_OTHER) {
			final_mode = mode;
		}

		// If in command state, we cannot witch the control mode, so switch to monitor state
		if(getCurrentState() == FRI_STATE_CMD) {
			log(Debug) << "Switching to monitor state before switching control mode..." << endlog();
			final_state = FRI_STATE_CMD;
			call_set_ctrl_mode = true;
			switchToMonitorState();
			return;
		// Switch the control mode
		} else if(getCurrentState() == FRI_STATE_MON) {
			log(Debug) << "Switching control mode..." << endlog();
			call_set_ctrl_mode = true;
			switch(final_mode) {
				case FRI_CTRL_POSITION:
					data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_POSITION * 10;
					return;
				case FRI_CTRL_JNT_IMP:
					data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_JNT_IMP * 10;
					return;
				case FRI_CTRL_CART_IMP:
					data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_CART_IMP * 10;
					return;
				default:
					log(Warning) << "Unknown control mode requested: " << final_mode << endlog();
					call_set_ctrl_mode = false;
					final_mode = getCurrentControlMode();
					final_state = getCurrentState();
					return;
			}
		}
	}
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(KUKACommander)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(iros::KUKACommander)
