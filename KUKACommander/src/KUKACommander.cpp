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

		this->addPort("desJntImpedance", port_desJntImpedance).doc("Desired joint impedance (stiffness and dampening)");
		this->addPort("desCartImpedance", port_desCartImpedance).doc("Desired Cartesian impedance (stiffness and dampening)");

		this->addPort("msrJointState", port_msrJointState).doc("Measured joint states (position/rotation)");
		this->addPort("msrCartPos", port_msrCartPos).doc("Measured joint states (position/rotation)");
		this->addPort("msrExtCartWrench", port_msrExtCartWrench).doc("Measured joint states (position/rotation)");

		this->addEventPort("nAxesEvent", port_nAxesEvent, boost::bind(&KUKACommander::n_axes_process_event, this, _1)).doc("Retrieves events from nAxesGeneratorPos when a movement has started/stopped");
		this->addEventPort("cartesianEvent", port_cartesianEvent, boost::bind(&KUKACommander::cartesian_process_event, this, _1)).doc("Retrieves events from CartesianGeneratorPos when a movement has started/stopped");

		this->addEventPort("generatedCartPos", port_generatedCartPos, boost::bind(&KUKACommander::merge_generated_cartesian, this, _1)).doc("Retrieves generated desired Cartesian position");
		this->addPort("generatedCartVel", port_generatedCartVel).doc("Retrieves generated desired Cartesian velocity");
		this->addPort("generatedCartOdom", port_generatedCartOdom).doc("Merges Cartesian position and velocity to odometry");

		this->addEventPort("generatedJntState", port_generatedJntState, boost::bind(&KUKACommander::convert_joint_state_to_pos, this, _1)).doc("Retrieves generated desired joint state");
		this->addPort("desiredJntPos", port_desiredJntPos).doc("Converted state to joint position");

		// Operations regarding the control of the KRC and grouped in the Commander service
		provides("Commander")->addOperation("getCurrentState", &KUKACommander::getCurrentState, this, OwnThread).doc("get the current FRI state");
		provides("Commander")->addOperation("getCurrentControlMode", &KUKACommander::getCurrentControlMode, this, OwnThread).doc("get the current control mode");

		provides("Commander")->addOperation("switchToMonitorState", &KUKACommander::switchToMonitorState, this, OwnThread).doc("switch to monitor state: friStop()");
		provides("Commander")->addOperation("switchToCommandState", &KUKACommander::switchToCommandState, this, OwnThread).doc("switch to command state: friStart()");
		provides("Commander")->addOperation("stopCommunication", &KUKACommander::stopCommunication, this, OwnThread).doc("stop communication: friClose");

		provides("Commander")->addOperation("setControlMode", &KUKACommander::setControlMode, this, OwnThread).doc("set control mode");
		provides("Commander")->addOperation("activateGravityCompensation", &KUKACommander::activateGravityCompensation, this, OwnThread).doc("Switches to gravity compensation mode");

		provides("Commander")->addOperation("setCartesianImpedance", &KUKACommander::setCartesianImpedance, this, OwnThread).doc("Sets the Cartesian impedance");
		provides("Commander")->addOperation("setJointImpedance", &KUKACommander::setJointImpedance, this, OwnThread).doc("Sets the Joint impedance");

		provides("Commander")->addOperation("moveToJointPosition", &KUKACommander::moveToJointPosition, this, OwnThread).doc("Moves the arm to the desired joint position");
		provides("Commander")->addOperation("moveToCartesianPosition", &KUKACommander::moveToCartesianPosition, this, OwnThread).doc("Moves the arm to the desired joint position");

		provides("Commander")->addOperation("stopMovements", &KUKACommander::stopMovements, this, OwnThread).doc("Stops all movements");
		provides("Commander")->addOperation("isMoving", &KUKACommander::isMoving, this, OwnThread).doc("Checks whether the robot is moving");

		// Initialize variables with 0
		data_to_krl.boolData = 0;
		for(uint8_t i = 0; i < FRI_USER_SIZE; i++) {
			data_to_krl.intData[i] = 0;
			data_to_krl.realData[i] = 0;
		}
		// Set default controller mode
		data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_POSITION * 10;

		nAxes_is_moving = false;
		cart_is_moving = false;

		log(Debug) << "KUKACommander constructed !" << endlog();
	}

	bool KUKACommander::configureHook(){
		Logger::In in((this->getName()));

		if(!(n_axes_generator_pos_peer = getPeer("nAxesGeneratorPos"))) {
			log(Error) << "Peer 'nAxesGeneratorPos' not found" << endlog();
			return false;
		}
		nAxes_moveTo = n_axes_generator_pos_peer->getOperation("moveTo");
		nAxes_resetPosition = n_axes_generator_pos_peer->getOperation("resetPosition");

		if(!(cartesian_generator_pos_peer = getPeer("CartesianGeneratorPos"))) {
			log(Error) << "Peer 'CartesianGeneratorPos' not found" << endlog();
			return false;
		}

		cartesian_moveTo = cartesian_generator_pos_peer->getOperation("moveTo");
		cartesian_resetPosition = cartesian_generator_pos_peer->getOperation("resetPosition");

		log(Debug) << "KUKACommander configured !" << endlog();
		return true;
	}

	bool KUKACommander::startHook(){
		Logger::In in((this->getName()));
		port_to_krl.write(data_to_krl);
		log(Debug) << "KUKACommander started !" << endlog();
		return true;
	}

	void KUKACommander::updateHook(){
		Logger::In in((this->getName()));

		port_fri_state.read(data_fri_state);
		port_robot_state.read(data_robot_state);

		port_from_krl.read(data_from_krl);


		if(call_set_ctrl_mode)
			setControlMode();

		//log(Debug) << "KUKACommander executes updateHook !" << endlog();
	}

	void KUKACommander::stopHook() {
		Logger::In in((this->getName()));
		stopCommunication();
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
		log(Debug) << "Switching to Monitor state" << endlog();
		data_to_krl.intData[FRI_TO_KRL_MON_CMD] = 20;
		port_to_krl.write(data_to_krl);
	}

	void KUKACommander::switchToCommandState() {
		log(Debug) << "Switching to Command state" << endlog();
		data_to_krl.intData[FRI_TO_KRL_MON_CMD] = 10;
		port_to_krl.write(data_to_krl);
	}

	void KUKACommander::stopCommunication() {
		log(Debug) << "Stopping communication" << endlog();
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
				log(Info) << "Successfully set new control mode: " << final_mode << endlog();
				call_set_ctrl_mode = false;
				final_state = FRI_STATE_MON; // default value
				final_mode = FRI_CTRL_OTHER; // default value
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
					break;
				case FRI_CTRL_JNT_IMP:
					data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_JNT_IMP * 10;
					break;
				case FRI_CTRL_CART_IMP:
					data_to_krl.intData[FRI_TO_KRL_CTRL_MODE] = FRI_CTRL_CART_IMP * 10;
					break;
				default:
					log(Warning) << "Unknown control mode requested: " << final_mode << endlog();
					call_set_ctrl_mode = false;
					final_mode = getCurrentControlMode();
					final_state = getCurrentState();
					return;
			}
			port_to_krl.write(data_to_krl);
			return;
		}
	}

	void KUKACommander::setCartesianImpedance(geometry_msgs::Twist stiffness, geometry_msgs::Twist damping) {
		log(Debug) << "Changing Cartesian impedance" << endlog();
		lwr_fri::CartesianImpedance impedance;
		impedance.stiffness = stiffness;
		impedance.damping = damping;
		port_desCartImpedance.write(impedance);
	}

	void KUKACommander::setJointImpedance(boost::array<float, 7> stiffness, boost::array<float, 7> damping) {
		log(Debug) << "Changing joint impedance" << endlog();
		lwr_fri::FriJointImpedance impedance;
		impedance.stiffness = stiffness;
		impedance.damping = damping;
		port_desJntImpedance.write(impedance);
	}

	void KUKACommander::activateGravityCompensation(bool joint) {
		if(joint) {
			boost::array<float, 7> stiffness = {{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }};
			boost::array<float, 7> damping = {{ 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 }};
			setJointImpedance(stiffness, damping);
			setControlMode(FRI_CTRL_JNT_IMP);
		} else {
			geometry_msgs::Vector3 stiff;
			stiff.x = 0; stiff.y =  0; stiff.z = 0;
			geometry_msgs::Twist stiffness;
			stiffness.linear = stiff; stiffness.angular = stiff;
			geometry_msgs::Vector3 damp;
			damp.x = 0.7; damp.y = 0.7; damp.z = 0.7;
			geometry_msgs::Twist damping;
			damping.linear = damp; damping.angular = damp;
			setCartesianImpedance(stiffness, damping);
			setControlMode(FRI_CTRL_CART_IMP);
		}
	}

	bool KUKACommander::moveToJointPosition(boost::array<double, 7> jointPos, double time) {
		if(nAxes_is_moving || cart_is_moving)
			return false;

		return nAxes_moveTo(vector<double>(jointPos.begin(), jointPos.end()), time);
	}

	bool KUKACommander::moveToCartesianPosition(geometry_msgs::Pose CartPos, double time) {
		if(nAxes_is_moving || cart_is_moving)
			return false;

		log(Debug) << "move to Cartesian pos" << endlog();
		return cartesian_moveTo(CartPos, time);
	}

	void KUKACommander::stopMovements() {
		if(nAxes_is_moving)
			nAxes_resetPosition();
		else if(cart_is_moving)
			cartesian_resetPosition();
	}

	bool KUKACommander::isMoving() {
		return nAxes_is_moving || cart_is_moving;
	}

	void KUKACommander::print_var_from_krl() {
		print_user_variables(data_from_krl, true);
	}

	void KUKACommander::print_var_to_krl() {
		print_user_variables(data_to_krl, false);
	}

	void KUKACommander::print_user_variables(tFriKrlData var, bool from) {
		log(Info) << "=== Variables " << (from ? "from KRC" : "to KRC") << " ===" << endlog();
		log(Info) << "Booleans: " << var.boolData << endlog();
		log(Info) << "\tInteger \t Real " << endlog();
		for(uint8_t i = 0; i < FRI_USER_SIZE; i++) {
			log(Info) << (i + 1) << ":\t" << var.intData[i] << "\t\t" << var.realData[i] << endlog();
		}
	}

	void KUKACommander::n_axes_process_event(RTT::base::PortInterface*) {
		port_nAxesEvent.read(data_nAxes_event);
		auto pos = data_nAxes_event.find_last_of("_") + 1; // extract event name, form is "e_"+name+"_move_[event]"
		data_nAxes_event = data_nAxes_event.substr(pos, 20);

		if(data_nAxes_event.compare("started") == 0) {
			nAxes_is_moving = true;
		} else if(data_nAxes_event.compare("finished") == 0) {
			nAxes_is_moving = false;
		}
	}

	void KUKACommander::cartesian_process_event(RTT::base::PortInterface*) {
		port_cartesianEvent.read(data_cartesian_event);
		auto pos = data_cartesian_event.find_last_of("_") + 1; // extract event name, form is "e_"+name+"_move_[event]"
		data_cartesian_event = data_cartesian_event.substr(pos, 20);

		if(data_cartesian_event.compare("started") == 0) {
			cart_is_moving = true;
		} else if(data_cartesian_event.compare("finished") == 0) {
			cart_is_moving = false;
		}
	}

	void KUKACommander::merge_generated_cartesian(RTT::base::PortInterface*) {
		geometry_msgs::Pose tmp_pose;
		geometry_msgs::Twist tmp_twist;
		nav_msgs::Odometry tmp_odom;

		if(port_generatedCartPos.read(tmp_pose) == NewData) {
			port_generatedCartVel.read(tmp_twist);
			tmp_odom.pose.pose = tmp_pose;
			tmp_odom.twist.twist = tmp_twist;

			port_generatedCartOdom.write(tmp_odom);
		}
	}

	void KUKACommander::convert_joint_state_to_pos(RTT::base::PortInterface*) {
		sensor_msgs::JointState tmp_state;
		motion_control_msgs::JointPositions tmp_pos;
		if(port_generatedJntState.read(tmp_state) == NewData) {
			tmp_pos.positions = tmp_state.position;
			port_desiredJntPos.write(tmp_pos);
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
