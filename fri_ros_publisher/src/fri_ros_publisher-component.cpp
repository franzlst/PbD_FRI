#include "fri_ros_publisher-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

namespace iros {

	using namespace RTT;
	using namespace std;

	FRI_ROS_Publisher::FRI_ROS_Publisher(std::string const& name) : TaskContext(name){
		Logger::In in((this->getName()));

		// Input ports
		this->ports()->addPort("FRIState", fri_state_port).doc("Gets information about the FRI state");
		this->ports()->addPort("RobotState", robot_state_port).doc("Gets information about the robot state");

		this->ports()->addEventPort( "JointsPort", joints_pos_port ).doc( "Retrieves joint positions and fires an event when receiving" );
		this->ports()->addPort("PosePort", pose_port).doc("Retrieves Cartesian position and orientation.");
		this->ports()->addPort("ForcePort", force_port).doc("Retrieves Cartesian forces and torques");

		// Output ports
		this->ports()->addPort("JointImpedancePort", joint_impedance_port).doc("Publish joint impedances");
		this->ports()->addPort("CartImpedancePort", cart_impedance_port).doc("Publish Cartesian impedances");

		this->ports()->addPort("JointsPos_ROSPort", joints_pos_ros_port).doc("Publish joint positions");
		this->ports()->addPort("Pose_ROSPort", pose_ros_port).doc("Publish pose of the tool");
		this->ports()->addPort("Force_ROSPort", force_ros_port).doc("Publish external force");

		for(auto i = 0; i < 7; i++) {
			gravity_compensation_joints.damping[i] = 0.7;
			gravity_compensation_joints.stiffness[i] = 0.01;
		}

		gravity_compensation_cart.damping.angular.x = 0.3;
		gravity_compensation_cart.damping.angular.y = 0.3;
		gravity_compensation_cart.damping.angular.z = 0.3;
		gravity_compensation_cart.damping.linear.x = 0.3;
		gravity_compensation_cart.damping.linear.y = 0.3;
		gravity_compensation_cart.damping.linear.z = 0.3;
		gravity_compensation_cart.stiffness.angular.x = 0.01;
		gravity_compensation_cart.stiffness.angular.y = 0.01;
		gravity_compensation_cart.stiffness.angular.z = 0.01;
		gravity_compensation_cart.stiffness.linear.x = 0.01;
		gravity_compensation_cart.stiffness.linear.y = 0.01;
		gravity_compensation_cart.stiffness.linear.z = 0.01;

		log(Info) << "FRI_ROS_Publisher constructed !" << endlog();
	}

	bool FRI_ROS_Publisher::configureHook(){
		Logger::In in((this->getName()));

		if(!fri_state_port.connected()) {
			log(Error) << "FRI state port not connected. Stopping..." << endlog();
			return false;
		}

		if(!robot_state_port.connected()) {
			log(Error) << "Robot state port not connected. Stopping..." << endlog();
			return false;
		}

		if(!joints_pos_port.connected()) {
			log(Error) << "Joints port not connected. Stopping..." << endlog();
			return false;
		}

		if(!pose_port.connected()) {
			log(Error) << "Pose port not connected. Stopping..." << endlog();
			return false;
		}

		if(!force_port.connected()) {
			log(Error) << "Force port not connected. Stopping..." << endlog();
			return false;
		}

		log(Info) << "FRI_ROS_Publisher configured !" << endlog();
		return true;
	}

	bool FRI_ROS_Publisher::startHook(){
		Logger::In in((this->getName()));

		log(Info) << "FRI_ROS_Publisher started !" << endlog();
		return true;
	}

	void FRI_ROS_Publisher::updateHook(){
		Logger::In in((this->getName()));

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

		// When the drives are enabled (power on) and there are new values, publish them to ROS
		if(current_robot_state.power) { // power on
			if(current_robot_state.control != FRI_CTRL_CART_IMP) {
				static auto last_monitor_out_time = current_fri_state.timestamp;
				if(current_fri_state.timestamp - last_monitor_out_time > 1) {
					log(Info) << "Please switch to Joint Impedance Mode to record the trajectory" << endlog();
					last_monitor_out_time = current_fri_state.timestamp;
				}
				return;
			}

			if(pose_port.read(current_pose) == NewData) {
				static auto last_monitor_out_time = current_fri_state.timestamp;
				if(current_fri_state.timestamp - last_monitor_out_time > 1) {
					log(Info) << "Recording trajectory..." << endlog();
					last_monitor_out_time = current_fri_state.timestamp;
				}

				cart_impedance_port.write(gravity_compensation_cart);

				joints_pos_port.read(current_joints_pos);
				force_port.read(current_force);

				joints_pos_ros_port.write(current_joints_pos);
				pose_ros_port.write(current_pose);
				force_ros_port.write(current_force);
			}
		}

		//log(Info) << "FRI_ROS_Publisher executes updateHook !" << endlog();
	}

	void FRI_ROS_Publisher::stopHook() {
		//std::cout << "FRI_ROS_Publisher executes stopping !" <<std::endl;
	}

	void FRI_ROS_Publisher::cleanupHook() {
		//std::cout << "FRI_ROS_Publisher cleaning up !" <<std::endl;
	}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Fri_ros_publisher)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(iros::FRI_ROS_Publisher)
