#include "fri_ros_receiver-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include "rosbag/view.h"
#include "tf/transform_datatypes.h"

namespace iros {

	using namespace RTT;
	using namespace std;

	FRI_ROS_Receiver::FRI_ROS_Receiver(std::string const& name) : TaskContext(name), traj_bag_file("traj.bag"){
		Logger::In in((this->getName()));

		// Input ports
		this->ports()->addEventPort("FRIState", fri_state_port).doc("Gets information about the FRI state");
		this->ports()->addPort("RobotState", robot_state_port).doc("Gets information about the robot state");

		this->ports()->addPort("CurCartPosPort", cur_cart_pos_port).doc("Gets information about the pose of the tool");

		// Output ports
		this->ports()->addPort("CartPosPort", cart_pos_port).doc("Publish Cartesian positions");
		this->ports()->addPort("JointImpedancePort", joint_impedance_port).doc("Publish joint impedances");
		this->ports()->addPort("CartImpedancePort", cart_impedance_port).doc("Publish Cartesian impedances");
		this->ports()->addPort("CartForcePort", cart_force_port).doc("Publish Cartesian forces");

		this->addProperty( "TrajBagFile", traj_bag_file).doc("Filename of the trajectory bag");

		/*this->ports()->addPort("JointsPos_ROSPort", joints_pos_ros_port).doc("Receive joint positions");
		this->ports()->addPort("Pose_ROSPort", pose_ros_port).doc("Receive pose of the tool");
		this->ports()->addPort("Force_ROSPort", force_ros_port).doc("Receive external force");*/

		for(auto i = 0; i < 7; i++) {
			gravity_compensation_joints.damping[i] = 0.7;
			gravity_compensation_joints.stiffness[i] = 0.01;
		}

		gravity_compensation_cart.damping.angular.x = DAMPENING;
		gravity_compensation_cart.damping.angular.y = DAMPENING;
		gravity_compensation_cart.damping.angular.z = DAMPENING;
		gravity_compensation_cart.damping.linear.x = DAMPENING;
		gravity_compensation_cart.damping.linear.y = DAMPENING;
		gravity_compensation_cart.damping.linear.z = DAMPENING;
		gravity_compensation_cart.stiffness.angular.x = STIFFNESS;
		gravity_compensation_cart.stiffness.angular.y = STIFFNESS;
		gravity_compensation_cart.stiffness.angular.z = STIFFNESS;
		gravity_compensation_cart.stiffness.linear.x = STIFFNESS;
		gravity_compensation_cart.stiffness.linear.y = STIFFNESS;
		gravity_compensation_cart.stiffness.linear.z = STIFFNESS;

		initial_stiffness.damping.angular.x = INIT_DAMPENING;
		initial_stiffness.damping.angular.y = INIT_DAMPENING;
		initial_stiffness.damping.angular.z = INIT_DAMPENING;
		initial_stiffness.damping.linear.x = INIT_DAMPENING;
		initial_stiffness.damping.linear.y = INIT_DAMPENING;
		initial_stiffness.damping.linear.z = INIT_DAMPENING;
		initial_stiffness.stiffness.angular.x = INIT_STIFFNESS;
		initial_stiffness.stiffness.angular.y = INIT_STIFFNESS;
		initial_stiffness.stiffness.angular.z = INIT_STIFFNESS;
		initial_stiffness.stiffness.linear.x = INIT_STIFFNESS;
		initial_stiffness.stiffness.linear.y = INIT_STIFFNESS;
		initial_stiffness.stiffness.linear.z = INIT_STIFFNESS;

		log(Info) << "FRI_ROS_Publisher constructed !" << endlog();
	}

	bool FRI_ROS_Receiver::configureHook(){
		Logger::In in((this->getName()));

		if(!fri_state_port.connected()) {
			log(Error) << "FRI state port not connected. Stopping..." << endlog();
			return false;
		}

		if(!robot_state_port.connected()) {
			log(Error) << "Robot state port not connected. Stopping..." << endlog();
			return false;
		}

		if(!cart_pos_port.connected()) {
			log(Error) << "Joints port not connected. Stopping..." << endlog();
			return false;
		}

		if(!joint_impedance_port.connected()) {
			log(Error) << "Pose port not connected. Stopping..." << endlog();
			return false;
		}

		if(!cart_impedance_port.connected()) {
			log(Error) << "Force port not connected. Stopping..." << endlog();
			return false;
		}

		readTrajBag();

		if(poses.size() == 0) {
			log(Error) << "There is no trajectory to replay, stopping..." << endlog();
			return false;
		}

		log(Info) << "FRI_ROS_Publisher configured !" << endlog();
		return true;
	}

	bool FRI_ROS_Receiver::startHook(){
		Logger::In in((this->getName()));

		log(Info) << "FRI_ROS_Publisher started !" << endlog();
		return true;
	}

	void FRI_ROS_Receiver::updateHook(){
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
					log(Info) << "Please switch to Joint Impedance Mode to replay the trajectory" << endlog();
					last_monitor_out_time = current_fri_state.timestamp;
				}
				return;
			}


			static unsigned int index = 0;
			static bool started = false;
			if(!started) {
				log(Info) << "Starting to replay trajectory" << endlog();
				log(Info) << "There are" << endlog();
				log(Info) << "* " << poses.size() << " poses" << endlog();
				log(Info) << "* " << joint_states.size() << " joint states" << endlog();
				log(Info) << "* " << forces.size() << " forces" << endlog();
				log(Info) << "Moving to initial position" << endlog();
				started = true;
			}

			if(index == 0) {
				cart_impedance_port.write(initial_stiffness);
				cur_cart_pos_port.read(current_pose);
				geometry_msgs::Pose next_pose = poses[0];
				tf::Vector3 cur_pos(current_pose.position.x, current_pose.position.y, current_pose.position.z);
				tf::Vector3 cur_rot(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
				tf::Vector3 delta_pos(next_pose.position.x - cur_pos.x(), next_pose.position.y - cur_pos.y(), next_pose.position.z - cur_pos.z());
				tf::Vector3 delta_rot(next_pose.orientation.x - cur_rot.x(), next_pose.orientation.y - cur_rot.y(), next_pose.orientation.z - cur_rot.z());

				log(Info) << "Abs Pos Diff: " << delta_pos.length() << " Abs Rot diff: " << delta_rot.length() << endlog();

				double max_pos_delta = current_fri_state.desiredCmdSampleTime * MAX_SPEED;
				double max_rot_delta = current_fri_state.desiredCmdSampleTime * MAX_TURN_SPEED;
				double abs_pos_diff = delta_pos.length();
				double abs_rot_diff = delta_rot.length();

				if(abs_pos_diff < max_pos_delta && abs_rot_diff < max_rot_delta) {
					log(Info) << "Initial position reached." << endlog();
					index++;
					return;
				}

				log(Info) << "max_pos_delta: " << max_pos_delta << " abs_pos_diff: " << abs_pos_diff << endlog();
				tf::Vector3 next_pos = cur_pos + max_pos_delta * (delta_pos / abs_pos_diff);
				tf::Vector3 next_rot = cur_rot + max_rot_delta * (delta_rot / abs_rot_diff);

				log(Info) << "Cur Pos x: " << cur_pos.x() << " Next pos x: " << next_pose.position.x << " New next x: " << next_pos.x() << " Delta x: " << delta_pos.x() << endlog();
				log(Info) << "Cur Rot x: " << cur_rot.x() << " Next Rot x: " << next_pose.orientation.x << " New next x: " << next_rot.x() << " Delta x: " << delta_rot.x() << endlog();

				next_pose.position.x = next_pos.x();
				next_pose.position.y = next_pos.y();
				next_pose.position.z = next_pos.z();
				next_pose.orientation.x = next_rot.x();
				next_pose.orientation.y = next_rot.y();
				next_pose.orientation.z = next_rot.z();

				cart_pos_port.write(next_pose);
			} else if(index < poses.size()) {
				log(Info) << "Moving to next position" << endlog();
				cart_impedance_port.write(gravity_compensation_cart);
				cart_pos_port.write(poses[index]);
				cart_force_port.write(forces[index]);
				index++;
			} else {
				log(Info) << "Final position reached!" << endlog();
			}
		}

		//log(Info) << "FRI_ROS_Publisher executes updateHook !" << endlog();
	}

	void FRI_ROS_Receiver::stopHook() {
		//std::cout << "FRI_ROS_Publisher executes stopping !" <<std::endl;
	}

	void FRI_ROS_Receiver::cleanupHook() {
		//std::cout << "FRI_ROS_Publisher cleaning up !" <<std::endl;
	}

	void FRI_ROS_Receiver::readTrajBag() {
		traj_bag.open(traj_bag_file, rosbag::bagmode::Read);

		vector<string> topics({"/fri/Pose", "/fri/Force", "/fri/JointPos"});

		rosbag::View view(traj_bag, rosbag::TopicQuery(topics));

		poses.reserve(view.size() / 3);
		forces.reserve(view.size() / 3);
		joint_states.reserve(view.size() / 3);
		for(rosbag::MessageInstance const m : view)	{
			if(m.getTopic() == "/fri/Pose") {
				geometry_msgs::Pose::ConstPtr pose = m.instantiate<geometry_msgs::Pose>();
				if (pose != NULL) {
					poses.push_back(*pose);
				}
			}

			if(m.getTopic() == "/fri/Force") {
				geometry_msgs::Wrench::ConstPtr force = m.instantiate<geometry_msgs::Wrench>();
				if (force != NULL) {
					forces.push_back(*force);
				}
			}

			if(m.getTopic() == "/fri/JointPos") {
				sensor_msgs::JointState::ConstPtr joints = m.instantiate<sensor_msgs::JointState>();
				if (joints != NULL) {
					joint_states.push_back(*joints);
				}
			}
		}

		traj_bag.close();
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
ORO_CREATE_COMPONENT(iros::FRI_ROS_Receiver)
