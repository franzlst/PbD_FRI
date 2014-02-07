#ifndef OROCOS_FRI_ROS_RECEIVER_COMPONENT_HPP
#define OROCOS_FRI_ROS_RECEIVER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <kuka_lwr_fri/typekit/Types.hpp>
#include <lwr_fri/typekit/Types.hpp>
#include "rosbag/bag.h"

namespace iros {

using namespace RTT;
using namespace std;

#define DAMPENING 0.7
#define STIFFNESS 0.01
#define INIT_DAMPENING 0.5
#define INIT_STIFFNESS 100
#define MAX_SPEED 5
#define MAX_TURN_SPEED 1

	class FRI_ROS_Receiver : public RTT::TaskContext{
	  public:
		FRI_ROS_Receiver(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		string traj_bag_file;

	  private:
		void readTrajBag();

		InputPort<tFriRobotState> robot_state_port;
		InputPort<tFriIntfState> fri_state_port;
		//InputPort<sensor_msgs::JointState> joint_state_port;
		InputPort<geometry_msgs::Pose> cur_cart_pos_port;

		tFriRobotState current_robot_state;
		tFriIntfState current_fri_state;

		OutputPort<geometry_msgs::Pose> cart_pos_port;
		OutputPort<lwr_fri::FriJointImpedance> joint_impedance_port;
		OutputPort<lwr_fri::CartesianImpedance> cart_impedance_port;
		OutputPort<geometry_msgs::Wrench> cart_force_port;

		/*InputPort< sensor_msgs::JointState >  joints_pos_ros_port;
		InputPort< geometry_msgs::Pose > pose_ros_port;
		InputPort< geometry_msgs::Wrench > force_ros_port;*/

		lwr_fri::FriJointImpedance gravity_compensation_joints;
		lwr_fri::CartesianImpedance initial_stiffness;
		lwr_fri::CartesianImpedance gravity_compensation_cart;

		sensor_msgs::JointState current_joints_pos;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Wrench current_force;

		vector<sensor_msgs::JointState> joint_states;
		vector<geometry_msgs::Pose> poses;
		vector<geometry_msgs::Wrench> forces;

		rosbag::Bag traj_bag;
	};

}
#endif
