#ifndef OROCOS_FRI_ROS_PUBLISHER_COMPONENT_HPP
#define OROCOS_FRI_ROS_PUBLISHER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <kuka_lwr_fri/typekit/Types.hpp>
#include <lwr_fri/typekit/Types.hpp>

namespace iros {

using namespace RTT;
using namespace std;


	class FRI_ROS_Publisher : public RTT::TaskContext{
	  public:
		FRI_ROS_Publisher(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	  private:

	    InputPort<tFriRobotState> robot_state_port;
		InputPort<tFriIntfState> fri_state_port;

		tFriRobotState current_robot_state;
		tFriIntfState current_fri_state;

		InputPort< sensor_msgs::JointState >  joints_pos_port;
		InputPort< geometry_msgs::Pose > pose_port;
		InputPort< geometry_msgs::Wrench > force_port;

		Property<bool> fts_publish;

		OutputPort<lwr_fri::FriJointImpedance> joint_impedance_port;
		OutputPort<lwr_fri::CartesianImpedance> cart_impedance_port;

		OutputPort< sensor_msgs::JointState >  joints_pos_ros_port;
		OutputPort< geometry_msgs::Pose > pose_ros_port;
		OutputPort< geometry_msgs::Wrench > force_ros_port;

		OutputPort<bool> publishing_event;

		lwr_fri::FriJointImpedance gravity_compensation_joints;
		lwr_fri::CartesianImpedance gravity_compensation_cart;

		sensor_msgs::JointState current_joints_pos;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Wrench current_force;


	};

}
#endif
