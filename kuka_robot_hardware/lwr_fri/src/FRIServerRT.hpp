// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>

// Author: Ruben Smits, Wilm Decre
// Maintainer: Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _FRI_SERVER_RT_HPP_
#define _FRI_SERVER_RT_HPP_

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

#include <kuka_lwr_fri/friComm.h>
#include <kuka_lwr_fri/typekit/Types.hpp>


namespace lwr_fri {

using namespace RTT;

class FRIServerRT: public RTT::TaskContext {
public:
	FRIServerRT(const std::string& name);
	~FRIServerRT();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

	bool isPowerOn() { return m_msr_data.robot.power!=0; }

private:
	bool connect_to_krc();
	bool read_from_krc();
	bool send_to_krc();

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

	sensor_msgs::JointState m_jointStates;
	lwr_fri::FriJointState m_friJointState;

	motion_control_msgs::JointPositions m_jntPos;
	motion_control_msgs::JointVelocities m_jntVel;
	motion_control_msgs::JointEfforts m_jntTorques;

	geometry_msgs::Pose m_cartPos;
	geometry_msgs::Twist m_cartTwist;
	geometry_msgs::Wrench m_cartWrench;

	tFriKrlData m_fromKRL;
	tFriKrlData m_toKRL;

    lwr_fri::FriJointImpedance m_jntImpedance;
    lwr_fri::CartesianImpedance m_cartImpedance;

	KDL::Jacobian m_jac;

	/**
	 * variables
	 */
	OutputPort<tFriKrlData> m_fromKRLPort;
	InputPort<tFriKrlData> m_toKRLPort;
	/**
	 * events
	 */
	OutputPort<uint16_t> m_events;

	/**
	 * statistics
	 */
	OutputPort<tFriRobotState> m_RobotStatePort;
	OutputPort<tFriIntfState> m_FriStatePort;

	/**
	 * Current robot data
	 */
	OutputPort<sensor_msgs::JointState> m_jointStatePort;
	OutputPort<lwr_fri::FriJointState> m_friJointStatePort;

	OutputPort<geometry_msgs::Pose>  m_msrCartPosPort;
	OutputPort<geometry_msgs::Pose>  m_cmdCartPosPort;
	OutputPort<geometry_msgs::Pose>  m_cmdCartPosFriOffsetPort;
    OutputPort<geometry_msgs::Wrench> m_estExtTcpWrenchPort;

    OutputPort<KDL::Jacobian> jacobianPort;

	InputPort<motion_control_msgs::JointPositions> m_jntPosPort;
	InputPort<motion_control_msgs::JointVelocities> m_jntVelPort;
	InputPort<geometry_msgs::Pose> m_cartPosPort;
	InputPort<geometry_msgs::Twist> m_cartTwistPort;
	InputPort<motion_control_msgs::JointEfforts> m_addJntTrqPort;
	InputPort<geometry_msgs::Wrench> m_addTcpWrenchPort;
	InputPort<lwr_fri::FriJointImpedance> m_jntImpedancePort;
	InputPort<lwr_fri::CartesianImpedance> m_cartImpedancePort;

	int m_local_port,m_socket,m_remote_port, m_control_mode;

	const char* m_remote_address;
	struct sockaddr m_remote_addr;
	uint16_t counter, fri_state_last;
	bool m_init;
};

}//Namespace LWR

#endif//_FRI_SERVER_RT_HPP_
