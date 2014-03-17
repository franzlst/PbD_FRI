// Copyright  (C)  2009  Ruben Smits <ruben dot_cart smits at mech dot kuleuven dot be>,
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

#include <rtt/Component.hpp>

#include <rtt/Logger.hpp>
#include <kdl/frames.hpp>

#include <iostream>
#include <sys/types.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <rtdm/rtdm.h>
#include <fcntl.h>

#include "FRIServerRT.hpp"
#include <tf_conversions/tf_kdl.h>


namespace lwr_fri {

using namespace RTT;
using namespace std;

FRIServerRT::FRIServerRT(const string& name) :
	TaskContext(name, PreOperational){

	this->addPort("fromKRL", m_fromKRLPort);
	this->addPort("toKRL", m_toKRLPort);

	this->addPort("events", m_events).doc(
			"Port through which discrete events are emitted");

	this->addPort("RobotState", m_RobotStatePort).doc(
			"Port containing the status of the robot");
	this->addPort("FRIState", m_FriStatePort).doc(
			"Port containing the status of the FRI communication");
	this->addPort("msrJointState", m_jointStatePort);
	this->addPort("msrFriJointState", m_friJointStatePort);
	this->addPort("msrCartPos", m_msrCartPosPort);
	this->addPort("msrExtCartWrench", m_estExtTcpWrenchPort);
	this->addPort("cmdCartPos", m_cmdCartPosPort);
	this->addPort("cmdCartPosFriOffset", m_cmdCartPosFriOffsetPort);
	this->addPort("desJntPos", m_jntPosPort);
	this->addPort("desJntVel", m_jntVelPort);
	this->addPort("desCartPos", m_cartPosPort);
	this->addPort("desCartTwist", m_cartTwistPort);
	this->addPort("desAddJntTrq", m_addJntTrqPort);
	this->addPort("desAddTcpWrench", m_addTcpWrenchPort);
	this->addPort("desJntImpedance", m_jntImpedancePort);
	this->addPort("desCartImpedance", m_cartImpedancePort);

	this->addProperty("udp_port", m_local_port);

	memset(&m_msr_data, 0, sizeof(m_msr_data) );
	memset(&m_cmd_data, 0, sizeof(m_cmd_data) );
}

FRIServerRT::~FRIServerRT() {
}

bool FRIServerRT::configureHook() {
	//Check the sizes of all data:
	if (!FRI_CHECK_SIZES_OK) {
		log(Error) << "Padding on this platform is not OK :(" << endlog();
		return false;
	}
	//Check the byte order and float representation:
	{
		FRI_PREPARE_CHECK_BYTE_ORDER;
		if (!FRI_CHECK_BYTE_ORDER_OK) {
			log(Error)
					<< "Byte order and float representations are not OK on this platform :("
					<< endlog();
			return false;
		}
	}

	// presize the events port
	m_events.setDataSample(FRI_STATE_INVALID);
	fri_state_last = FRI_STATE_INVALID;

	//Resizing dynamic size objects
	m_jointStates.name.resize(LBR_MNJ);
	m_jointStates.position.resize(LBR_MNJ);
	m_jointStates.effort.resize(LBR_MNJ);
	m_jointStates.header.frame_id = "dummy_frame_id";
	m_friJointState.header.frame_id = "dummy_frame_id";

	jacobianPort.setDataSample(m_jac);

	for (unsigned int i = 0; i < LBR_MNJ; i++) {
		std::ostringstream ss;
		ss << "arm_" << i+1 <<"_joint";
		m_jointStates.name[i] = ss.str();
	}
	m_jointStatePort.setDataSample(m_jointStates);

	provides()->addAttribute("counter", counter);

	return connect_to_krc();
}

bool FRIServerRT::startHook() {
	counter = 0;
	m_init = true;
	return true;
}

void FRIServerRT::updateHook() {
	// Read data to m_msr_data
	if(read_from_krc()) {

		// Call event when the state has switched
		if (fri_state_last != m_msr_data.intf.state) {
			if(m_msr_data.intf.state == FRI_STATE_MON)
				m_events.write(FRI_STATE_MON);
			else if(m_msr_data.intf.state == FRI_STATE_CMD) 
				m_events.write(FRI_STATE_CMD);
			else
				m_events.write(FRI_STATE_OFF);
			fri_state_last = m_msr_data.intf.state;
		}

		m_RobotStatePort.write(m_msr_data.robot);
		m_FriStatePort.write(m_msr_data.intf);

		// Fill in fri_joint_state and joint_state
		for (unsigned int i = 0; i < LBR_MNJ; i++) {
			m_friJointState.msrJntPos[i] = m_msr_data.data.msrJntPos[i];
			m_friJointState.cmdJntPos[i] = m_msr_data.data.cmdJntPos[i];
			m_friJointState.cmdJntPosFriOffset[i] = m_msr_data.data.cmdJntPosFriOffset[i];
			m_friJointState.msrJntTrq[i] = m_msr_data.data.msrJntTrq[i];
			m_friJointState.estExtJntTrq[i] = m_msr_data.data.estExtJntTrq[i];
		}
		m_jointStates.position.assign(m_msr_data.data.msrJntPos,
				m_msr_data.data.msrJntPos + LBR_MNJ);
		m_jointStates.effort.assign(m_msr_data.data.estExtJntTrq,
				m_msr_data.data.estExtJntTrq + LBR_MNJ);

		m_jointStates.header.stamp.fromNSec ( RTT::os::TimeService::Instance()->getNSecs() );
		//m_joint_states.header.stamp.fromSec( m_msr_data.intf.timestamp ); --> only accurate to 1/10th of a second !!!
		m_friJointState.header.stamp=m_jointStates.header.stamp;
		m_friJointStatePort.write(m_friJointState);
		m_jointStatePort.write(m_jointStates);

		//Put KRL data onto the ports(no parsing)
		m_fromKRL = m_msr_data.krl;
		m_fromKRLPort.write(m_fromKRL);

		// Create a Pose from the measured Cartesian position
		geometry_msgs::Quaternion quat;
		KDL::Frame cartPos;
		cartPos.M=KDL::Rotation(m_msr_data.data.msrCartPos[0],
				m_msr_data.data.msrCartPos[1], m_msr_data.data.msrCartPos[2],
				m_msr_data.data.msrCartPos[4], m_msr_data.data.msrCartPos[5],
				m_msr_data.data.msrCartPos[6], m_msr_data.data.msrCartPos[8],
				m_msr_data.data.msrCartPos[9], m_msr_data.data.msrCartPos[10]);
		cartPos.p.x(m_msr_data.data.msrCartPos[3]);
		cartPos.p.y(m_msr_data.data.msrCartPos[7]);
		cartPos.p.z(m_msr_data.data.msrCartPos[11]);
		tf::poseKDLToMsg(cartPos,m_cartPos);
		m_msrCartPosPort.write(m_cartPos);

		// Create a Pose from the commanded Cartesian position
		cartPos.M = KDL::Rotation(m_msr_data.data.cmdCartPos[0],
				m_msr_data.data.cmdCartPos[1], m_msr_data.data.cmdCartPos[2],
				m_msr_data.data.cmdCartPos[4], m_msr_data.data.cmdCartPos[5],
				m_msr_data.data.cmdCartPos[6], m_msr_data.data.cmdCartPos[8],
				m_msr_data.data.cmdCartPos[9], m_msr_data.data.cmdCartPos[10]);
		cartPos.p.x(m_msr_data.data.cmdCartPos[3]);
		cartPos.p.y(m_msr_data.data.cmdCartPos[7]);
		cartPos.p.z(m_msr_data.data.cmdCartPos[11]);
		tf::poseKDLToMsg(cartPos,m_cartPos);
		m_cmdCartPosPort.write(m_cartPos);

		// Create a Pose from the commanded Cartesian position offset
		cartPos.M = KDL::Rotation(m_msr_data.data.cmdCartPosFriOffset[0],
				m_msr_data.data.cmdCartPosFriOffset[1], m_msr_data.data.cmdCartPosFriOffset[2],
				m_msr_data.data.cmdCartPosFriOffset[4],	m_msr_data.data.cmdCartPosFriOffset[5],
				m_msr_data.data.cmdCartPosFriOffset[6],	m_msr_data.data.cmdCartPosFriOffset[8],
				m_msr_data.data.cmdCartPosFriOffset[9],	m_msr_data.data.cmdCartPosFriOffset[10]);
		cartPos.p.x(m_msr_data.data.cmdCartPosFriOffset[3]);
		cartPos.p.y(m_msr_data.data.cmdCartPosFriOffset[7]);
		cartPos.p.z(m_msr_data.data.cmdCartPosFriOffset[11]);
		tf::poseKDLToMsg(cartPos,m_cartPos);
		m_cmdCartPosFriOffsetPort.write(m_cartPos);

		// Get the Jacobian
		/*for ( int i = 0; i < FRI_CART_VEC; i++)
			for ( int j = 0; j < LBR_MNJ; j++)
				m_jac(i,j) = m_msr_data.data.jacobian[i*LBR_MNJ+j];
		//KUKA uses Tx, Ty, Tz, Rz, Ry, Rx convention, so we need to swap Rz and Rx
		m_jac.data.row(3).swap(m_jac.data.row(5));
		jacobianPort.write(m_jac);*/


		// Store external Cartesian force/torque
		m_cartWrench.force.x = m_msr_data.data.estExtTcpFT[0];
		m_cartWrench.force.y = m_msr_data.data.estExtTcpFT[1];
		m_cartWrench.force.z = m_msr_data.data.estExtTcpFT[2];
		m_cartWrench.torque.x = m_msr_data.data.estExtTcpFT[5];
		m_cartWrench.torque.y = m_msr_data.data.estExtTcpFT[4];
		m_cartWrench.torque.z = m_msr_data.data.estExtTcpFT[3];
		m_estExtTcpWrenchPort.write(m_cartWrench);

		//Fill in datagram to send:
		m_cmd_data.head.datagramId = FRI_DATAGRAM_ID_CMD;
		m_cmd_data.head.packetSize = sizeof(tFriCmdData);
		m_cmd_data.head.sendSeqCount = ++counter;
		m_cmd_data.head.reflSeqCount = m_msr_data.head.sendSeqCount;

		if (!isPowerOn()) {
			// necessary to write cmd if not powered on. See kuka FRI user manual p6 and friremote.cpp:
			for (int i = 0; i < LBR_MNJ; i++) {
				m_cmd_data.cmd.jntPos[i]=m_msr_data.data.cmdJntPos[i]+m_msr_data.data.cmdJntPosFriOffset[i];
			}
		}


		if (m_msr_data.intf.state == FRI_STATE_MON || !isPowerOn()) {
			// joint position control capable modes:
			if (m_msr_data.robot.control == FRI_CTRL_POSITION
					|| m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS; // Put together binary mask
				for (unsigned int i = 0; i < LBR_MNJ; i++) {
					// see note above with !isPowerOn()
					// the user manual speaks of 'mimic msr.data.msrCmdJntPos' which is ambiguous.
					// on the other hand, the friremote.cpp will send this whenever (!isPowerOn() || state != FRI_STATE_CMD)
					// so we mimic the kuka reference code here...
					m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i]+m_msr_data.data.cmdJntPosFriOffset[i];
				}
			}
			// Default values (impedance, force) and additional flags for joint impedance mode
			if ( m_msr_data.robot.control == FRI_CTRL_JNT_IMP ) {
				m_cmd_data.cmd.cmdFlags |= FRI_CMD_JNTTRQ | FRI_CMD_JNTSTIFF | FRI_CMD_JNTDAMP;
				for (unsigned int i = 0; i < LBR_MNJ; i++) {
					// TODO: Default impedance parameters
					m_cmd_data.cmd.addJntTrq[i] = 0.0;
					m_cmd_data.cmd.jntStiffness[i] = 250;
					m_cmd_data.cmd.jntDamping[i]   = 0.7;
				}
			}
			// Default values (impedance, force) and additional flags for Cartesian impedance mode
			if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS | FRI_CMD_TCPFT;
				m_cmd_data.cmd.cmdFlags |= FRI_CMD_CARTSTIFF | FRI_CMD_CARTDAMP;
				for (unsigned int i = 0; i < FRI_CART_FRM_DIM; i++)
					m_cmd_data.cmd.cartPos[i] = m_msr_data.data.msrCartPos[i];
				for(unsigned int i = 0 ; i < FRI_CART_VEC ; i++)
					m_cmd_data.cmd.addTcpFT[i]=0.0;
				for(unsigned int i=0; i < FRI_CART_VEC/2 ; i++)	{
					// TODO: Default impedance parameters
					//Linear part;
					m_cmd_data.cmd.cartStiffness[i]=100;
					m_cmd_data.cmd.cartDamping[i]=0.1;
					//rotational part;
					m_cmd_data.cmd.cartStiffness[i+FRI_CART_VEC/2]=10;
					m_cmd_data.cmd.cartDamping[i+FRI_CART_VEC/2]=0.1;
				}
			}
		}

		//Only send if state is in FRI_STATE_CMD and drives are powered
		if ( (m_msr_data.intf.state == FRI_STATE_CMD) && isPowerOn() ) {

			//Valid ports in joint position and joint impedance mode
			if (m_msr_data.robot.control == FRI_CTRL_POSITION
					|| m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
				//Read desired positions
				if (NewData == m_jntPosPort.read(m_jntPos)) {
					if (m_jntPos.positions.size() == LBR_MNJ) {
						for (unsigned int i = 0; i < LBR_MNJ; i++)
							m_cmd_data.cmd.jntPos[i] = m_jntPos.positions[i];
					} else {
						log(Error) << "Wrong number of joint positions: " << m_jntPos.positions.size() << endlog();
					}
				}

				if (NewData == m_jntVelPort.read(m_jntVel)) {
					if (m_jntVel.velocities.size() == LBR_MNJ) {
						for (unsigned int i = 0; i < LBR_MNJ; i++)
							m_cmd_data.cmd.jntPos[i] += m_jntVel.velocities[i]*m_msr_data.intf.desiredCmdSampleTime;
					} else {
						log(Error) << "Wrong number of joint velocities: " << m_jntVel.velocities.size() << endlog();
					}
				}
			}

			//Valid ports only in joint impedance mode
			if (m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
				//Read desired additional joint torques
				if (NewData == m_addJntTrqPort.read(m_jntTorques)) {
					if (m_jntTorques.efforts.size() == LBR_MNJ) {
						for (unsigned int i = 0; i < LBR_MNJ; i++)
							m_cmd_data.cmd.addJntTrq[i]	= m_jntTorques.efforts[i];
					} else {
						log(Error) << "Wrong number of joint torques: " << m_jntTorques.efforts.size() << endlog();
					}
				}

				//Read desired joint impedance
				if (m_jntImpedancePort.read(m_jntImpedance)
						== NewData) {
					for (unsigned int i = 0; i < LBR_MNJ; i++) {
						m_cmd_data.cmd.jntStiffness[i] = m_jntImpedance.stiffness[i];
						m_cmd_data.cmd.jntDamping[i] = m_jntImpedance.damping[i];
					}
				}

			//Valid ports only in Cartesian impedance mode
			} else if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
				if (NewData == m_cartPosPort.read(m_cartPos)) {
					KDL::Rotation rot =	KDL::Rotation::Quaternion(
						m_cartPos.orientation.x, m_cartPos.orientation.y,
						m_cartPos.orientation.z, m_cartPos.orientation.w);
					m_cmd_data.cmd.cartPos[0] = rot.data[0];
					m_cmd_data.cmd.cartPos[1] = rot.data[1];
					m_cmd_data.cmd.cartPos[2] = rot.data[2];
					m_cmd_data.cmd.cartPos[4] = rot.data[3];
					m_cmd_data.cmd.cartPos[5] = rot.data[4];
					m_cmd_data.cmd.cartPos[6] = rot.data[5];
					m_cmd_data.cmd.cartPos[8] = rot.data[6];
					m_cmd_data.cmd.cartPos[9] = rot.data[7];
					m_cmd_data.cmd.cartPos[10] = rot.data[8];

					m_cmd_data.cmd.cartPos[3] = m_cartPos.position.x;
					m_cmd_data.cmd.cartPos[7] = m_cartPos.position.y;
					m_cmd_data.cmd.cartPos[11] = m_cartPos.position.z;
				}

				if (NewData == m_addTcpWrenchPort.read(m_cartWrench)) {
					m_cmd_data.cmd.addTcpFT[0] = m_cartWrench.force.x;
					m_cmd_data.cmd.addTcpFT[1] = m_cartWrench.force.y;
					m_cmd_data.cmd.addTcpFT[2] = m_cartWrench.force.z;
					m_cmd_data.cmd.addTcpFT[3] = m_cartWrench.torque.z;
					m_cmd_data.cmd.addTcpFT[4] = m_cartWrench.torque.y;
					m_cmd_data.cmd.addTcpFT[5] = m_cartWrench.torque.x;
				}

				if (NewData == m_cartTwistPort.read(m_cartTwist)) {
					KDL::Twist t;
					tf::twistMsgToKDL (m_cartTwist, t);
					KDL::Frame T_old;
					T_old.M = KDL::Rotation(m_cmd_data.cmd.cartPos[0],
							m_cmd_data.cmd.cartPos[1],
							m_cmd_data.cmd.cartPos[2],
							m_cmd_data.cmd.cartPos[4],
							m_cmd_data.cmd.cartPos[5],
							m_cmd_data.cmd.cartPos[6],
							m_cmd_data.cmd.cartPos[8],
							m_cmd_data.cmd.cartPos[9],
							m_cmd_data.cmd.cartPos[10]);
					T_old.p.x(m_cmd_data.cmd.cartPos[3]);
					T_old.p.y(m_cmd_data.cmd.cartPos[7]);
					T_old.p.z(m_cmd_data.cmd.cartPos[11]);

					KDL::Frame T_new = addDelta (T_old, t, m_msr_data.intf.desiredCmdSampleTime);

					m_cmd_data.cmd.cartPos[0] = T_new.M.data[0];
					m_cmd_data.cmd.cartPos[1] = T_new.M.data[1];
					m_cmd_data.cmd.cartPos[2] = T_new.M.data[2];
					m_cmd_data.cmd.cartPos[4] = T_new.M.data[3];
					m_cmd_data.cmd.cartPos[5] = T_new.M.data[4];
					m_cmd_data.cmd.cartPos[6] = T_new.M.data[5];
					m_cmd_data.cmd.cartPos[8] = T_new.M.data[6];
					m_cmd_data.cmd.cartPos[9] = T_new.M.data[7];
					m_cmd_data.cmd.cartPos[10] = T_new.M.data[8];
					m_cmd_data.cmd.cartPos[3] = T_new.p.x();
					m_cmd_data.cmd.cartPos[7] = T_new.p.y();
					m_cmd_data.cmd.cartPos[11] = T_new.p.z();
				}
				if (NewData == m_cartImpedancePort.read(m_cartImpedance)){
					m_cmd_data.cmd.cartStiffness[0]=m_cartImpedance.stiffness.linear.x;
					m_cmd_data.cmd.cartStiffness[1]=m_cartImpedance.stiffness.linear.y;
					m_cmd_data.cmd.cartStiffness[2]=m_cartImpedance.stiffness.linear.z;
					m_cmd_data.cmd.cartStiffness[5]=m_cartImpedance.stiffness.angular.x;
					m_cmd_data.cmd.cartStiffness[4]=m_cartImpedance.stiffness.angular.y;
					m_cmd_data.cmd.cartStiffness[3]=m_cartImpedance.stiffness.angular.z;
					m_cmd_data.cmd.cartDamping[0]=m_cartImpedance.damping.linear.x;
					m_cmd_data.cmd.cartDamping[1]=m_cartImpedance.damping.linear.y;
					m_cmd_data.cmd.cartDamping[2]=m_cartImpedance.damping.linear.z;
					m_cmd_data.cmd.cartDamping[5]=m_cartImpedance.damping.angular.x;
					m_cmd_data.cmd.cartDamping[4]=m_cartImpedance.damping.angular.y;
					m_cmd_data.cmd.cartDamping[3]=m_cartImpedance.damping.angular.z;
				}
			} else if (m_msr_data.robot.control == FRI_CTRL_OTHER) {
				this->error();
			}
		}//End command mode

		m_toKRLPort.read(m_toKRL);
		m_cmd_data.krl = m_toKRL;

		send_to_krc();

		if(m_init){
			rtos_enable_rt_warning();
			m_init = false;
		}
	}
	this->trigger();
}

void FRIServerRT::stopHook() {
}

void FRIServerRT::cleanupHook() {
	rt_dev_close(m_socket);
}

bool FRIServerRT::connect_to_krc() {
	m_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	rt_dev_setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(m_local_port);

	if (rt_dev_bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
		log(Error) << "Binding of port failed with errno " << errno << endlog();
		return false;
	}

	return true;
}

bool FRIServerRT::read_from_krc() {
	socklen_t addr_len = sizeof(m_remote_addr);
	int n = rt_dev_recvfrom(m_socket, (void*) &m_msr_data, sizeof(m_msr_data), 0,
			&m_remote_addr, &addr_len);
	if (sizeof(tFriMsrData) != n) {
		log(Error) << "Bad packet length: " << n << ", expected: "
				<< sizeof(tFriMsrData) << endlog();
		return false;
	}

	return true;
}

bool FRIServerRT::send_to_krc() {
	if (0 > rt_dev_sendto(m_socket, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
					(sockaddr*) &m_remote_addr, sizeof(m_remote_addr))) {
		log(Error) << "Sending datagram failed." << endlog();
		return false;
	}

	return true;
}

}//namespace LWR

ORO_CREATE_COMPONENT(lwr_fri::FRIServerRT)
