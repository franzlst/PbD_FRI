#ifndef OROCOS_KUKACOMMANDER_COMPONENT_HPP
#define OROCOS_KUKACOMMANDER_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include "KUKACommander/get_fri_state.h"
#include "KUKACommander/get_fri_ctrl.h"
#include "KUKACommander/set_fri_ctrl.h"
#include "KUKACommander/set_cart_imp.h"
#include "KUKACommander/set_jnt_imp.h"
#include "KUKACommander/set_bool.h"
#include "KUKACommander/get_bool.h"
#include "KUKACommander/move_to_jnt_pos.h"
#include "KUKACommander/move_to_cart_pos.h"

#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

#include <geometry_msgs/Twist.h>

#include <boost/array.hpp>



#define FRI_TO_KRL_CTRL_MODE	14
#define FRI_TO_KRL_MON_CMD		15
#define FRI_FROM_KRL_COND		14
#define FRI_FROM_KRL_STATE		15

namespace iros {

	using namespace RTT;
	using namespace std;

	class KUKACommanderROS : public RTT::TaskContext{
	  public:
		KUKACommanderROS(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	  private:
		bool initOrocosInterface();
		bool initROSInterface();

		TaskContext* commander;
		ros::NodeHandle ROSnode;
		ros::CallbackQueue callbackQueue;

		ros::ServiceServer getCurrentStateService;
		ros::ServiceServer getCurrentControlModeService;
		ros::ServiceServer switchToMonitorStateService;
		ros::ServiceServer switchToCommandStateService;
		ros::ServiceServer stopCommunicationService;
		ros::ServiceServer setControlModeService;
		ros::ServiceServer setCartesianImpedanceService;
		ros::ServiceServer setJointImpedanceService;
		ros::ServiceServer activateGravityCompensationService;
		ros::ServiceServer moveToJointPositionService;
		ros::ServiceServer moveToCartesianPositionService;
		ros::ServiceServer stopMovementsService;
		ros::ServiceServer isMovingService;

		OperationCaller<FRI_STATE(void)> getCurrentState;
		OperationCaller<FRI_CTRL(void)> getCurrentControlMode;
		OperationCaller<void(void)> switchToMonitorState;
		OperationCaller<void(void)> switchToCommandState;
		OperationCaller<void(void)> stopCommunication;
		OperationCaller<void(FRI_CTRL)> setControlMode;
		OperationCaller<void(geometry_msgs::Twist, geometry_msgs::Twist)> setCartesianImpedance;
		OperationCaller<void(boost::array<float, 7>, boost::array<float, 7>)> setJointImpedance;
		OperationCaller<void(bool)> activateGravityCompensation;
		OperationCaller<bool(boost::array<double, 7>, double)> moveToJointPosition;
		OperationCaller<bool(geometry_msgs::Pose, double)> moveToCartesianPosition;
		OperationCaller<void(void)> stopMovements;
		OperationCaller<bool(void)> isMoving;


		bool getCurrentStateROS(KUKACommander::get_fri_state::Request&, KUKACommander::get_fri_state::Response&);
		bool getCurrentControlModeROS(KUKACommander::get_fri_ctrl::Request&, KUKACommander::get_fri_ctrl::Response&);
		bool switchToMonitorStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool switchToCommandStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool stopCommunicationROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool setControlModeROS(KUKACommander::set_fri_ctrl::Request&, KUKACommander::set_fri_ctrl::Response&);
		bool setCartesianImpedanceROS(KUKACommander::set_cart_imp::Request&, KUKACommander::set_cart_imp::Response&);
		bool setJointImpedanceROS(KUKACommander::set_jnt_imp::Request&, KUKACommander::set_jnt_imp::Response&);
		bool activateGravityCompensationROS(KUKACommander::set_bool::Request&, KUKACommander::set_bool::Response&);
		bool moveToJointPositionROS(KUKACommander::move_to_jnt_pos::Request&, KUKACommander::move_to_jnt_pos::Response&);
		bool moveToCartesianPositionROS(KUKACommander::move_to_cart_pos::Request&, KUKACommander::move_to_cart_pos::Response&);
		bool stopMovementsROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool isMovingROS(KUKACommander::get_bool::Request&, KUKACommander::get_bool::Response&);
	};

	inline bool KUKACommanderROS::getCurrentStateROS(KUKACommander::get_fri_state::Request&, KUKACommander::get_fri_state::Response& response) {
		log(Debug) << "Call service getCurrentState" << endlog();
		response.state = getCurrentState();
		return true;
	}

	inline bool KUKACommanderROS::getCurrentControlModeROS(KUKACommander::get_fri_ctrl::Request&, KUKACommander::get_fri_ctrl::Response& response) {
		log(Debug) << "Call service getCurrentControlMode" << endlog();
		response.control = getCurrentControlMode();
		return true;
	}

	inline bool KUKACommanderROS::switchToMonitorStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		log(Debug) << "Call service switchToMonitorState" << endlog();
		switchToMonitorState();
		return true;
	}

	inline bool KUKACommanderROS::switchToCommandStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		log(Debug) << "Call service switchToCommandState" << endlog();
		switchToCommandState();
		return true;
	}

	inline bool KUKACommanderROS::stopCommunicationROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		log(Debug) << "Call service stopCommunication" << endlog();
		stopCommunication();
		return true;
	}

	inline bool KUKACommanderROS::setControlModeROS(KUKACommander::set_fri_ctrl::Request& request, KUKACommander::set_fri_ctrl::Response&) {
		log(Debug) << "Call service setControlMode" << endlog();
		setControlMode((FRI_CTRL) request.control);
		return true;
	}

	inline bool KUKACommanderROS::setCartesianImpedanceROS(KUKACommander::set_cart_imp::Request& request, KUKACommander::set_cart_imp::Response&) {
		log(Debug) << "Call service setCartesianImpedance" << endlog();
		setCartesianImpedance(request.stiffness, request.damping);
		return true;
	}

	inline bool KUKACommanderROS::setJointImpedanceROS(KUKACommander::set_jnt_imp::Request& request, KUKACommander::set_jnt_imp::Response&) {
		log(Debug) << "Call service setJointImpedance" << endlog();
		setJointImpedance(request.stiffness, request.damping);
		return true;
	}

	/*inline bool KUKACommanderROS::setCartesianImpedanceROS(KUKACommander::set_cart_imp::Request& request, KUKACommander::set_cart_imp::Response&) {
		log(Debug) << "Call service setCartesianImpedance" << endlog();
		setCartesianImpedance(request.stiffness, request.damping);
		return true;
	}*/

	inline bool KUKACommanderROS::activateGravityCompensationROS(KUKACommander::set_bool::Request& request, KUKACommander::set_bool::Response&) {
		log(Debug) << "Call service activateGravityCompensation" << endlog();
		activateGravityCompensation(request.activate);
		return true;
	}

	inline bool KUKACommanderROS::moveToJointPositionROS(KUKACommander::move_to_jnt_pos::Request& request, KUKACommander::move_to_jnt_pos::Response& response) {
		log(Debug) << "Call service moveToJointPosition" << endlog();
		response.success = moveToJointPosition(request.position, request.time);
		return true;
	}

	inline bool KUKACommanderROS::moveToCartesianPositionROS(KUKACommander::move_to_cart_pos::Request& request, KUKACommander::move_to_cart_pos::Response& response) {
		log(Debug) << "Call service moveToCartesianPosition" << endlog();
		response.success = moveToCartesianPosition(request.position, request.time);
		return true;
	}

	inline bool KUKACommanderROS::stopMovementsROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		log(Debug) << "Call service stopMovements" << endlog();
		stopMovements();
		return true;
	}

	inline bool KUKACommanderROS::isMovingROS(KUKACommander::get_bool::Request&, KUKACommander::get_bool::Response& response) {
		log(Debug) << "Call service isMoving" << endlog();
		response.answer = isMoving();
		return true;
	}

}
#endif
