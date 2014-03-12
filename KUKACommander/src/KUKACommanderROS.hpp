#ifndef OROCOS_KUKACOMMANDER_COMPONENT_HPP
#define OROCOS_KUKACOMMANDER_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include "KUKACommander/get_fri_state.h"
#include "KUKACommander/get_fri_ctrl.h"
#include "KUKACommander/set_fri_ctrl.h"

#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>



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

		OperationCaller<FRI_STATE(void)> getCurrentState;
		OperationCaller<FRI_CTRL(void)> getCurrentControlMode;
		OperationCaller<void(void)> switchToMonitorState;
		OperationCaller<void(void)> switchToCommandState;
		OperationCaller<void(void)> stopCommunication;
		OperationCaller<void(FRI_CTRL)> setControlMode;


		bool getCurrentStateROS(KUKACommander::get_fri_state::Request&, KUKACommander::get_fri_state::Response&);
		bool getCurrentControlModeROS(KUKACommander::get_fri_ctrl::Request&, KUKACommander::get_fri_ctrl::Response&);
		bool switchToMonitorStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool switchToCommandStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool stopCommunicationROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
		bool setControlModeROS(KUKACommander::set_fri_ctrl::Request&, KUKACommander::set_fri_ctrl::Response&);
	};

	inline bool KUKACommanderROS::getCurrentStateROS(KUKACommander::get_fri_state::Request&, KUKACommander::get_fri_state::Response& response) {
		response.state = getCurrentState();
		return true;
	}

	inline bool KUKACommanderROS::getCurrentControlModeROS(KUKACommander::get_fri_ctrl::Request&, KUKACommander::get_fri_ctrl::Response& response) {
		response.control = getCurrentControlMode();
		return true;
	}

	inline bool KUKACommanderROS::switchToMonitorStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		switchToMonitorState();
		return true;
	}

	inline bool KUKACommanderROS::switchToCommandStateROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		switchToCommandState();
		return true;
	}

	inline bool KUKACommanderROS::stopCommunicationROS(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
		stopCommunication();
		return true;
	}

	inline bool KUKACommanderROS::setControlModeROS(KUKACommander::set_fri_ctrl::Request& request, KUKACommander::set_fri_ctrl::Response&) {
		setControlMode((FRI_CTRL) request.control);
		return true;
	}

}
#endif
