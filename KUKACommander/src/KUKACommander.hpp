#ifndef OROCOS_KUKACOMMANDER_COMPONENT_HPP
#define OROCOS_KUKACOMMANDER_COMPONENT_HPP

#include <rtt/RTT.hpp>

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

	class KUKACommander : public RTT::TaskContext{
	  public:
		KUKACommander(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		/**
		 * Switch to monitor state
		 * Calls friStop() on the KRC
		 */
		void switchToMonitorState();

		/**
		 * Switch to command state
		 * Calls friStart() on the KRC
		 */
		void switchToCommandState();

		/**
		 * Stop communication
		 * Calls friClose() on the KRC
		 */
		void stopCommunication();

		/**
		 * Get the current state
		 * Get the current state of the FRI interface
		 * @return The current state of the FRI interface
		 * @see ::FRI_STATE
		 */
		FRI_STATE getCurrentState();

		/**
		 * Get the current control mode
		 * @return The current control mode
		 * @see ::FRI_CTRL
		 */
		FRI_CTRL getCurrentControlMode();

		/**
		 * Changes the control mode
		 * Changes the control mode to mode. If the current state is control state, it first
		 * switches to monitor state and afterwards automatically switches back
		 * @param mode Desired control mode
		 */
		void setControlMode(FRI_CTRL mode = FRI_CTRL_OTHER);

	  private:
		InputPort<tFriKrlData> port_from_krl;
		OutputPort<tFriKrlData> port_to_krl;

		InputPort<tFriRobotState> port_robot_state;
		InputPort<tFriIntfState> port_fri_state;

		tFriKrlData data_from_krl;
		tFriKrlData data_to_krl;

		tFriRobotState data_robot_state;
		tFriIntfState data_fri_state;

		bool call_set_ctrl_mode;
	};

}
#endif
