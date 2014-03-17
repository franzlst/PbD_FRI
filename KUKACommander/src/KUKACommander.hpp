#ifndef OROCOS_KUKACOMMANDER_COMPONENT_HPP
#define OROCOS_KUKACOMMANDER_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <boost/array.hpp>

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

		/**
		 * Sets the Cartesian impedance
		 * The Cartesian stiffness and damping each consist of linear x, y z and rotational x, y, z
		 * @param stiffness Cartesian stiffness
		 * @param damping Cartesian damping
		 */
		void setCartesianImpedance(geometry_msgs::Twist stiffness, geometry_msgs::Twist damping);

		/**
		 * Sets the Joint impedance
		 * The Joint stiffness and damping each consist of seven values
		 * @param stiffness Array of 7 stiffness parameters
		 * @param damping Array of 7 damping parameters
		 */
		void setJointImpedance(boost::array<float, 7> stiffness, boost::array<float, 7> damping);

		/**
		 * Switches into gravity compensation mode
		 * This is a "fake" gravity compensation, as it uses one of the two impedance control modes.
		 * The stiffness is set to 0 and the damping to critical damping.
		 * @param joint If joint is set to true (default), the joint impedance mode is used.
		 * Otherwise, Cartesian mode is used.
		 */
		void activateGravityCompensation(bool joint = true);

		/**
		 * Moves the arm to jointPos
		 * A trajectory in generated moving the arm within time to the desired joint position jointPos.
		 * If jointPos is not reachable within time (due to velocity limits), it is done as fast as possible.
		 * @param jointPos The desired joint position
		 * @param time The time (in seconds), the movement shall take
		 * @return False if there is an ongoing movement or an error occurs. True otherwise.
		 */
		bool moveTo(boost::array<double, 7> jointPos, double time);

	  private:
		void print_user_variables(tFriKrlData, bool from);
		void print_var_from_krl();
		void print_var_to_krl();

		void n_axes_process_event(RTT::base::PortInterface* port_interface);

		InputPort<tFriKrlData> port_from_krl;
		OutputPort<tFriKrlData> port_to_krl;

		InputPort<tFriRobotState> port_robot_state;
		InputPort<tFriIntfState> port_fri_state;

		OutputPort<lwr_fri::FriJointImpedance> port_desJntImpedance;
		OutputPort<lwr_fri::CartesianImpedance> port_desCartImpedance;

		InputPort<sensor_msgs::JointState> port_msrJointState;
		InputPort<geometry_msgs::Pose>  port_msrCartPos;
		InputPort<geometry_msgs::Wrench> port_msrExtCartWrench;

	    InputPort<string> port_nAxesEvent;

		tFriKrlData data_from_krl;
		tFriKrlData data_to_krl;

		tFriRobotState data_robot_state;
		tFriIntfState data_fri_state;

		string data_nAxes_event;

		bool call_set_ctrl_mode;

		bool nAxes_is_moving;
		bool cart_is_moving;

		TaskContext* n_axes_generator_pos_peer;

	    OperationCaller<bool(const vector<double>&, double)> nAxes_moveTo;
	    OperationCaller<void(void)> nAxes_resetPosition;
	};

}
#endif
