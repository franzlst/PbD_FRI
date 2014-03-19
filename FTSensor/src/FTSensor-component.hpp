#ifndef OROCOS_FTSENSOR_COMPONENT_HPP
#define OROCOS_FTSENSOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/typekit/Types.hpp>

namespace iros {

using namespace RTT;
using namespace std;

	class FTSensor : public RTT::TaskContext{
	  public:
		FTSensor(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	  private:
		bool open_connection();
		void close_connection();

		string fts_ip;
		int fts_port;
		int fts_socket;

		InputPort<bool> publish;

		OutputPort<geometry_msgs::Vector3> force_port;
		OutputPort<geometry_msgs::Vector3> torque_port;

		geometry_msgs::Vector3 current_force;
		geometry_msgs::Vector3 current_torque;

	};
}

#endif
