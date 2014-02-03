#include <motion_control_msgs/boost/JointPositions.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSJointPositionsPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/motion_control_msgs/JointPositions")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<motion_control_msgs::JointPositions>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"JointPositions";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "JointPositions" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSJointPositionsPlugin )
