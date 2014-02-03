
#include <motion_control_msgs/JointAccelerations.h>
#include <motion_control_msgs/JointEfforts.h>
#include <motion_control_msgs/JointVelocities.h>
#include <motion_control_msgs/JointPositions.h>

#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSmotion_control_msgsPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
                   if(name == "/motion_control_msgs/JointAccelerations")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<motion_control_msgs::JointAccelerations>());
         if(name == "/motion_control_msgs/JointEfforts")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<motion_control_msgs::JointEfforts>());
         if(name == "/motion_control_msgs/JointVelocities")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<motion_control_msgs::JointVelocities>());
         if(name == "/motion_control_msgs/JointPositions")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<motion_control_msgs::JointPositions>());

          return false;
      }
      
      std::string getTransportName() const {
          return "ros";
      }
      
      std::string getTypekitName() const {
          return std::string("ros-")+"motion_control_msgs";
      }
      std::string getName() const {
          return std::string("rtt-ros-") + "motion_control_msgs" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSmotion_control_msgsPlugin )
