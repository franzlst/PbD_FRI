#include <motion_control_msgs/JointAccelerations.h>
#include <motion_control_msgs/JointEfforts.h>
#include <motion_control_msgs/JointVelocities.h>
#include <motion_control_msgs/JointPositions.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_motion_control_msgs_JointAccelerations();
        void rtt_ros_addType_motion_control_msgs_JointEfforts();
        void rtt_ros_addType_motion_control_msgs_JointVelocities();
        void rtt_ros_addType_motion_control_msgs_JointPositions();

   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROSmotion_control_msgsTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("ros-")+"motion_control_msgs";
      }

      virtual bool loadTypes() {
          // call all factory functions
                  rtt_ros_addType_motion_control_msgs_JointAccelerations(); // factory function for adding TypeInfo.
        rtt_ros_addType_motion_control_msgs_JointEfforts(); // factory function for adding TypeInfo.
        rtt_ros_addType_motion_control_msgs_JointVelocities(); // factory function for adding TypeInfo.
        rtt_ros_addType_motion_control_msgs_JointPositions(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSmotion_control_msgsTypekitPlugin )

