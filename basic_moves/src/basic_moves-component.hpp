#ifndef OROCOS_BASIC_MOVES_COMPONENT_HPP
#define OROCOS_BASIC_MOVES_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <kuka_lwr_fri/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

namespace iros {

using namespace RTT;

class Basic_moves : public RTT::TaskContext{
  public:
    Basic_moves(std::string const& name);
    virtual ~Basic_moves();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    InputPort< sensor_msgs::JointState >  joints_pos_port;
    InputPort< geometry_msgs::Pose > pose_port;

    sensor_msgs::JointState current_joints_pos;
    geometry_msgs::Pose current_pose;
};

}
#endif
