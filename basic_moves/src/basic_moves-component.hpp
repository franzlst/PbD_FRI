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
using namespace std;

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
    InputPort<tFriRobotState> robot_state_port;
    InputPort<tFriIntfState> fri_state_port;

    tFriRobotState current_robot_state;
    tFriIntfState current_fri_state;

    InputPort< sensor_msgs::JointState >  joints_pos_port;
    InputPort< geometry_msgs::Pose > pose_port;
    InputPort< string > axes_event_port;

    sensor_msgs::JointState current_joints_pos;
    geometry_msgs::Pose current_pose;

    size_t current_joints_pos_index;
    vector< vector<double> > joints_pos_storage;

    TaskContext* axes_generator_pos_peer;

    OperationCaller<bool(const vector<double>&, double)> move_to_caller;
    OperationCaller<bool(void)> axes_gen_running_caller;
    OperationCaller<void(void)> reset_position_caller;
};

}
#endif
