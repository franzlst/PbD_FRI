#include <motion_control_msgs/boost/JointEfforts.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::DataSource< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::AssignCommand< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::ValueDataSource< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::OutputPort< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::InputPort< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::Property< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::Attribute< motion_control_msgs::JointEfforts >;
template class RTT_EXPORT RTT::Constant< motion_control_msgs::JointEfforts >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_motion_control_msgs_JointEfforts() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<motion_control_msgs::JointEfforts>("/motion_control_msgs/JointEfforts") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<motion_control_msgs::JointEfforts> >("/motion_control_msgs/JointEfforts[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<motion_control_msgs::JointEfforts> >("/motion_control_msgs/cJointEfforts[]") );
        }

    
}

