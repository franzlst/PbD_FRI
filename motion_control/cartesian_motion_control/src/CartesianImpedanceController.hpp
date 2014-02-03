/**************************************************************************
 *         (C) 2011 Ruben Smits <ruben.smits@mech.kuleuven.be>            *
 *  (C) 2012 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>   *
 *               Department of Mechanical Engineering,                    *
 *              Katholieke Universiteit Leuven, Belgium.                  *
 *                                                                        *
 *  You may redistribute this software and/or modify it under either the  *
 *  terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
 *  <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
 *  discretion) of the Modified BSD License:                              *
 *  Redistribution and use in source and binary forms, with or without    *
 *  modification, are permitted provided that the following conditions    *
 *  are met:                                                              *
 *  1. Redistributions of source code must retain the above copyright     *
 *  notice, this list of conditions and the following disclaimer.         *
 *  2. Redistributions in binary form must reproduce the above copyright  *
 *  notice, this list of conditions and the following disclaimer in the   *
 *  documentation and/or other materials provided with the distribution.  *
 *  3. The name of the author may not be used to endorse or promote       *
 *  products derived from this software without specific prior written    *
 *  permission.                                                           *
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
 *  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
 *  POSSIBILITY OF SUCH DAMAGE.                                           *
 *                                                                        *
 **************************************************************************/

#ifndef __CARTESIANIMPEDANCECONTROLLER_HPP__
#define __CARTESIANIMPEDANCECONTROLLER_HPP__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>

using namespace RTT;

namespace MotionControl{


    class CartesianImpedanceController : public RTT::TaskContext
    {
    public:
        CartesianImpedanceController(const std::string& name): 
            RTT::TaskContext(name),
            force_thres_exceeded(false)
        {
            this->addEventPort("CartesianSensorPosition",port_pose_meas);
            this->addEventPort("CartesianDesiredPosition",port_pose_desi);
            this->addEventPort("CartesianSensorTwist",port_twist_meas);
            this->addEventPort("CartesianDesiredTwist",port_twist_desi);

            this->addPort("CartesianOutputWrench",port_wrench_out);
            this->addPort("ext_ref_mode", ext_ref_mode)
		    .doc("If 0, use mesured as desired pos, otherwise desired pos");

            this->addPort("force_thres_ex", force_thres_ex).doc("force threshold exceeded event");

            this->addProperty("stiffness",m_stiffness).doc("Cartesian Stiffness in tool frame");
            this->addProperty("damping",m_damping).doc("Cartesian Damping in tool frame");

            force_thres.resize(2);
            this->addProperty("force_thres",force_thres).
                doc("Force threshold for raising force_thres_exceeded event [trans,rot]");
        };

    private:
        bool startHook(){
			// Starting fails when no data on "CartesianSensorPosition" port
            if(port_pose_meas.read(m_pose_meas) == RTT::NoData )
                return false;
            port_twist_meas.read(m_twist_meas);
			// initialize m_pose/twist_desit with 'safe' values
			m_pose_desi = m_pose_meas;
			m_twist_desi = m_twist_meas;
			
            port_pose_desi.clear();
            port_twist_desi.clear();
            return true;
        };

        void updateHook(){
            double norm_force, norm_torque;
            KDL::Frame pm, pd;
            KDL::Twist tm, td, delta_x, delta_x_dot;
            RTT::FlowStatus fs;

            bool cur_ext_ref_mode=false;

            fs=ext_ref_mode.read(cur_ext_ref_mode);

            port_pose_meas.read(m_pose_meas);
            port_pose_desi.read(m_pose_desi);
            port_twist_meas.read(m_twist_meas);
            port_twist_desi.read(m_twist_desi);

            tf::PoseMsgToKDL(m_pose_meas,pm);
            tf::PoseMsgToKDL(m_pose_desi,pd);
            tf::TwistMsgToKDL(m_twist_meas,tm);
            tf::TwistMsgToKDL(m_twist_desi,td);

            delta_x = diff(pm, pd);
            delta_x_dot = diff(tm, td);

            m_wrench_out.force.x = m_stiffness.linear.x*delta_x.vel.x() + m_damping.linear.x*delta_x_dot.vel.x();
            m_wrench_out.force.y = m_stiffness.linear.y*delta_x.vel.y() + m_damping.linear.y*delta_x_dot.vel.y();
            m_wrench_out.force.z = m_stiffness.linear.z*delta_x.vel.z() + m_damping.linear.z*delta_x_dot.vel.z();
            m_wrench_out.torque.x = m_stiffness.angular.x*delta_x.rot.x() + m_damping.angular.x*delta_x_dot.rot.x();
            m_wrench_out.torque.y = m_stiffness.angular.y*delta_x.rot.y() + m_damping.angular.y*delta_x_dot.rot.y();
            m_wrench_out.torque.z = m_stiffness.angular.z*delta_x.rot.z() + m_damping.angular.z*delta_x_dot.rot.z();

            // check threshold
            norm_force = sqrt(pow(m_wrench_out.force.x, 2) +
                              pow(m_wrench_out.force.y, 2) +
                              pow(m_wrench_out.force.z, 2));

            norm_torque = sqrt(pow(m_wrench_out.torque.x, 2) +
                               pow(m_wrench_out.torque.y, 2) +
                               pow(m_wrench_out.torque.z, 2));

            // exceeding...
            if(!force_thres_exceeded && (norm_force>force_thres[0] || norm_torque>force_thres[1])) {
                force_thres_ex.write(true);
                force_thres_exceeded=true;
                log(Warning) << "exceeded for threshold, norm_force: " << norm_force
                             << " , norm_torque: " << norm_torque << endlog();
            }

            if(force_thres_exceeded && (norm_force<force_thres[0] && norm_torque<force_thres[1])) {
                force_thres_ex.write(false);
                force_thres_exceeded=false;
                log(Warning) << "fallen below threshold, norm_force: " << norm_force
                             << " , norm_torque: " << norm_torque << endlog();
            }

            if (cur_ext_ref_mode == false) {
                m_wrench_out.force.x=0; m_wrench_out.force.y=0; m_wrench_out.force.z=0;
                m_wrench_out.torque.x=0; m_wrench_out.torque.y=0; m_wrench_out.torque.z=0;
            }

            port_wrench_out.write(m_wrench_out);
        };

        void stopHook(){
            m_wrench_out.force.x=0;
            m_wrench_out.force.y=0;
            m_wrench_out.force.z=0;
            m_wrench_out.torque.x=0;
            m_wrench_out.torque.y=0;
            m_wrench_out.torque.z=0;
            port_wrench_out.write(m_wrench_out);
        };

        geometry_msgs::Pose m_pose_meas, m_pose_desi;
        geometry_msgs::Twist m_twist_meas, m_twist_desi;
        geometry_msgs::Wrench m_wrench_out;
        geometry_msgs::Twist m_stiffness, m_damping;
        
        RTT::InputPort<geometry_msgs::Pose> port_pose_meas, port_pose_desi;
        RTT::InputPort<geometry_msgs::Twist> port_twist_meas, port_twist_desi;
        RTT::OutputPort<geometry_msgs::Wrench> port_wrench_out;

        RTT::InputPort<bool> ext_ref_mode;
        RTT::OutputPort<bool> force_thres_ex;
        std::vector<double> force_thres;
	    bool force_thres_exceeded;
    };
}//namespace
        
#endif
