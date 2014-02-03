// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2010 Ruben Smits <ruben.smits@mech.kuleuven.be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __CARTESIAN_GENERATOR_POS_H__
#define __CARTESIAN_GENERATOR_POS_H__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#include <string>

namespace MotionControl
{
    /**
     * This class implements a TaskContext that creates a path in
     * Cartesian space between the current cartesian position and a
     * new desired cartesian position. It uses trapezoidal
     * velocity-profiles for every dof using a maximum velocity and a
     * maximum acceleration. It generates frame and twist setpoints
     * which can be used by MotionControl::CartesianControllerPos,
     * MotionControl::CartesianControllerPosVel or MotionControl::CartesianControllerVel.
     *
     */
    class CartesianGeneratorPos : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
        CartesianGeneratorPos(std::string name);
        virtual ~CartesianGeneratorPos();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

    private:
        bool moveTo(geometry_msgs::Pose pose, double time=0);
        void resetPosition();

      KDL::Frame                        m_traject_end, m_traject_begin;
      KDL::Frame                        m_position_desi_local;
      KDL::Twist                        m_velocity_desi_local, m_velocity_begin_end, m_velocity_delta;
      KDL::Twist                        m_maximum_velocity, m_maximum_acceleration;
      geometry_msgs::Twist              m_gm_maximum_velocity, m_gm_maximum_acceleration;

      std::vector<KDL::VelocityProfile_Trap>      m_motion_profile;
      RTT::os::TimeService::ticks                     m_time_begin;
      RTT::os::TimeService::Seconds                   m_time_passed;
      double                                      m_max_duration;

      bool                                        m_is_moving,m_once;
      std::string								  move_started_event; 
      std::string								  move_finished_event;  

    protected:
      /// Dataport containing the current measured end-effector
      /// frame, shared with MotionControl::CartesianSensor
      RTT::InputPort< geometry_msgs::Pose >   m_position_meas_port;

      /// Dataport containing the current desired end-effector
      /// frame, shared with MotionControl::CartesianControllerPos,
      /// MotionControl::CartesianControllerPosVel
      RTT::OutputPort< geometry_msgs::Pose >  m_position_desi_port;
      /// Dataport containing the current desired end-effector
      /// twist, shared with MotionControl::CartesianControllerPosVel,
      /// MotionControl::CartesianControllerVel
      RTT::OutputPort< geometry_msgs::Twist >  m_velocity_desi_port;

      RTT::OutputPort<std::string> event_port;

  }; // class
} //namespace

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( MotionControl::CartesianGeneratorPos )

#endif // __CARTESIAN_GENERATOR_POS_H__

