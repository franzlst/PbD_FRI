// $Id: nAxisControllerPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006-2008 Ruben Smits <ruben.smits@mech.kuleuven.be>
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

#ifndef __N_AXES_CONTROLLER_POS_H__
#define __N_AXES_CONTROLLER_POS_H__

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>

#include <sensor_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

namespace MotionControl
{
    /**
     * This component can control the positions of multiple axes. It
     * uses a simple position-feedback to calculate an output
     * velocity.  velocity_out = K_gain * ( position_desired -
     * position_measured)
     *
     * The initial state of the component is PreOperational
     * \ingroup nAxesComponents
     */

    class nAxesControllerPos : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class
         *
         * @param name name of the component
         *
         */
        nAxesControllerPos(const std::string& name);

        virtual ~nAxesControllerPos();
        /**
         * Configures the component, make sure the properties are
         * updated using the OCL::DeploymentComponent or the marshalling
         * interface of the component.
         *
         * The number of axes and the control gains are updated.
         *
         * @return false if the gains property does not match the
         * number of axes, true otherwise
         */
        virtual bool configureHook();
        /**
         * Starts the component
         *
         * @return failes if the input-ports are not ready or the size
         * of the input-ports does not match the number of axes this
         * component is configured for.
         */
        virtual bool startHook();
        /**
         * Updates the output using the control equation, the measured
         * and the desired position
         */
        virtual void updateHook();
        virtual void stopHook();

    private:
        unsigned int        m_num_axes;

        sensor_msgs::JointState m_joint_state;
        motion_control_msgs::JointPositions m_p_desi;
        motion_control_msgs::JointVelocities m_v_out;
        std::vector<double> m_gain;

        /// The measured positions
        RTT::InputPort< sensor_msgs::JointState >    port_joint_state;
        /// The desired positions
        RTT::InputPort< motion_control_msgs::JointPositions >    port_p_desi;
        /// The output velocities
        RTT::OutputPort< motion_control_msgs::JointVelocities >   port_v_out;

        /// The control gain values for the axes.
        std::vector<double>  prop_gain;
        /// The number of axes to configure the components with.
        unsigned int prop_num_axes;
    }; // class
}//namespace
#endif // __N_AXES_CONTROLLER_POS_H__
