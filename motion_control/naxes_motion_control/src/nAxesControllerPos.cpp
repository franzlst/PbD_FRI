// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006 Ruben Smits <ruben.smits@mech.kuleuven.ac.be>
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


#include "nAxesControllerPos.hpp"
#include <rtt/Component.hpp>

namespace MotionControl
{

    using namespace RTT;
    using namespace std;
    typedef nAxesControllerPos MyType;


    nAxesControllerPos::nAxesControllerPos(const string& name)
        : TaskContext(name,PreOperational)
    {
        //Creating TaskContext

        //Adding Ports
        this->addPort("JointState"  , port_joint_state);
        this->addPort("nAxesDesiredPosition" , port_p_desi);
        this->addPort("nAxesOutputVelocity"  , port_v_out);

        //Adding Properties
        this->addProperty("num_axes",prop_num_axes).doc("Number of Axes");
        this->addProperty("K",prop_gain).doc("Proportional Gain");
    }

    nAxesControllerPos::~nAxesControllerPos(){};

    bool nAxesControllerPos::configureHook()
    {
        //Check and read all properties
        m_num_axes=prop_num_axes;
        if(prop_gain.size()!=m_num_axes){
            Logger::In in(this->getName().data());
            log(Error)<<"Size of K does not match num_axes" <<endlog();
            return false;
        }

        m_gain=prop_gain;

        //Resizing all containers to correct size
        m_joint_state.name.resize(m_num_axes);
        m_joint_state.position.resize(m_num_axes);
        m_joint_state.velocity.resize(m_num_axes);
        m_joint_state.effort.resize(m_num_axes);

        m_p_desi.positions.resize(m_num_axes);

        //Initialise output ports:
        m_v_out.velocities.assign(m_num_axes,0);
        port_v_out.write( m_v_out );

        return true;
    }


    bool nAxesControllerPos::startHook()
    {
        Logger::In in(this->getName());
        //check connection and sizes of input-ports
        if(!port_joint_state.connected()){
            log(Error)<<port_joint_state.getName()<<" not ready"<<endlog();
            return false;
        }
        if(!port_p_desi.connected()){
            log(Error)<<port_p_desi.getName()<<" not ready"<<endlog();
            return false;
        }
        if(port_joint_state.read(m_joint_state)==NoData){
            log(Error)<<"No data availabe on "<< port_joint_state.getName()<<", I refuse to start!"<<endlog();
            return false;
        }
        if(m_joint_state.position.size()!=m_num_axes){
            log(Error)<<"Size of "<<port_joint_state.getName()<<": "<<m_joint_state.position.size()<<" != " << m_num_axes<<endlog();
            return false;
        }
        if(port_p_desi.read(m_p_desi)==NoData){
            log(Error)<<"No data availabe on "<< port_p_desi.getName()<<", I refuse to start!"<<endlog();
            return false;
        }
        if(m_p_desi.positions.size()!=m_num_axes){
            log(Error)<<"Size of "<<port_p_desi.getName()<<": "<<m_p_desi.positions.size()<<" != " << m_num_axes<<endlog();
            return false;
        }

        return true;

    }


    void nAxesControllerPos::updateHook()
    {
        // copy Input and Setpoint to local values
        port_joint_state.read( m_joint_state );
        port_p_desi.read( m_p_desi );

        // position feedback
        for(unsigned int i=0; i<m_num_axes; i++)
            m_v_out.velocities[i] = m_gain[i] * (m_p_desi.positions[i] - m_joint_state.position[i]);

        port_v_out.write( m_v_out );
    }



    void nAxesControllerPos::stopHook()
    {
        for(unsigned int i=0; i<m_num_axes; i++){
            m_v_out.velocities[i] = 0.0;
        }
        port_v_out.write( m_v_out );
    }

}//namespace

ORO_CREATE_COMPONENT( MotionControl::nAxesControllerPos )
