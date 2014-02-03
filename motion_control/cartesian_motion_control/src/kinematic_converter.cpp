// Copyright (C) 2007 Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright (C) 2011 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
//
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/ocl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "kinematic_converter.hpp"

namespace MotionControl
{
    using namespace std;
    using namespace RTT;
    using namespace KDL;

    KinematicConverter::KinematicConverter(string name) :
        TaskContext(name,PreOperational),
        chain_prop("Chain","Kinematic Description of the robot chain"),
        toolframe("ToolLocation","Offset between the robot's end effector and the tool location"),
        kinematics_status(true)
    {
        //Create the ports
	this->addPort("CartesianSensorPosition", cartpos_port);
        this->addPort("CartesianOutputVelocity", cartvel_port);
        this->addPort("nAxesSensorPosition", naxespos_port);
        this->addPort("nAxesOutputVelocity", naxesvel_port);

        cartpos_port.write(KDL::Frame::Identity());
        // initialize cartvel_port to KDL::Twist::Zero()

        this->properties()->addProperty(chain_prop);
        this->properties()->addProperty(toolframe);

    }

    KinematicConverter::~KinematicConverter()
    {
    }

    bool KinematicConverter::configureHook()
    {
        chain = chain_prop;
        chain.addSegment(Segment(Joint(Joint::None),toolframe.value()));

        nj = chain.getNrOfJoints();
        jointpositions=new JntArray(nj);
        jointvelocities=new JntArray(nj);
        naxesposition.resize(nj);
        naxesvelocities.resize(nj);

        fksolver=new ChainFkSolverPos_recursive(chain);
        iksolver=new ChainIkSolverVel_pinv(chain);
        return true;

    }

    void KinematicConverter::cleanupHook()
    {
        delete jointpositions;
        delete jointvelocities;
        delete fksolver;
        delete iksolver;
    }

    bool KinematicConverter::startHook()
    {
        //Initialize: calculate everything once
        this->updateHook();
        //Check if there were any problems calculating the kinematics
        return kinematics_status>=0;
    }

    void KinematicConverter::updateHook()
    {
        //Read out the ports
        naxespos_port.read(naxesposition);
        cartvel_port.read(cartvel);

        //Check if the jointpositions-port value as a correct size
        if(nj==naxesposition.size()){

            //copy into KDL-type
            for(unsigned int i=0;i<nj;i++)
                (*jointpositions)(i)=naxesposition[i];

            //Calculate forward position kinematics
            kinematics_status = fksolver->JntToCart(*jointpositions,cartpos);
            //Only set result to port if it was calcuted correctly
            if(kinematics_status>=0)
                cartpos_port.write(cartpos);
            else
                log(Error)<<"Could not calculate forward kinematics"<<endlog();

            //Calculate inverse velocity kinematics
            kinematics_status = iksolver->CartToJnt(*jointpositions,cartvel,*jointvelocities);
            if(kinematics_status<0){
                SetToZero(*jointvelocities);
                log(Error)<<"Could not calculate inverse kinematics"<<endlog();
            }

            for(unsigned int i=0;i<nj;i++)
                naxesvelocities[i]=(*jointvelocities)(i);

            naxesvel_port.write(naxesvelocities);
        }
        else
            kinematics_status=-1;
    }

    void KinematicConverter::stopHook()
    {
        for(unsigned int i=0;i<nj;i++)
            naxesvelocities[i]=0.0;
        naxesvel_port.write(naxesvelocities);
    }

}

ORO_CREATE_COMPONENT( MotionControl::KinematicConverter );
