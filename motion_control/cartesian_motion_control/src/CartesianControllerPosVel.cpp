// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006 Ruben Smits <ruben.smits@mech.kuleuven.be>
// Copyright (C) 2011 Markus Klotzbuecher <markus.klotzbuecher@mech.kuleuven.be>
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

#include "CartesianControllerPosVel.hpp"
#include <rtt/Logger.hpp>

using namespace RTT;
using namespace KDL;
using namespace std;

namespace MotionControl
{
    CartesianControllerPosVel::CartesianControllerPosVel(string name)
        : TaskContext(name,PreOperational),
          _gain_local(6,0.0),
          _position_meas(),
          _position_desi(),
          _velocity_desi(),
          _velocity_out(),
          _controller_gain(6, 0.0)
    {
        this->addPort("CartesianSensorPosition", _position_meas);
        this->addPort("CartesianDesiredPosition", _position_desi);
        this->addPort("CartesianDesiredVelocity", _velocity_desi);
        this->addPort("CartesianOutputVelocity", _velocity_out);

        this->addProperty("K", _controller_gain).doc("Proportional Controller Gain");
    }

    CartesianControllerPosVel::~CartesianControllerPosVel() {};

    bool CartesianControllerPosVel::configureHook()
    {
        if(_controller_gain.size()!=6)
            return false;
        _gain_local=_controller_gain;
        return true;
    }

    bool CartesianControllerPosVel::startHook() { return true; }

    void CartesianControllerPosVel::updateHook()
    {
        FlowStatus fs;
        // copy Input and Setpoint to local values

        fs = _position_meas.read(_position_meas_local);
        if(fs==NoData)
            return;

        fs = _position_desi.read(_position_desi_local);
        if(fs==NoData)
            return;

        fs = _velocity_desi.read(_velocity_desi_local);
        if(fs==NoData)
            return;

        // feedback on position
        _velocity_feedback = diff(_position_meas_local, _position_desi_local);
        for(unsigned int i=0; i<6; i++)
            _velocity_feedback(i) *= _gain_local[i];

        // Add desired velocity as feedforward
        _velocity_out_local = _velocity_desi_local + _velocity_feedback;
        _velocity_out.write(_velocity_out_local);
    }

    void CartesianControllerPosVel::stopHook() {}
    void CartesianControllerPosVel::cleanupHook() {}

} //namespace
