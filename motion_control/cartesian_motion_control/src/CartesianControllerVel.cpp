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

#include "CartesianControllerVel.hpp"

using namespace RTT;
using namespace KDL;
using namespace std;

namespace MotionControl
{
    CartesianControllerVel::CartesianControllerVel(string name)
        : TaskContext(name,PreOperational),
          _gain_local(6,0.0),
          _is_initialized(false),
          _position_meas("CartesianSensorPosition"),
          _velocity_desi("CartesianDesiredVelocity"),
          _velocity_out("CartesianOutputVelocity"),
          _controller_gain(6,0.0)
    {
	    this->ports()->addPort("CartesianSensorPosition", _position_meas);
        this->ports()->addPort("CartesianDesiredVelocity", _velocity_desi);
        this->ports()->addPort("CartesianOutputVelocity", _velocity_out);

        this->addProperty("K", _controller_gain).doc("Proportional controller gain");
    }

    CartesianControllerVel::~CartesianControllerVel() {};

    bool CartesianControllerVel::configureHook()
    {
        if(_controller_gain.size()!=6)
            return false;
        _gain_local=_controller_gain;
        return true;
    }

    bool CartesianControllerVel::startHook() { return true; }

    void CartesianControllerVel::updateHook()
    {
        FlowStatus fs;
        // copy Input and Setpoint to local values
        fs = _position_meas.read(_position_meas_local);
        if (fs == NoData)
            return;

        fs = _velocity_desi.read(_velocity_desi_local);
        if (fs == NoData)
            return;

        // initialize
        if (!_is_initialized) {
            _is_initialized = true;
            _position_integrated = _position_meas_local;
            _time_begin = os::TimeService::Instance()->getTicks();
        }

        // integrate velocity
        double time_difference = os::TimeService::Instance()->secondsSince(_time_begin);
        _time_begin = os::TimeService::Instance()->getTicks();
        _position_integrated = addDelta(_position_integrated, _velocity_desi_local, time_difference);

        // position feedback on integrated velocity
        _velocity_feedback = diff(_position_meas_local, _position_integrated);
        for(unsigned int i=0; i<6; i++)
            _velocity_feedback(i) *= _gain_local[i];

        // feedback + feedforward
        _velocity_out_local = _velocity_desi_local + _velocity_feedback;
        _velocity_out.write(_velocity_out_local);
    }

    void CartesianControllerVel::stopHook() {}
    void CartesianControllerVel::cleanupHook() {}

} //namespace



