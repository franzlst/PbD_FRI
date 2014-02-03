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

#include "nAxesGeneratorPos.hpp"
#include <rtt/Component.hpp>
#include <rtt/os/MutexLock.hpp>

namespace MotionControl
{
    using namespace RTT;
    using namespace KDL;
    using namespace std;
    typedef nAxesGeneratorPos MyType;

    nAxesGeneratorPos::nAxesGeneratorPos(const string& name)
      : TaskContext(name,PreOperational),
	move_started_event("e_"+name+"_move_started"), 
	move_finished_event("e_"+name+"_move_finished"), traj_finished_event("e_"+name+"_traj_finished"), 
	
	is_moving(false), isTrajMoving(false), trajIndex(0)

{
        //Creating TaskContext

        //Adding properties
        this->addProperty("num_axes",num_axes_prop).doc("Number of Axes");
        this->addProperty("max_vel", v_max_prop).doc("Maximum Velocity in Trajectory");
        this->addProperty("max_acc", a_max_prop).doc("Maximum Acceleration in Trajectory");
        this->addProperty("movingTimeOnPort", movingTimeOnPort).doc("Moving time for desired positions on port. ");
        this->addProperty("delayTimes", delay_times_prop).doc("Delay times for each joint. ");
        //Adding ports
        this->addPort("nAxesSensorPosition"  , p_m_port );
        this->addPort("nAxesDesiredPosition" , p_d_port );
        this->addPort("nAxesDesiredVelocity" , v_d_port );
        this->addPort("events", event_port);
        this->addEventPort("nAxesJointPosition"  , joint_endpose_port, boost::bind(&nAxesGeneratorPos::moveToOnPort, this));
        this->addEventPort("nAxesJointPositionDelayed"  , joint_endpose_delayed_port, boost::bind(&nAxesGeneratorPos::moveToDelayedOnPort, this));

        //Adding Operations
        this->addOperation( "moveTo",&MyType::moveTo,this)
				.doc("Set the position setpoint")
				.arg("setpoint", "joint setpoint for all axes")
				.arg("time", "minimum time to complete trajectory");
        this->addOperation( "resetPosition", &MyType::resetPosition, this )
				.doc("Reset generator position");
        this->addOperation( "pause", &MyType::pause, this ).doc("Pause motion");
        this->addOperation( "moveToDelayed", &MyType::moveToDelayed, this)
				.doc("Set the position setpoint")
				.arg("setpoint", "joint setpoint for all axes")
				.arg("time", "minimum time to complete trajectory")
				.arg("delay_times","extra delay times for the axes");
        this->addOperation( "moveTraject", &MyType::newTraject, this)
        		.doc("Set trajectory to be executed as fast as possible, setpoint times will be ignored.");

    }

    nAxesGeneratorPos::~nAxesGeneratorPos()
    {}

    bool nAxesGeneratorPos::configureHook()
    {
        Logger::In in(this->getName());
        num_axes=num_axes_prop;
        if(v_max_prop.size()!=num_axes){
            log(Error)<<"Size of max_vel does not match num_axes"<<endlog();
            return false;
        }

        if(a_max_prop.size()!=num_axes){
            log(Error)<<"Size of max_acc does not match num_axes" <<endlog();
            return false;
        }

        if (delay_times_prop.size() != num_axes){
            log(Error)<<"Size of delay_times does not match num_axes" <<endlog();
            return false;
        }

        v_max=v_max_prop;
        a_max=a_max_prop;

        //Resizing all containers to correct size
        p_d.positions.resize(num_axes);
        v_d.velocities.resize(num_axes);
        motion_profile.resize(num_axes);

        //Initialise motion profiles
        for(unsigned int i=0;i<num_axes;i++)
            motion_profile[i].SetMax(v_max[i],a_max[i]);

        //Initialise output ports:
        p_d.positions.assign(num_axes,0);
        p_d_port.setDataSample( p_d );
        v_d.velocities.assign(num_axes,0);
        v_d_port.setDataSample( v_d );
        event_port.setDataSample(move_finished_event);
        return true;
    }


    bool nAxesGeneratorPos::startHook()
    {
        Logger::In in(this->getName());
        //check connection and sizes of input-ports
        if(!p_m_port.connected()){
            log(Error)<<p_m_port.getName()<<" not ready"<<endlog();
            return false;
        }
        if(p_m_port.read(joint_state)==NoData){
            log(Error)<<"No data available on "<<p_m_port.getName()<<", I refuse to start"<<endlog();
            return false;
        }
        if(joint_state.position.size()!=num_axes){
            log(Error)<<"Size of "<<p_m_port.getName()<<": "<<joint_state.position.size()<<" != " << num_axes<<endlog();
            return false;
        }

        p_d.positions = joint_state.position;
        p_d_port.write( p_d );
        is_moving = false;

        return true;
    }

    void nAxesGeneratorPos::updateHook()
    {
    	//log(Info)<<"nAxesGeneratorPos: tack"<<endlog();
        if (is_moving){
//        	log(Info)<<"nAxesGeneratorPos: tick"<<endlog();
            time_passed = os::TimeService::Instance()->secondsSince(time_begin);
            if ( time_passed > max_duration ){// Profile is ended
                // set end position
                for (unsigned int i=0; i<num_axes; i++){
                    p_d.positions[i] = motion_profile[i].Pos( max_duration );
                    v_d.velocities[i] = motion_profile[i].Vel( max_duration );
                }
                is_moving = false;
				// send move_finished_event (once)
                event_port.write(move_finished_event);
                if (isTrajMoving == true) {
                	this->moveTraject();
                }
            }else{
                for(unsigned int i=0; i<num_axes; i++){
                    p_d.positions[i] = motion_profile[i].Pos( time_passed );
                    v_d.velocities[i] = motion_profile[i].Vel( time_passed );
                }
            }
            p_d_port.write( p_d );
            v_d_port.write( v_d );
        }
    }

    void nAxesGeneratorPos::stopHook()
    {
    }

    void nAxesGeneratorPos::moveToOnPort() {
    	if (joint_endpose_port.read(joint_endpose) == NoData) {
    		log(Error) << "No desired end-position on port. "<< endlog();
    		return;
    	}
		this->moveTo(joint_endpose.positions, movingTimeOnPort);
    	return;
    }

    void nAxesGeneratorPos::moveToDelayedOnPort() {
    	if (joint_endpose_delayed_port.read(joint_endpose) == NoData) {
    		log(Error) << "No desired end-position on port. "<< endlog();
    		return;
    	}
		this->moveToDelayed(joint_endpose.positions, movingTimeOnPort, delay_times_prop);
    	return;
    }

    bool nAxesGeneratorPos::newTraject(trajectory_msgs::JointTrajectory& new_traject)
    {
    	if (isTrajMoving == false) {
    		traject = new_traject;
    		isTrajMoving = true;
    		trajIndex = 0;
    		this->moveTo(traject.points[trajIndex].positions, 0.1);
    		return true;
    	}
    	else {
    		log(Error) << "Previous trajectory is still being executed, new trajectory ignored. "<< endlog();
    		return false;
    	}

    }

    bool nAxesGeneratorPos::moveTraject()
    {
    	trajIndex = trajIndex + 1;
    	if (trajIndex < traject.points.size()) {
    		this->moveTo(traject.points[trajIndex].positions, 0.1);
    	}
    	else {
    		event_port.write(traj_finished_event);
    		isTrajMoving = false;
    	}
		return true;
    }

    bool nAxesGeneratorPos::moveTo(const vector<double>& position, double time)
    {
        Logger::In in((this->getName()));
        if(position.size()!=num_axes){
            log(Error)<<"Size of position != "<<num_axes<<endlog();
            return false;
        }
        if (!isRunning()) {
        	log(Error)<<"Can't move when not running."<<endlog();
        	return false;
        }

        RTT::os::MutexLock lock(moving_mut);
        // if previous movement is finished
        if (!is_moving){
            max_duration = 0;
            // get current position/
            p_m_port.read( joint_state );
            for (unsigned int i=0; i<num_axes; i++){
                // Set motion profiles
                motion_profile[i].SetProfileDuration( joint_state.position[i], position[i], time );
                // Find lengthiest trajectory
                max_duration = max( max_duration, motion_profile[i].Duration() );
            }
            // Rescale trajectories to maximal duration
            log(Info)<<"Moving to [";
            for(unsigned int i = 0; i < num_axes; i++){
                motion_profile[i].SetProfileDuration( joint_state.position[i], position[i], max_duration );
                log(Info)<<position[i]<<" ";
            }
            log(Info)<<"] in "<<max_duration<<" seconds."<<endlog();


            time_begin = os::TimeService::Instance()->getTicks();
            time_passed = 0;

            is_moving = true;
			// send move_started_event 
			event_port.write(move_started_event);

            return true;
        }
        // still moving
        else{
            log(Warning)<<"Still moving, not executing new command."<<endlog();
            return false;
        }

    }

    bool nAxesGeneratorPos::moveToDelayed(const vector<double>& position, double time, const vector<double>& delay_times )
    {
        Logger::In in((this->getName()));
        if( (position.size()!=num_axes) && (delay_times.size()!=num_axes) ){
            log(Error)<<"Size of position or delay_times != "<<num_axes<<endlog();
            return false;
        }

        // if previous movement is finished
	if (!is_moving){
            max_duration = 0;
            // get current position/
            p_m_port.read( joint_state );
            for (unsigned int i=0; i<num_axes; i++){
                // Set motion profiles
                motion_profile[i].SetProfileDuration( joint_state.position[i], position[i], time );
                // Find lengthiest trajectory
                max_duration = max( max_duration, motion_profile[i].Duration() );
            }
	    
            // Rescale trajectories to maximal duration
	    double max_delay=0.0;
		for(unsigned int i = 0; i < num_axes; i++){
			motion_profile[i].SetProfileDuration( joint_state.position[i], position[i], max_duration+delay_times[i] );
			max_delay = max(max_delay, delay_times[i]);
	    }
	    max_duration += max_delay;

		time_begin = os::TimeService::Instance()->getTicks();
		time_passed = 0;

		is_moving = true;
		return true;
        }
        // still moving
        else{
            log(Warning)<<"Still moving, not executing new command."<<endlog();
            return false;
        }

    }


    void nAxesGeneratorPos::resetPosition()
    {
        p_m_port.read( joint_state );
        for(unsigned int i = 0; i < num_axes; i++)
            v_d.velocities[i] = 0;
        p_d.positions=joint_state.position;
        p_d_port.write( p_d );
        v_d_port.write( v_d );
        is_moving = false;
        isTrajMoving = false;
    }

    void nAxesGeneratorPos::pause()
    {
        for(unsigned int i = 0; i < num_axes; i++)
            v_d.velocities[i] = 0;
        v_d_port.write( v_d );
        is_moving = false;
        isTrajMoving = false;
    }
}//namespace

ORO_CREATE_COMPONENT( MotionControl::nAxesGeneratorPos )
