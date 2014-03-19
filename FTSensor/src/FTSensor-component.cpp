#include "FTSensor-component.hpp"
#include <rtt/Component.hpp>
#include "ftsComm.h"

#include <iostream>
#include <sys/types.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <rtdm/rtdm.h>
#include <fcntl.h>


namespace iros {

using namespace RTT;
using namespace std;

	FTSensor::FTSensor(std::string const& name) : TaskContext(name), fts_ip("192.168.1.1"), fts_port(49152) {

		this->addProperty("fts_ip", fts_ip);
		this->addProperty("fts_port", fts_port);

		// Input port
		this->ports()->addPort("Publishing", publish).doc("Receives new 'true' value when something is published");

		// Output ports
		this->ports()->addPort("ForcePort", force_port).doc("Publish forces from FT sensor");
		this->ports()->addPort("TorquePort", torque_port).doc("Publish torques from FT sensor");


		Logger::In in((this->getName()));
		log(Debug) << "FTSensor constructed !" << endlog();
	}

	bool FTSensor::configureHook(){
		Logger::In in((this->getName()));

		geometry_msgs::Vector3 dummy;
		dummy.x = 0.0; dummy.y = 0.0; dummy.z = 0.0;
		force_port.setDataSample(dummy);
		torque_port.setDataSample(dummy);

		if(!open_connection())
			return false;

		log(Debug) << "FTSensor configured !" << endlog();
		return true;
	}

	bool FTSensor::startHook(){
		Logger::In in((this->getName()));
		log(Debug) << "FTSensor started !" << endlog();
		return true;
	}

	void FTSensor::updateHook(){
		Logger::In in((this->getName()));

		char raw_response[36];
		FTS_Respone response;

		int err;
		if((err = rt_dev_recv(fts_socket, (void*) &raw_response, sizeof(raw_response), 0 )) < sizeof(raw_response)) {
			if(err < 0) {
				log(Error) << "Error while receiving packet from F/T sensor. Error code: " << err << endlog();
			} else {
				log(Error) << "Not all bytes were received from F/T sensor. Bytes received: " << err << endlog();
			}
		}

		bool publish_now;
		if(publish.read(publish_now) != NewData)
			return;

		if(! publish_now)
			return;

		response.rdt_sequence = ntohl(*(uint32_t*)&raw_response[0]);
		response.ft_sequence = ntohl(*(uint32_t*)&raw_response[4]);
		response.status = ntohl(*(uint32_t*)&raw_response[8]);
		response.force.x = ntohl(*(int32_t*)&raw_response[12]);
		response.force.y = ntohl(*(int32_t*)&raw_response[16]);
		response.force.z = ntohl(*(int32_t*)&raw_response[20]);
		response.torque.x = ntohl(*(int32_t*)&raw_response[24]);
		response.torque.y = ntohl(*(int32_t*)&raw_response[28]);
		response.torque.z = ntohl(*(int32_t*)&raw_response[32]);


		static int ctr = 0;
		if(ctr % 1000 == 0) {
			log(Debug) << "FT Sensor Status:   " << response.status << endlog();
			log(Debug) << "FT Sensor Sequence: " << response.rdt_sequence << endlog();
			log(Debug) << "FT Sensor Force:  x = " << response.force.x << " y = " << response.force.y << " z = " << response.force.z << endlog();
			log(Debug) << "FT Sensor Torque: x = " << response.torque.x << " y = " << response.torque.y << " z = " << response.torque.z << endlog();
		}
		ctr++;

		current_force.x = response.force.x;
		current_force.y = response.force.y;
		current_force.z = response.force.z;
		current_torque.x = response.torque.x;
		current_torque.y = response.torque.y;
		current_torque.z = response.torque.z;

		force_port.write(current_force);
		torque_port.write(current_torque);

		//log(Debug) << "FTSensor executes updateHook !" << endlog();
	}

	void FTSensor::stopHook() {
		Logger::In in((this->getName()));
		log(Debug) << "FTSensor executes stopping !" << endlog();
	}

	void FTSensor::cleanupHook() {
		Logger::In in((this->getName()));

		close_connection();

		log(Debug) << "FTSensor cleaned up !" << endlog();
	}

	bool FTSensor::open_connection() {

		struct sockaddr_in fts_addr;	/* Address of Net F/T. */

		if ((fts_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
			log(Error) << "The socket to the F/T sensor couldn't be created. Error code: " << fts_socket << endlog();
			return false;
		}
		//setsockopt(fts_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0); // This is not supported by RTnet!

		fts_addr.sin_family = AF_INET;
		fts_addr.sin_port = htons(fts_port);
		inet_aton(fts_ip.c_str(), &fts_addr.sin_addr); // convert IP string to in_addr

		int err;
		if ((err = rt_dev_connect(fts_socket, (struct sockaddr *)&fts_addr, sizeof(fts_addr))) != 0) {
			log(Error) << "The connection to the F/T sensor couldn't be established. Error code: " << err << endlog();
			return false;
		}

		FTS_Command fts_request;
		fts_request.command_header = htons(FTS_HEADER);
		fts_request.command = htons(FTS_CMD_START_RT);
		fts_request.sample_count = htonl(0); // Infinity

		if((err = rt_dev_send(fts_socket, (void*) &fts_request, sizeof(fts_request), 0)) < sizeof(fts_request)) {
			if(err < 0) {
				log(Error) << "Error while sending start request to F/T sensor. Error code: " << err << endlog();
			} else {
				log(Error) << "Not all bytes were transmitted while sending start request to F/T sensor. Bytes transmitted: " << err << endlog();
			}
			return false;
		}

		return true;
	}

	void FTSensor::close_connection() {
		FTS_Command fts_request;
		fts_request.command_header = htons(FTS_HEADER);
		fts_request.command = htons(FTS_CMD_STOP);
		fts_request.sample_count = htonl(0);

		if(rt_dev_send(fts_socket, (void*) &fts_request, sizeof(fts_request), 0) < sizeof(fts_request)) {
			log(Error) << "Error while sensing stop request to F/T sensor."<< endlog();
		}

		if(rt_dev_close(fts_socket) != 0) {
			log(Error) << "Error while closing socket to F/T sensor." << endlog();
		}
	}
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(FTSensor)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(iros::FTSensor)
