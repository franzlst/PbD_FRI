/*
 * ftsComm.h
 *
 *  Created on: Feb 11, 2014
 *      Author: Franz Steinmetz
 */

#ifndef FTSCOMM_H_
#define FTSCOMM_H_

namespace iros {

	#define FTS_HEADER			0x1234

	#define FTS_CMD_STOP		0x00
	#define FTS_CMD_START_RT	0x02
	#define FTS_CMD_START_BUF	0x03
	#define FTS_CMD_START_MUL	0x04
	#define FTS_CMD_RESET_LATCH	0x41
	#define FTS_CMD_SET_BIAS	0x42


	typedef struct {
		uint16_t command_header;	// Required, must be 0x1234
		uint16_t command;			// Command to execute, 0x00 = Stop, 0x02 = Start RT, 0x03 = Start Buffered, 0x04 = Start Multi, 0x41 = Reset Latch, 0x42 = Set Software Bios
		uint32_t sample_count;		// Samples to output (0 = infinite)
	} FTS_Command;

	typedef struct {
		int32_t x;
		int32_t y;
		int32_t z;
	} FTS_Vector;

	typedef struct {
		uint32_t rdt_sequence;		// RDT sequence number of this packet.
		uint32_t ft_sequence;		// The record’s internal sequence number
		uint32_t status;			// System status code
		FTS_Vector force;			// has to be divided by the Counts per Force
		FTS_Vector torque;			// has to be divided by the Counts per Torque
	} FTS_Respone;


}

#endif /* FTSCOMM_H_ */
