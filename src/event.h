/*
 * event.h
 *
 *  Created on: Feb 12, 2018
 *      Author: David
 */

#ifndef EVENT_H_
#define EVENT_H_

typedef enum {
	NO_EVENT = 0x00,
	START_TEMPERATURE_POR = 0x01,
	START_TEMPERATURE_QUERY = 0x02,
	FINISH_TEMPERATURE_QUERY = 0x04,
	ACK_RECEIVED = 0x08,
	LOAD_MEASURE_CMD = 0x10,
	LOAD_STOP_CMD = 0x20
}event_flag_t;

extern event_flag_t event_flag;

#endif /* EVENT_H_ */
