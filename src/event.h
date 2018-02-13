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
	FINISH_TEMPERATURE_QUERY = 0x04
}event_flag_t;

extern event_flag_t event_flag;

#endif /* EVENT_H_ */
