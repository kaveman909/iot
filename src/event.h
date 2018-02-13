/*
 * event.h
 *
 *  Created on: Feb 12, 2018
 *      Author: David
 */

#ifndef EVENT_H_
#define EVENT_H_

typedef enum {
	NO_EVENT,
	START_TEMPERATURE_QUERY
}event_flag_t;

extern event_flag_t event_flag;

#endif /* EVENT_H_ */
