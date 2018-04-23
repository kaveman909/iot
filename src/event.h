/*
 * event.h
 *
 *  Created on: Feb 12, 2018
 *      Author: David
 */

#ifndef EVENT_H_
#define EVENT_H_
#include <stdint.h>
typedef enum {
	NO_EVENT = 0x00,
	START_TEMPERATURE_POR = 0x01,
	START_TEMPERATURE_QUERY = 0x02,
	FINISH_TEMPERATURE_QUERY = 0x04,
	ACK_RECEIVED = 0x08,
	LOAD_MEASURE_CMD = 0x10,
	LOAD_STOP_CMD = 0x20,
	WAIT_FOR_MEASUREMENT = 0x40,
	NACK_RECEIVED = 0x80,
	DATA_RECEIVED = 0x0100,
	WAIT_FOR_LSB = 0x0200,
	LOAD_IMU_CONFIG_ADDR = 0x0400,
	LOAD_IMU_CONFIG_DATA = 0x0800,
	LOAD_IMU_CONFIG_NEXT = 0x1000,
	LOAD_IMU_START_WRITE = 0x2000
}event_flag_t;

extern uint32_t event_flag;

#endif /* EVENT_H_ */
