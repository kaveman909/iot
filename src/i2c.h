/*
 * i2c.h
 *
 *  Created on: Feb 12, 2018
 *      Author: David
 */

#ifndef I2C_H_
#define I2C_H_

#include "em_i2c.h"

#define EM_I2C0 EM1
#define WRITE_BIT 0
#define READ_BIT 1

void i2c_start_measurement(void);
void i2c_finish_measurement(void);
void i2c_setup(void);
void i2c_open(void);
void i2c_close(void);
void i2c_sensor_por(void);
void i2c_load_measure_cmd(void);
void i2c_load_stop_cmd(void);
void i2c_handle_first_byte(void);
void i2c_handle_second_byte(void);
int32_t i2c_get_temperature_deg_mC(void) ;

#endif /* I2C_H_ */
