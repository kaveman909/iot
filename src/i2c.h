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
void i2c_measure_temp_blocking(void);
void i2c_setup(void);
void i2c_close(void);
void i2c_sensor_por(void);

#endif /* I2C_H_ */