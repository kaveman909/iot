/*
 * imu.c
 *
 *  Created on: Apr 22, 2018
 *      Author: David
 */

#include "i2c.h"
#include "imu.h"
#include "event.h"
#include "native_gecko.h"

static uint32_t imu_config_index = 0;
static uint32_t imu_config_addr_vector[2][NUM_OF_CONFIG_CMDS] = {{
	IMU_PWR_MGMT_1,
	IMU_PWR_MGMT_2,
	IMU_ACCEL_CONFIG_2,
	IMU_INT_ENABLE,
	IMU_MOT_DETECT_CTRL,
	IMU_WOM_THR,
	IMU_LP_ACCEL_ODR,
	IMU_PWR_MGMT_1
},
{
	IMU_PWR_MGMT_1_CONFIG,
	IMU_PWR_MGMT_2_CONFIG,
	IMU_ACCEL_CONFIG_2_CONFIG,
	IMU_INT_ENABLE_CONFIG,
	IMU_MOT_DETECT_CTRL_CONFIG,
	IMU_WOM_THR_CONFIG,
	IMU_LP_ACCEL_ODR_CONFIG,
	IMU_PWR_MGMT_1_CONFIG2
}};

void imu_init(void) {
	event_flag |= LOAD_IMU_START_WRITE;
	gecko_external_signal(event_flag);
}

void imu_init_sched(void) {
	i2c_open();
	imu_start_write();
}

void imu_start_write(void) {
	// load slave address (writing)
	I2C0->TXDATA = (IMU_I2C_ADDRESS << 1) | WRITE_BIT;
	// start command
	I2C0->CMD = I2C_CMD_START;
	// wait for ack
	event_flag |= LOAD_IMU_CONFIG_ADDR;
	gecko_external_signal(event_flag);
}

void imu_config_addr(void) {
	// load the address we are configuring
	I2C0->TXDATA = imu_config_addr_vector[0][imu_config_index];
	// wait for ack
	event_flag |= LOAD_IMU_CONFIG_DATA;
	gecko_external_signal(event_flag);
}

void imu_config_data(void) {
	// load the data we are configuring
	I2C0->TXDATA = imu_config_addr_vector[1][imu_config_index];
	// wait for ack
	event_flag |= LOAD_IMU_CONFIG_NEXT;
	gecko_external_signal(event_flag);
}

void imu_config_next(void) {
	// stop the current configuration
	I2C0->CMD = I2C_CMD_STOP;
	while((I2C0->STATUS & I2C_STATUS_PSTOP) == I2C_STATUS_PSTOP);
	imu_config_index++;
	// start the config of the next element if we have more to do
	if (imu_config_index < NUM_OF_CONFIG_CMDS) {
		imu_start_write();
	} else {
		// done with i2c for now
		i2c_close();
	}
}
