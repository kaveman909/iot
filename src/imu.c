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

// globals
const uint16_t gyro_full_scale[4] = {250, 500, 1000, 2000};

static uint32_t imu_config_index = 0;
static uint32_t imu_config_addr_vector[2][NUM_OF_CONFIG_CMDS] = {{
	IMU_GYRO_CONFIG,
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
	IMU_GYRO_CONFIG_CONFIG,
	IMU_PWR_MGMT_1_CONFIG,
	IMU_PWR_MGMT_2_CONFIG,
	IMU_ACCEL_CONFIG_2_CONFIG,
	IMU_INT_ENABLE_CONFIG,
	IMU_MOT_DETECT_CTRL_CONFIG,
	IMU_WOM_THR_CONFIG,
	IMU_LP_ACCEL_ODR_CONFIG,
	IMU_PWR_MGMT_1_CONFIG2
}};

static int16_t gyro_result_buffer[NUM_OF_GYRO_REGISTERS/2];
static uint8_t gyro_result_index = 0;
static bool gyro_enable = false;

void imu_init(void) {
	event_flag |= LOAD_IMU_START_WRITE;
	gecko_external_signal(event_flag);
}

void imu_init_sched(void) {
	i2c_open();
	imu_start_write();
}

static void imu_enable_gyro_start_write(void) {
	// load slave address (writing)
	I2C0->TXDATA = (IMU_I2C_ADDRESS << 1) | WRITE_BIT;
	// start command
	I2C0->CMD = I2C_CMD_START;
	// wait for ack
	event_flag |= LOAD_IMU_EN_GYRO_ADDR;
	gecko_external_signal(event_flag);
}

int8_t imu_enable_gyro(bool enable) {
	if (i2c_get_lock() == false) {
		gyro_enable = enable;
		i2c_open();
		imu_enable_gyro_start_write();
		return GYRO_EN_SUCCESS;
	} else {
		return GYRO_EN_FAILURE;
	}
}

static bool gyro_enable_part2 = false;
void imu_en_gyro_addr(void) {
	// load the address we are configuring
	if (gyro_enable_part2) {
		I2C0->TXDATA = IMU_PWR_MGMT_1;
	} else {
		I2C0->TXDATA = IMU_PWR_MGMT_2;
	}
	// wait for ack
	event_flag |= LOAD_IMU_EN_GYRO_DATA;
	gecko_external_signal(event_flag);
}

void imu_en_gyro_data(void) {
	// load the data we are configuring
	if (gyro_enable) {
		if (gyro_enable_part2) {
			I2C0->TXDATA = IMU_PWR_MGMT_1_CONFIG;
		} else {
			I2C0->TXDATA = IMU_PWR_MGMT_2_EN_GYRO;
		}
	} else {
		if (gyro_enable_part2) {
			I2C0->TXDATA = IMU_PWR_MGMT_1_CONFIG2;
		} else {
			I2C0->TXDATA = IMU_PWR_MGMT_2_CONFIG;
		}
	}
	// wait for ack
	event_flag |= LOAD_IMU_EN_GYRO_DONE;
	gecko_external_signal(event_flag);
}

void imu_en_gyro_done(void) {
	// stop the current configuration
	I2C0->CMD = I2C_CMD_STOP;
	while((I2C0->STATUS & I2C_STATUS_PSTOP) == I2C_STATUS_PSTOP);
	if (!gyro_enable_part2) {
		gyro_enable_part2 = true;
		imu_enable_gyro_start_write();
	} else {
		gyro_enable_part2 = false;
		i2c_close();
	}
}

int8_t imu_read_gyro_start(void) {
	if (i2c_get_lock() == false) {
		i2c_open();
		// load slave address (writing)
		I2C0->TXDATA = (IMU_I2C_ADDRESS << 1) | WRITE_BIT;
		// start command
		I2C0->CMD = I2C_CMD_START;
		// wait for ack
		event_flag |= LOAD_IMU_READ_GYRO_ADDR;
		gecko_external_signal(event_flag);
		return GYRO_READ_SUCCESS;
	} else {
		return GYRO_READ_FAILURE;
	}
}

void imu_read_gyro_addr(void) {
	// load the address we are going to read
	I2C0->TXDATA = GYRO_XOUT_H; // first gyro result address for burst read
	// wait for ack
	event_flag |= LOAD_IMU_READ_GYRO_DATA;
	gecko_external_signal(event_flag);
}

void imu_read_gyro_data(void) {
	// send repeated start command
	I2C0->CMD = I2C_CMD_START;
	// load slave address (reading)
	I2C0->TXDATA = (IMU_I2C_ADDRESS << 1) | READ_BIT;
	// wait for ack and data
	event_flag |= LOAD_IMU_READ_GYRO_DRDY;
	gecko_external_signal(event_flag);
}

void imu_read_gyro_drdy(void) {
	if (gyro_result_index & 1) {
		// odd
		((uint8_t *)(gyro_result_buffer))[gyro_result_index - 1] = i2c_get_rxdata();
	} else {
		// even
		((uint8_t *)(gyro_result_buffer))[gyro_result_index + 1] = i2c_get_rxdata();
	}
	gyro_result_index++;
	if (gyro_result_index < NUM_OF_GYRO_REGISTERS) {
		// get next byte
		// send an ack to slave
		I2C0->CMD = I2C_CMD_ACK;
		while((I2C0->STATUS & I2C_STATUS_PACK) == I2C_STATUS_PACK);
		// wait for data
		event_flag |= LOAD_IMU_READ_GYRO_DRDY;
		gecko_external_signal(event_flag);
	} else {
		gyro_result_index = 0;
		// received all the gyro data; send NACK then STOP condition
		I2C0->CMD = I2C_CMD_NACK;
		while((I2C0->STATUS & I2C_STATUS_PNACK) == I2C_STATUS_PNACK);
		I2C0->CMD = I2C_CMD_STOP;
		while((I2C0->STATUS & I2C_STATUS_PSTOP) == I2C_STATUS_PSTOP);
		// we are done with i2c for now, release peripheral
		i2c_close();

		// alert main event handler we are ready to process gyro data
		event_flag |= GYRO_PROCESS_DATA;
		gecko_external_signal(event_flag);
	}
}

/*
void imu_calculate_degPerSec(void) {
	for (int i = 0; i < (NUM_OF_GYRO_REGISTERS/2); i++) {
		gyro_degPerSec[i] = ((float)(gyro_result_buffer[i]) * (float)(gyro_full_scale[IMU_GYRO_FS_SEL]))/32768.0;
	}
}
*/

static float gyro_degPerSec[NUM_OF_GYRO_REGISTERS/2] = {0, 0, 0};
static float gyro_accum_degPerSec[NUM_OF_GYRO_REGISTERS/2] = {0, 0, 0};
static float gyro_avg_degPerSec[NUM_OF_GYRO_REGISTERS/2] = {0, 0, 0};

static int16_t gyro_degPerSec_i[NUM_OF_GYRO_REGISTERS/2] = {0, 0, 0};
static int16_t gyro_avg_degPerSec_i[NUM_OF_GYRO_REGISTERS/2] = {0, 0, 0};

static float gyro_avg_samples = 0;

void imu_update_avg(void) {
	gyro_avg_samples++;
	for (int i = 0; i < (NUM_OF_GYRO_REGISTERS/2); i++) {
		gyro_degPerSec[i] = ((float)(gyro_result_buffer[i]) * (float)(gyro_full_scale[IMU_GYRO_FS_SEL]))/32768.0;
		gyro_accum_degPerSec[i] += gyro_degPerSec[i];
		gyro_avg_degPerSec[i] = gyro_accum_degPerSec[i] / gyro_avg_samples;
		// update int versions
		gyro_degPerSec_i[i] = (int16_t)(gyro_degPerSec[i]);
		gyro_avg_degPerSec_i[i] = (int16_t)(gyro_avg_degPerSec[i]);
	}
}

void imu_clear_avg(void) {
	gyro_avg_samples = 0;
	for (int i = 0; i < (NUM_OF_GYRO_REGISTERS/2); i++) {
		gyro_degPerSec[i] = 0;
		gyro_accum_degPerSec[i] = 0;
		gyro_avg_degPerSec[i] = 0;
	}
}

float imu_get_gyro_data_avg(uint8_t axis) {
	return gyro_avg_degPerSec[axis];
}

float imu_get_gyro_data_rt(uint8_t axis) {
	return gyro_degPerSec[axis];
}

int16_t * imu_get_gyro_data_rt_int(void) {
	return gyro_degPerSec_i;
}

int16_t * imu_get_gyro_data_avg_int(void) {
	return gyro_avg_degPerSec_i;
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

