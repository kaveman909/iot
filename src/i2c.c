/*
 * i2c.c
 *
 *  Created on: Feb 12, 2018
 *      Author: David
 */

#include <stdbool.h>
#include "i2c.h"
#include "em_i2c.h"
#include "cmu.h"
#include "gpio.h"
#include "user_sleep.h"

#define I2C0_SCL_Port gpioPortC
#define I2C0_SDA_Port gpioPortC
#define SENSOR_ENABLE_Port gpioPortD

#define I2C0_SCL_Pin 10
#define I2C0_SDA_Pin 11
#define SENSOR_ENABLE_Pin 9

#define I2C_SLAVE_ADDR 0x40 // address of Si7021
#define CMD_MEAS_TEMP_NO_HOLD 0xF3
#define WRITE_BIT 0
#define READ_BIT 1

static I2C_Init_TypeDef i2c_init = I2C_INIT_DEFAULT;
static volatile float temperature_degC;

static inline float raw_data_to_temp_degC(uint16_t raw_data) {
	float temp_degC = 175.72f * (float)raw_data;
	temp_degC /= 65536;
	temp_degC -= 46.85f;

	return temp_degC;
}

/** Setup the I2C GPIO pins.
 * Implementation adapted from Silicon Lab's AN0011SW Application Note.
 */
void i2c_setup(void) {
	/* Enable the high freq peripheral clock and I2C0 clock */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);

}


void i2c_open(void) {
	/* Setup SCL and SDA to be open-drain outputs, initialized to high */
	GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeWiredAnd, true);
	GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeWiredAnd, true);

	/* Toggle 9 times to reset any I2C slaves */
	for (int i = 0; i < 9; i++) {
		GPIO_PinOutClear(I2C0_SCL_Port, I2C0_SCL_Pin);
		GPIO_PinOutSet(I2C0_SCL_Port, I2C0_SCL_Pin);
	}

	/* Route the I2C0 appropriately.  SDA #16, SCL #14 per schematic */
	I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
	I2C0->ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC16 | I2C_ROUTELOC0_SCLLOC_LOC14;

	/* Initialize the I2C */
	I2C_Init(I2C0, &i2c_init);

	/* Reset I2C bus */
	if (I2C0->STATE & I2C_STATE_BUSY) {
		I2C0->CMD = I2C_CMD_ABORT;
	}
	/* I2C needs at least EM1 in Master Mode, so block EM2 */
	blockSleepMode(EM_I2C0 + 1);
	/* Clear interrupts */
	I2C_IntClear(I2C0, I2C_IFC_ACK | I2C_IFC_NACK);
	/* Setup interrupts */
	// I2C_IntEnable(I2C0);
	/* Enable interrupt in CPU */
	// NVIC_EnableIRQ(I2C0_IRQn);
}

/** run after temperature data has been collected to save power */
void i2c_close(void) {
	/* Reset (hi-z) SCL and SDA pins */
	GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeDisabled, false);
	GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeDisabled, false);

	// Turn off the sensor.  disabling the GPIO is OK because there is hw pull-down
	// on the enable pins.
	GPIO_PinModeSet(SENSOR_ENABLE_Port, SENSOR_ENABLE_Pin, gpioModeDisabled, false);

	unblockSleepMode(EM_I2C0 + 1);
}

void i2c_sensor_por(void) {
	GPIO_PinModeSet(SENSOR_ENABLE_Port, SENSOR_ENABLE_Pin, gpioModePushPull, true);
}

void i2c_measure_temp_blocking(void) {
	uint16_t data_lsb;
	uint16_t data_msb;
	// load slave address (writing)
	I2C0->TXDATA = (I2C_SLAVE_ADDR << 1) | WRITE_BIT;
	// start command
	I2C0->CMD = I2C_CMD_START;
	// wait for ack
	while(!(I2C0->IF & I2C_IF_ACK));
	I2C0->IFC = I2C_IFC_ACK;
	// load measure command
	I2C0->TXDATA = CMD_MEAS_TEMP_NO_HOLD;
	// wait for ack
	while(!(I2C0->IF & I2C_IF_ACK));
	I2C0->IFC = I2C_IFC_ACK;
	while(1) {
		// send repeat start command
		I2C0->CMD = I2C_CMD_START;
		// load slave address (reading)
		I2C0->TXDATA = (I2C_SLAVE_ADDR << 1) | READ_BIT;
		// slave will NACK until conversion complete
		while(!(I2C0->IF & I2C_IF_ACK) && !(I2C0->IF & I2C_IF_NACK));
		if (I2C0->IF & I2C_IF_ACK) {
			I2C0->IFC = I2C_IFC_ACK;
			// measurement is ready, wait for msb data
			while (!(I2C0->IF & I2C_IF_RXDATAV));
			data_msb = I2C0->RXDATA;
			// send ack
			I2C0->CMD = I2C_CMD_ACK;
			// wait for lsb data
			while (!(I2C0->IF & I2C_IF_RXDATAV));
			data_lsb = I2C0->RXDATA;
			// send nack, then stop to release the bus
			I2C0->CMD = I2C_CMD_NACK;
			while((I2C0->STATUS & I2C_STATUS_PNACK) == I2C_STATUS_PNACK);
			I2C0->CMD = I2C_CMD_STOP;
			while((I2C0->STATUS & I2C_STATUS_PSTOP) == I2C_STATUS_PSTOP);
			temperature_degC = raw_data_to_temp_degC((data_msb << 8) + data_lsb);
			break; // exit loop since we are done
		} else {
			I2C0->IFC = I2C_IFC_NACK;
			// keep waiting
		}
	}
}

void I2C0_IRQHandler(void) {

}