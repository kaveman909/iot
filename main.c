/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "gpio.h"
#include "cmu.h"
#include "letimer.h"
#include "user_sleep.h"
#include "event.h"
#include "i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************
extern event_flag_t event_flag;

//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************


//***********************************************************************************
// main
//***********************************************************************************

/**
 * @brief  Main function
 */
int main(void)
{
	// Initialize device
	initMcu();
	// Initialize board
	initBoard();

	/* Initialize GPIO */
	gpio_init();

	// Initialize clocks
	cmu_init();

	// Initialize stack
	gecko_init(&config);

	// Initialize LETIMER
	letimer_clock_init();
	letimer_init();
	i2c_setup();

	while(1) {
		if(event_flag & START_TEMPERATURE_POR) {
			event_flag &= ~START_TEMPERATURE_POR;
			i2c_sensor_por();
		} else if (event_flag & START_TEMPERATURE_QUERY) {
			event_flag &= ~START_TEMPERATURE_QUERY;
			i2c_open();
			i2c_start_measurement();
		} else if (event_flag & FINISH_TEMPERATURE_QUERY) {
			event_flag &= ~FINISH_TEMPERATURE_QUERY;
			i2c_finish_measurement();
		} else if ((event_flag & (WAIT_FOR_MEASUREMENT | NACK_RECEIVED)) == (WAIT_FOR_MEASUREMENT | NACK_RECEIVED)) {
			event_flag &= ~(WAIT_FOR_MEASUREMENT | NACK_RECEIVED);
			i2c_finish_measurement();
		} else if ((event_flag & (WAIT_FOR_MEASUREMENT | DATA_RECEIVED)) == (WAIT_FOR_MEASUREMENT | DATA_RECEIVED)) {
			event_flag &= ~(WAIT_FOR_MEASUREMENT | DATA_RECEIVED);
			i2c_handle_first_byte();
		} else if ((event_flag & (WAIT_FOR_LSB | DATA_RECEIVED)) == (WAIT_FOR_LSB | DATA_RECEIVED)) {
			event_flag &= ~(WAIT_FOR_LSB | DATA_RECEIVED);
			i2c_handle_second_byte();
			i2c_close();
			letimer_reset_compare1();
			// clear interrupt-generated flags
			event_flag &= ~(NACK_RECEIVED | ACK_RECEIVED | DATA_RECEIVED);
		} else if ((event_flag & (LOAD_MEASURE_CMD | ACK_RECEIVED)) == (LOAD_MEASURE_CMD | ACK_RECEIVED)) {
			event_flag &= ~(LOAD_MEASURE_CMD | ACK_RECEIVED);
			i2c_load_measure_cmd();
		} else if ((event_flag & (LOAD_STOP_CMD | ACK_RECEIVED)) == (LOAD_STOP_CMD | ACK_RECEIVED)) {
			event_flag &= ~(LOAD_STOP_CMD | ACK_RECEIVED);
			i2c_load_stop_cmd();
			letimer_update_compare1();
		} else if (event_flag == NO_EVENT) {
			sleep();
		}
	}
}
/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
