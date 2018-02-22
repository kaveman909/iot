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
#include "infrastructure.h"
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
static const gecko_configuration_t config = { .config_flags = 0, .sleep.flags =
		SLEEP_FLAGS_DEEP_SLEEP_ENABLE, .bluetooth.max_connections =
		MAX_CONNECTIONS, .bluetooth.heap = bluetooth_stack_heap,
		.bluetooth.heap_size = sizeof(bluetooth_stack_heap),
		.bluetooth.sleep_clock_accuracy = 100, // ppm
		.gattdb = &bg_gattdb_data, .ota.flags = 0, .ota.device_name_len = 3,
		.ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
		.pa.config_enable = 1, // Enable high power PA
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT,// Configure PA input to VBAT
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
static uint8_t connection;
//***********************************************************************************
// function prototypes
//***********************************************************************************

//***********************************************************************************
// functions
//***********************************************************************************

//***********************************************************************************
// main
//***********************************************************************************

/** Source code for "temperatureMeasure" function modified from SiLabs Example Project "soc-thermometer"
 * Due credit is given to SiLabs. */
static void temperatureMeasure(void) {
	uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
	uint8_t flags = 0x00; /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
	int32_t tempData; /* Stores the Temperature data read from the RHT sensor. */
	uint32_t temperature; /* Stores the temperature data read from the sensor in the correct format */
	uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */

	/* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
	UINT8_TO_BITSTREAM(p, flags);
	tempData = i2c_get_temperature_deg_mC();
	/* Convert sensor data to correct temperature format */
	temperature = FLT_TO_UINT32(tempData, -3);
	/* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
	UINT32_TO_BITSTREAM(p, temperature);
	/* Send indication of the temperature in htmTempBuffer to all "listening" clients.
	 * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
	 *  0xFF as connection ID will send indications to all connections. */
	gecko_cmd_le_connection_get_rssi(connection);
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
			gattdb_temperature_measurement, 5, htmTempBuffer);
}

/**
 * @brief  Main function
 */
int main(void) {
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
	while (1) {
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;
		int16_t tx_level;
		static int8_t rssi_hist = 100;
		int8_t rssi;
		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
		/* This boot event is generated when the system boots up after reset.
		 * Do not call any stack commands before receiving the boot event.
		 * Here the system is set to start advertising immediately after boot procedure. */
		case gecko_evt_system_boot_id:

			/* Set advertising parameters. 100ms advertisement interval. All channels used.
			 * The first two parameters are minimum and maximum advertising interval, both in
			 * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
			gecko_cmd_le_gap_set_adv_parameters((uint16_t)(ADVERT_MIN_MS * 1.6f), (uint16_t)(ADVERT_MAX_MS * 1.6f), 7);

			/* Start general advertising and enable connections. */
			gecko_cmd_le_gap_set_mode(le_gap_general_discoverable,
					le_gap_undirected_connectable);

			/* Reset output power to 0 dBm */
			gecko_cmd_system_set_tx_power(0);
			break;

		case gecko_evt_le_connection_opened_id:
			connection = evt->data.evt_le_connection_opened.connection;
			gecko_cmd_le_connection_set_parameters(connection, CON_INT_MIN, CON_INT_MAX, SLAVE_LATENCY, SUP_TIMEOUT);
			gecko_cmd_le_connection_get_rssi(connection);
			break;

		case gecko_evt_le_connection_closed_id:
			/* Reset output power to 0 dBm */
			gecko_cmd_system_set_tx_power(0);
			/* Check if need to boot to dfu mode */
			if (boot_to_dfu) {
				/* Enter to DFU OTA mode */
				gecko_cmd_system_reset(2);
			} else {
				/* Restart advertising after client has disconnected */
				gecko_cmd_le_gap_set_mode(le_gap_general_discoverable,
						le_gap_undirected_connectable);
			}
			break;

			/* Events related to OTA upgrading
			 ----------------------------------------------------------------------------- */

			/* Check if the user-type OTA Control Characteristic was written.
			 * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
		case gecko_evt_gatt_server_user_write_request_id:

			if (evt->data.evt_gatt_server_user_write_request.characteristic
					== gattdb_ota_control) {
				/* Set flag to enter to OTA mode */
				boot_to_dfu = 1;
				/* Send response to Write Request */
				gecko_cmd_gatt_server_send_user_write_response(
						evt->data.evt_gatt_server_user_write_request.connection,
						gattdb_ota_control, bg_err_success);

				/* Close connection to enter to DFU OTA mode */
				gecko_cmd_endpoint_close(
						evt->data.evt_gatt_server_user_write_request.connection);
			}
			break;
		case gecko_evt_le_connection_rssi_id:
			rssi = evt->data.evt_le_connection_rssi.rssi;
			if (rssi > -35) {
				tx_level = -26; // -26dBm min from datasheet
			} else if (rssi > -45) {
				tx_level = -20;
			} else if (rssi > -55) {
				tx_level = -15;
			} else if (rssi > -65) {
				tx_level = -5;
			} else if (rssi > -75) {
				tx_level = 0;
			} else if (rssi > -85) {
				tx_level = 5;
			} else {
				tx_level = 8; // +8dBm max from datasheet
			}
			if (rssi != rssi_hist) {
				gecko_cmd_system_set_tx_power(tx_level * 10);
			}
			rssi_hist = rssi;
			break;
		case gecko_evt_system_external_signal_id:
			if (event_flag & START_TEMPERATURE_POR) {
				event_flag &= ~START_TEMPERATURE_POR;
				i2c_sensor_por();
			}
			if (event_flag & START_TEMPERATURE_QUERY) {
				event_flag &= ~START_TEMPERATURE_QUERY;
				i2c_open();
				i2c_start_measurement();
			}
			if (event_flag & FINISH_TEMPERATURE_QUERY) {
				event_flag &= ~FINISH_TEMPERATURE_QUERY;
				i2c_finish_measurement();
			}
			if ((event_flag & (WAIT_FOR_MEASUREMENT | NACK_RECEIVED))
					== (WAIT_FOR_MEASUREMENT | NACK_RECEIVED)) {
				event_flag &= ~(WAIT_FOR_MEASUREMENT | NACK_RECEIVED);
				i2c_finish_measurement();
			}
			if ((event_flag & (WAIT_FOR_MEASUREMENT | DATA_RECEIVED))
					== (WAIT_FOR_MEASUREMENT | DATA_RECEIVED)) {
				event_flag &= ~(WAIT_FOR_MEASUREMENT | DATA_RECEIVED);
				i2c_handle_first_byte();
			}
			if ((event_flag & (WAIT_FOR_LSB | DATA_RECEIVED))
					== (WAIT_FOR_LSB | DATA_RECEIVED)) {
				event_flag &= ~(WAIT_FOR_LSB | DATA_RECEIVED);
				i2c_handle_second_byte();
				i2c_close();
				letimer_reset_compare1();
				temperatureMeasure();
				// clear interrupt-generated flags
				event_flag &= ~(NACK_RECEIVED | ACK_RECEIVED | DATA_RECEIVED);
			}
			if ((event_flag & (LOAD_MEASURE_CMD | ACK_RECEIVED))
					== (LOAD_MEASURE_CMD | ACK_RECEIVED)) {
				event_flag &= ~(LOAD_MEASURE_CMD | ACK_RECEIVED);
				i2c_load_measure_cmd();
			}
			if ((event_flag & (LOAD_STOP_CMD | ACK_RECEIVED))
					== (LOAD_STOP_CMD | ACK_RECEIVED)) {
				event_flag &= ~(LOAD_STOP_CMD | ACK_RECEIVED);
				i2c_load_stop_cmd();
				letimer_update_compare1();
			}
			break;
		default:
			break;
		}
	}
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
