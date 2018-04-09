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
#include "ps_keys.h"
//#include <stdio.h>

//***********************************************************************************
// defined files
//***********************************************************************************
#define LED_ON_TIME_MS        50
#define LED_PERIOD_STEP_MS    100
//#define LED_TIMEOUT_SEC       30

#define LED_ON_TIME_NS        (LED_ON_TIME_MS * 1000000UL)
#define LED_PERIOD_STEP_NS    (LED_PERIOD_STEP_MS * 1000000UL)
#define LFO_HZ                32768
#define LFO_NS                30518
#define LED_ON_TIME_LFO       (LED_ON_TIME_NS/LFO_NS)
#define LED_PERIOD_STEP_LFO   (LED_PERIOD_STEP_NS/LFO_NS)
//#define LED_TIMEOUT_LFO       (LED_TIMEOUT_SEC * LFO_HZ)

#define LED_BLINK_RATE_HANDLE 1
#define LED_TIMEOUT_HANDLE    2

// Private Types
typedef union {
	uint8_t data[4];
	struct {
		uint8_t led_blink_rate;
		uint8_t led_intensity;
		uint8_t speaker_pitch;
		uint8_t speaker_volume;
	} s;
} ps_data_t;

//***********************************************************************************
// global variables
//***********************************************************************************
static uint8_t connection;
//volatile static uint8_t attribute_data;
static const uint8_t gattdb_ps_data[] = {
		gattdb_led_blink_rate, gattdb_led_intensity, gattdb_speaker_pitch, gattdb_speaker_volume
};
static const uint8_t gattdb_ps_default_data[] = {
		default_led_blink_rate, default_led_intensity, default_speaker_pitch, default_speaker_volume
};
static ps_data_t ps_data;
volatile static uint32_t passkey;
static uint8_t bonding_handle;

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
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
			gattdb_temperature_measurement, 5, htmTempBuffer);
}

void ps_keys_init(uint8_t ps_att_len, const uint8_t * ps_att_data, const uint8_t * ps_att_default_data, ps_data_t * ps_data) {
	for (int i = 0; i < ps_att_len; i++) {
		//gecko_cmd_flash_ps_erase(ps_key_led_blink_rate);
		struct gecko_msg_flash_ps_load_rsp_t * temp = gecko_cmd_flash_ps_load(ps_att_data[i] + ps_key_base);
		if (temp->result != 0) {
			// Write the default data, since this the PS Key is not valid yet (first time after flashing)
			gecko_cmd_gatt_server_write_attribute_value(ps_att_data[i], 0, 1, (ps_att_default_data + i));
			ps_data->data[i] = *(ps_att_default_data + i);
		} else {
			// The PS Key is valid; load the saved data to the attribute database
			gecko_cmd_gatt_server_write_attribute_value(ps_att_data[i], 0, temp->value.len, temp->value.data);
			ps_data->data[i] = temp->value.data[0];
		}
	}
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
		static int16_t tx_level_hist = 0;
		int8_t rssi;
		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
		/* This boot event is generated when the system boots up after reset.
		 * Do not call any stack commands before receiving the boot event.
		 * Here the system is set to start advertising immediately after boot procedure. */
		case gecko_evt_system_boot_id:
			gecko_cmd_sm_delete_bondings();
			//printf("System boot\r\n");
			/* Set up bonding (flags = 0b0111 = 0x07):
			 * (0:1) Bonding requires MITM protection
			 * (1:1) Encryption requires bonding
			 * (2:1) Secure connections only
			 * (3:0) Bonding request does not need to be confirmed
			 *
			 * Device only has a display (no keyboard)
			 */
			gecko_cmd_sm_configure(0x07, sm_io_capability_displayonly);
			/* Accept new bondings */
			gecko_cmd_sm_set_bondable_mode(1);
			/* Set hard-coded passkey for now */
			gecko_cmd_sm_set_passkey(123456);
			/* Set advertising parameters. 100ms advertisement interval. All channels used.
			 * The first two parameters are minimum and maximum advertising interval, both in
			 * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
			gecko_cmd_le_gap_set_adv_parameters((uint16_t)(ADVERT_MIN_MS * 1.6f), (uint16_t)(ADVERT_MAX_MS * 1.6f), 7);

			/* Start general advertising and enable connections. */
			gecko_cmd_le_gap_set_mode(le_gap_general_discoverable,
					le_gap_undirected_connectable);

			/* Reset output power to 0 dBm */
			gecko_cmd_system_set_tx_power(0);

			/* Initialize PS data */

			ps_keys_init(sizeof(gattdb_ps_data), gattdb_ps_data, gattdb_ps_default_data, &ps_data);
			break;

		case gecko_evt_sm_passkey_display_id:
			passkey = evt->data.evt_sm_passkey_display.passkey;
			break;

		case gecko_evt_sm_bonded_id:
			break;

		case gecko_evt_sm_bonding_failed_id:
			gecko_cmd_sm_bonding_confirm(connection, false);
			gecko_cmd_le_connection_close(connection);
			/* Ensure that bond is deleted at this point */
			if (bonding_handle != 0xff) {
				gecko_cmd_sm_delete_bonding(bonding_handle);
			}
			break;

		case gecko_evt_le_connection_opened_id:
			connection = evt->data.evt_le_connection_opened.connection;
			/* Increasing security triggers the pairing process */
			gecko_cmd_sm_increase_security(connection);
			bonding_handle = evt->data.evt_le_connection_opened.bonding;
			if (bonding_handle != 0xff) {
				gecko_cmd_le_connection_set_parameters(connection, CON_INT_MIN, CON_INT_MAX, SLAVE_LATENCY, SUP_TIMEOUT);
			}
			gecko_cmd_le_connection_get_rssi(connection);
			gecko_cmd_hardware_set_soft_timer(32768/2, 0, 0);
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
			gecko_cmd_hardware_set_soft_timer(0, 0, 0);
			break;

			/* Events related to OTA upgrading
			 ----------------------------------------------------------------------------- */

		case gecko_evt_hardware_soft_timer_id:
				if (evt->data.evt_hardware_soft_timer.handle == 0) {
					gecko_cmd_le_connection_get_rssi(connection);
				} else if (evt->data.evt_hardware_soft_timer.handle == LED_BLINK_RATE_HANDLE) {
					if(GPIO_PinOutGet(LED_BW_port, LED_BW_pin)) {
						GPIO_PinOutClear(LED_BW_port, LED_BW_pin);
						gecko_cmd_hardware_set_soft_timer((LED_PERIOD_STEP_LFO * ps_data.s.led_blink_rate) - LED_ON_TIME_LFO, LED_BLINK_RATE_HANDLE, true);
					} else {
						GPIO_PinOutSet(LED_BW_port, LED_BW_pin);
						gecko_cmd_hardware_set_soft_timer(LED_ON_TIME_LFO, LED_BLINK_RATE_HANDLE, true);
					}
				} else if (evt->data.evt_hardware_soft_timer.handle == LED_TIMEOUT_HANDLE) {
					GPIO_PinOutClear(LED_BW_port, LED_BW_pin);
					gecko_cmd_hardware_set_soft_timer(0, LED_BLINK_RATE_HANDLE, true);
				}
		        break;

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

		case gecko_evt_gatt_server_attribute_value_id:
			// Attribute value has changed; check if persistent storage needs to be updated
			for (int i = 0; i < sizeof(gattdb_ps_data); i++) {
				if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_ps_data[i]) {
					//GPIO_PinOutToggle(LED0_port, LED0_pin);
					struct gecko_msg_gatt_server_read_attribute_value_rsp_t * temp =
							gecko_cmd_gatt_server_read_attribute_value(gattdb_ps_data[i], 0);
					gecko_cmd_flash_ps_save(gattdb_ps_data[i] + ps_key_base, temp->value.len, temp->value.data);
					ps_data.data[i] = temp->value.data[0];
					break; // no need to continue looping; at most one attribute has changed
				}
			}
			if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_led_on_off) {
				// Check if we should turn LED on for a short period
				struct gecko_msg_gatt_server_read_attribute_value_rsp_t * temp =
						gecko_cmd_gatt_server_read_attribute_value(gattdb_led_on_off, 0);
				if (temp->value.data[0] == 1) {
					// Enable LED pulsing
					GPIO_PinOutSet(LED_BW_port, LED_BW_pin);
					gecko_cmd_hardware_set_soft_timer(LED_ON_TIME_LFO, LED_BLINK_RATE_HANDLE, true);
					gecko_cmd_hardware_set_soft_timer((ps_data.s.led_intensity * LFO_HZ), LED_TIMEOUT_HANDLE, true);
				}
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
			if (tx_level != tx_level_hist) {
				gecko_cmd_system_set_tx_power(tx_level * 10);
				gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
					gattdb_tx_power_level, 1, (uint8_t *)&tx_level);
			}
			tx_level_hist = tx_level;
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
