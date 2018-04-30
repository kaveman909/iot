/*
 * ps_keys.h
 *
 *  Created on: Apr 2, 2018
 *      Author: David
 */

#ifndef PS_KEYS_H_
#define PS_KEYS_H_

#include "gatt_db.h"

// LED Control
#define ps_key_base 0x4000
#define default_led_blink_rate	 50
#define ps_key_led_blink_rate    (gattdb_led_blink_rate + ps_key_base)
#define default_led_intensity    60
#define ps_key_led_intesity      (gattdb_led_intensity + ps_key_base)

// Speaker Control
#define default_speaker_pitch    20 // 0.1 kHz
#define ps_key_speaker_pitch     (gattdb_speaker_pitch + ps_key_base)
#define default_speaker_volume   50 // % duty cycle
#define ps_key_speaker_volume    (gattdb_speaker_volume + ps_key_base)

#endif /* PS_KEYS_H_ */
