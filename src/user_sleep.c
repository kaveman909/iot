/*
 * sleep.c
 *
 *  Created on: Jan 31, 2018
 *      Author: David
 */
#include <stdbool.h>
#include <user_sleep.h>
#include "em_core.h"
#include "em_emu.h"

#define NUM_OF_SLEEP_MODES 5
static uint32_t sleep_block_counter[NUM_OF_SLEEP_MODES];

/** Credit given to SiLabs for the 'sleep' routine,
 * as this code is based off of their reference design */
void sleep(void) {
	if (sleep_block_counter[EM0]) {
		return;
	} else if (sleep_block_counter[EM1]) {
		return;
	} else if (sleep_block_counter[EM2]) {
		EMU_EnterEM1();
	} else if (sleep_block_counter[EM3]) {
		EMU_EnterEM2(true);
	} else {
		EMU_EnterEM3(true);
	}
}

void blockSleepMode(sleepstate_enum minimumMode) {
	CORE_ATOMIC_IRQ_DISABLE();
	sleep_block_counter[minimumMode]++;
	CORE_ATOMIC_IRQ_ENABLE();
}

void unblockSleepMode(sleepstate_enum minimumMode) {
	CORE_ATOMIC_IRQ_DISABLE();
	if (sleep_block_counter[minimumMode]) {
		sleep_block_counter[minimumMode]--;
	}
	CORE_ATOMIC_IRQ_ENABLE();
}

