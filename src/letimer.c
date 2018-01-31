/*
 * letimer.c
 *
 *  Created on: Jan 31, 2018
 *      Author: David
 */

#include <stdbool.h>
#include "letimer.h"
#include "sleep.h"
#include "cmu.h"

/** lowest mode the LETIMER is allowed to operate in.
 *  Valid range: EM0 - EM3
 */
#define EM_LETIMER EM0

static const LETIMER_Init_TypeDef g_letimer_init = {
	.enable = false, // keep off at initialization time
	.bufTop = false,
	.comp0Top = true, // load COMP0 into CNT when underflow
	.debugRun = true,
	.repMode = letimerRepeatFree,
	.ufoa0 = letimerUFOANone,
	.ufoa1 = letimerUFOANone
};

void letimer_clock_init(void) {
	if(EM_LETIMER < EM3) {
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	}
	else {
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	}
	CMU_ClockEnable(cmuClock_LFA, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
}

void letimer_init(void) {
	// Setup compare registers
	LETIMER_CompareSet(LETIMER0, 0, 0x8000);
	LETIMER_CompareSet(LETIMER0, 1, 0x4000);
	// Initialize the LETIMER
	LETIMER_Init(LETIMER0, &g_letimer_init);
	// wait for synchronization
	while(LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD);
	// clear all pending interrupts
	LETIMER_IntClear(LETIMER0, (LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1 |
			LETIMER_IFC_UF | LETIMER_IFC_REP0 | LETIMER_IFC_REP1));
	// enable select interrupts
	LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1));
}


