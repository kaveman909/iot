/*
 * letimer.c
 *
 *  Created on: Jan 31, 2018
 *      Author: David
 */

#include <stdbool.h>
#include <user_sleep.h>
#include "letimer.h"
#include "cmu.h"
#include "gpio.h"

/** lowest mode the LETIMER is allowed to operate in.
 *  Valid range: EM0 - EM3
 */
#define EM_LETIMER EM3
/** Period of the LED0 toggle, in seconds */
#define LED0_PERIOD (3.0f)
/** On time of the LED0 toggle, in seconds */
#define LED0_ON_TIME (0.075f)

static const LETIMER_Init_TypeDef g_letimer_init = {
	.enable = false, // keep off at initialization time
	.bufTop = false,
	.comp0Top = true, // load COMP0 into CNT when underflow
	.debugRun = true,
	.repMode = letimerRepeatFree,
	.ufoa0 = letimerUFOANone,
	.ufoa1 = letimerUFOANone
};

/* source:  https://stackoverflow.com/questions/101439/
 * the-most-efficient-way-to-implement-an-integer-based-power-function-powint-int
 */
static uint32_t ipow(uint32_t base, uint32_t exp)
{
    uint32_t result = 1;

    while (exp)
    {
        if (exp & 1) {
            result *= base;
        }
        exp >>= 1;
        base *= base;
    }
    return result;
}

static uint32_t get_letimer_period(float period_s) {
	uint32_t clockFreq;
	uint32_t targetPeriod;
	uint32_t prescaler = 0;
	uint32_t prescalerMult = 1;

	clockFreq = CMU_ClockFreqGet(cmuClock_LFA);
	// calculate prescaler
	targetPeriod = (uint32_t)((float)clockFreq * period_s);
	while ((targetPeriod / prescalerMult) > 0xFFFF) {
		prescalerMult *= 2;
		prescaler++;
	}
	//CMU_ClockPrescSet(cmuClock_LETIMER0, prescaler);
	CMU->LFAPRESC0 = prescaler;
	return (targetPeriod / prescalerMult);
}

static uint32_t get_letimer_on_time(float on_time_s) {
	uint32_t clockFreq;
	float targetOnTime;
	uint32_t prescaler;
	uint32_t prescalerMult;

	clockFreq = CMU_ClockFreqGet(cmuClock_LFA);
	//prescaler = CMU_ClockPrescGet(cmuClock_LETIMER0);
	prescaler = CMU->LFAPRESC0;
	prescalerMult = ipow(2, prescaler);
	targetOnTime = ((float)clockFreq * on_time_s) / (float)prescalerMult;

	return (uint32_t)targetOnTime;
}

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
	LETIMER_CompareSet(LETIMER0, 0, get_letimer_period(LED0_PERIOD));
	LETIMER_CompareSet(LETIMER0, 1, get_letimer_on_time(LED0_ON_TIME));
	// Initialize the LETIMER
	LETIMER_Init(LETIMER0, &g_letimer_init);
	// wait for synchronization
	while(LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD);
	// clear all pending interrupts
	LETIMER_IntClear(LETIMER0, (LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1 |
			LETIMER_IFC_UF | LETIMER_IFC_REP0 | LETIMER_IFC_REP1));
	// enable select interrupts
	LETIMER_IntEnable(LETIMER0, (LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1));
	// Block the next sleep mode (ex. timer configured for EM0, block EM1).
	blockSleepMode(EM_LETIMER + 1);
	// Enable the LETIMER0 Interrupt
	NVIC_EnableIRQ(LETIMER0_IRQn);
	// Start the timer
	LETIMER_Enable(LETIMER0, true);
}

void LETIMER0_IRQHandler(void) {
	uint32_t flags = LETIMER_IntGet(LETIMER0);
	LETIMER_IntClear(LETIMER0, flags);
	if (flags & LETIMER_IFC_COMP0) {
		GPIO_PinOutClear(LED0_port, LED0_pin);
	} else if (flags & LETIMER_IFC_COMP1) {
		GPIO_PinOutSet(LED0_port, LED0_pin);
	}
}






