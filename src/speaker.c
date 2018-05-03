/*
 * speaker.c
 *
 *  Created on: Apr 29, 2018
 *      Author: David
 */

#include "em_cmu.h"
#include "gpio.h"
#include "em_letimer.h"

/* LETIMER PWM Implementation based on AN0026 example code, "main_letimer_pwm_pulse.c" from Silicon Labs
 * https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/08/17/using_low_energytim-9RYB
 */

/* COMP1 is changed throughout the code to vary the PWM duty cycle
   but starts with the maximum value (100% duty cycle) */

static uint16_t speaker_frequency_Hz = 10000;
static float speaker_duty_cycle = 0.5;

static uint16_t comp1; // "duty cycle"
static uint16_t comp0; // "frequency"


void speaker_enable(bool enable) {
  if (enable) {
    LETIMER0->CMD = LETIMER_CMD_START;
    GPIO_PinOutSet(SPEAKER_EN_port, SPEAKER_EN_pin);
  } else {
    GPIO_PinOutClear(SPEAKER_EN_port, SPEAKER_EN_pin);
    LETIMER0->CMD = LETIMER_CMD_STOP;
  }
}

uint16_t speaker_set_freq(uint16_t freq_Hz) {
  bool running = LETIMER0->STATUS & LETIMER_STATUS_RUNNING ? true : false;
  LETIMER0->CMD = LETIMER_CMD_STOP;

  speaker_frequency_Hz = freq_Hz;
  comp0 = ((CMU_ClockFreqGet(cmuClock_LFA)  + (freq_Hz / 2)) / freq_Hz) - 1;
  comp1 = (uint16_t)((speaker_duty_cycle * (float)comp0) + 0.5f);
  LETIMER_CompareSet(LETIMER0, 0, comp0);
  LETIMER_CompareSet(LETIMER0, 1, comp1);
  if (running) {
    LETIMER0->CMD = LETIMER_CMD_START;
  }
  return comp0;
}

uint16_t speaker_set_duty(float duty) {
  bool running = LETIMER0->STATUS & LETIMER_STATUS_RUNNING ? true : false;
  LETIMER0->CMD = LETIMER_CMD_STOP;

  speaker_duty_cycle = duty;
  comp1 = (uint16_t)((speaker_duty_cycle * (float)comp0) + 0.5f);
  LETIMER_CompareSet(LETIMER0, 1, comp1);
  if (running) {
    LETIMER0->CMD = LETIMER_CMD_START;
  }
  return comp1;
}

void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_LETIMER0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PD6 and PD7 as push pull so the
     LETIMER can override them */
  GPIO_PinModeSet(SPEAKER_port, SPEAKER_pin, gpioModePushPull, 0);

  /* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
  LETIMER_RepeatSet(LETIMER0, 0, 0x01);
  LETIMER_RepeatSet(LETIMER0, 1, 0x01);

  /* Route LETIMER to location 3 (PA3) and enable outputs */
  LETIMER0->ROUTEPEN = LETIMER_ROUTEPEN_OUT0PEN;
  LETIMER0->ROUTELOC0 = LETIMER_ROUTELOC0_OUT0LOC_LOC3;

  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit =
  {
  .enable         = false,                   /* Start counting when init completed. */
  .debugRun       = true,                   /* Counter shall keep running during debug halt. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
  .repMode        = letimerRepeatFree       /* Count until stopped */
  };

  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit);

}

