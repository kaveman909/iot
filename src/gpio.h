//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef _GPIO_H
#define _GPIO_H

#include "em_gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED0 pin is
#define	LED0_port		gpioPortF
#define LED0_pin		4
#define LED0_default	false 	// off
// LED1 pin is
#define LED1_port		gpioPortF
#define LED1_pin		5
#define LED1_default	false	// off
// Bright White LED pin is
// (On WSTK:  P4)
#define LED_BW_port     gpioPortD
#define LED_BW_pin      10
#define LED_BW_default  false   // off
// IMU INT Pin is
// (On WSTK:  P6)
#define IMU_INT_port     gpioPortD
#define IMU_INT_pin      11
// Speaker PWM pin is
// (On WSTK:  P2)
#define SPEAKER_port     gpioPortA
#define SPEAKER_pin      3
#define SPEAKER_loc      3 // LETIM0_OUT0 location #3

// Speaker Enable pin is
// (On WSTK:  P8)
#define SPEAKER_EN_port    gpioPortD
#define SPEAKER_EN_pin     12
#define SPEAKER_EN_default false

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_init(void);

#endif // _GPIO_H

