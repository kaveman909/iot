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

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_init(void);

#endif // _GPIO_H

