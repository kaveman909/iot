//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"
#include "gpiointerrupt.h"
#include "event.h"
#include "native_gecko.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

void gpio_interrupt_callback(uint8_t pin) {
	if (pin == IMU_INT_pin) {
		event_flag |= IMU_MOTION_INTERRUPT;
		gecko_external_signal(event_flag);
	}
}

void gpio_init(void){

#if 0
	// Set LED ports to be standard output drive with default off (cleared)
  //GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);
#endif
	GPIO_DriveStrengthSet(LED_BW_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_PinModeSet(LED_BW_port, LED_BW_pin, gpioModePushPull, LED_BW_default);

	GPIO_PinModeSet(IMU_INT_port, IMU_INT_pin, gpioModeInputPullFilter, 1);
	GPIO_ExtIntConfig(IMU_INT_port, IMU_INT_pin, IMU_INT_pin, true, false, true);

	GPIO_DriveStrengthSet(SPEAKER_EN_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_PinModeSet(SPEAKER_EN_port, SPEAKER_EN_pin, gpioModePushPull, SPEAKER_EN_default);

	GPIOINT_Init();
	GPIOINT_CallbackRegister(IMU_INT_pin, gpio_interrupt_callback);
}


