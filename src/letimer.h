/*
 * letimer.h
 *
 *  Created on: Jan 31, 2018
 *      Author: David
 */

#ifndef LETIMER_H_
#define LETIMER_H_

#include "em_letimer.h"

void letimer_clock_init(void);
void letimer_init(void);
void letimer_update_compare1(void);
void letimer_reset_compare1(void);

#endif /* LETIMER_H_ */
