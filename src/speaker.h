/*
 * speaker.h
 *
 *  Created on: Apr 29, 2018
 *      Author: David
 */

#ifndef SPEAKER_H_
#define SPEAKER_H_

void LETIMER_setup(void);
void speaker_enable(bool enable);
uint16_t speaker_set_freq(uint16_t freq_Hz);
uint16_t speaker_set_duty(float duty);

#endif /* SPEAKER_H_ */
