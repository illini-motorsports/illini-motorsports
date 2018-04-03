/* 
 * File:   FSAE_pwm.h
 * Author: Jacob Drewniak
 *
 * Created on January 30, 2018, 10:22 PM
 */

#ifndef FSAE_PWM_H
#define	FSAE_PWM_H


#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"

void pwm_set(uint16_t duty_cycle, uint8_t OC);
void init_pwm(uint16_t period, uint8_t OC);

#endif