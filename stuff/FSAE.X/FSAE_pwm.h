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

public volatile uint8_t load1;
public volatile uint8_t load2;
public volatile uint8_t load3;
public volatile uint8_t load4;
public volatile uint8_t load5;
public volatile uint8_t load6;
public volatile uint8_t load7;
public volatile uint8_t load8;
public volatile uint8_t load9;
public volatile uint8_t load10;
public volatile uint8_t load11;
public volatile uint8_t load12;

void pwm_set(uint16_t duty_cycle, uint8_t OC);
void init_pwm(uint16_t period, uint8_t OC);
uint8_t pwm_load(uint8_t load);

#endif