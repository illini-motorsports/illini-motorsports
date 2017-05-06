/**
 * FSAE Library TLC5955 Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef FSAE_tlc5955_H
#define FSAE_tlc5955_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"
#include "../Wheel_Node.X/Wheel.h"

#define NUM_BYTES 97 // 769 bits
#define NUM_LED_MAIN 9

#define CLUSTER_LEFT 0
#define CLUSTER_RIGHT 1

#define CLUSTER_WARN_INTV 150
#define MAIN_BLINK_INTV 75

// Pin definitions
#define LAT_TLC5955_TRIS TRISGbits.TRISG9
#define LAT_TLC5955_LAT  LATGbits.LATG9
#define PWM_TLC5955_TRIS TRISGbits.TRISG8
#define PWM_TLC5955_LAT  LATGbits.LATG8

// Function definitions
void tlc5955_set_main_blink(uint8_t on, uint64_t color);
void tlc5955_set_cluster_warn(uint8_t which, uint8_t on, uint64_t color);
void tlc5955_write_color(uint64_t color, uint16_t onMap);
void tlc5955_write_main_color(uint64_t color);
void tlc5955_write_main_colors(uint64_t* colors);

void tlc5955_check_timers();

void init_tlc5955(void);

void _tlc5955_write_control(void);
void _tlc5955_write_gs(void);
void _tlc5955_reg_append(uint8_t num_bits, uint8_t data);
void _tlc5955_send_register(void);

void _tlc5955_init_spi(void);

#endif /* FSAE_tlc5955_H */
