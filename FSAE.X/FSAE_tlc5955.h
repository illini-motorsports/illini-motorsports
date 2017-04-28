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

#define NUM_BYTES 97 // 769 bits

// Pin definitions
#define LAT_TLC5955_TRIS TRISGbits.TRISG9
#define LAT_TLC5955_LAT  LATGbits.LATG9
#define PWM_TLC5955_TRIS TRISGbits.TRISG8
#define PWM_TLC5955_LAT  LATGbits.LATG8

// Function definitions
void init_tlc5955(void);
void _tlc5955_init_spi(void);
void _tlc5955_write_control(void);
void _tlc5955_write_gs(uint8_t color_idx);
void _tlc5955_write_same_color(uint64_t color, uint16_t onMap);
void _tlc5955_write_colors(uint64_t * colors);
void _tlc5955_reg_append(uint8_t num_bits, uint8_t data);
void _tlc5955_send_register(void);

#endif /* FSAE_tlc5955_H */
