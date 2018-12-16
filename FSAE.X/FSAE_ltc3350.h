/**
 * FSAE Library LTC3350 Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jacob Drewniak
 * Created:     2018-2019
 */
#ifndef FSAE_LTC3350_H
#define FSAE_LTC3350_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"

#define LTC3350_DEV_ADDR 0b0001001
#define PIC_DEV_ADDR     0b0011100
#define SETUP_TIME       3
#define HOLD_TIME        4

// Pin definitions
#define CHG_PFO_TRIS  TRISCbits.TRISC4
#define CHG_PFO_ANSEL ANSELCbits.ANSC4
#define CHG_PFO_PORT  PORTCbits.RC4
#define CHG_ALM_TRIS  TRISCbits.TRISC2
#define CHG_ALM_ANSEL ANSELCbits.ANSC2
#define CHG_ALM_PORT  PORTCbits.RC2
#define CHG_CPGD_TRIS  TRISGbits.TRISG9
#define CHG_CPGD_ANSEL ANSELGbits.ANSG9
#define CHG_CPGD_PORT  PORTCGits.RG9
#define DATA_LINE_TRIS TRISGbits.TRISG7
#define CLK_LINE_TRIS  TRISGbits.TRISG8
#define DATA_LINE_ANSEL ANSELGbits.ANSG7
#define CLK_LINE_ANSEL  ANSELGbits.ANSG8
#define DATA_LINE_LAT LATGbits.LATG7
#define CLK_LINE_LAT  LATGbits.LATG8
#define DATA_LINE_PORT PORTGbits.RG7
#define CLK_LINE_PORT  PORTGbits.RG8

volatile uint32_t micros;
int ack;
uint16_t data;
uint32_t current_time;

// Function definitions
void init_ltc3350(void);
uint16_t ltc3350_read(uint8_t addr);
void ltc3350_write(uint8_t addr, uint16_t data);
void _ltc3350_init_i2c(void);
uint16_t send_data_i2c(uint8_t blah);
uint16_t read_data_i2c();
void acknowledge(void);
void start_i2c(void);
void stop_i2c(void);
void setup_time(void);
void hold_time(void);

void delay();

#endif /* FSAE_LTC3350_H */
