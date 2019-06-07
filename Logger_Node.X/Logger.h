/**
 * Logger Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <sys/types.h>
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/errno.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_ltc3350.h"
#include "../FSAE.X/FSAE_nvm.h"
#include "../FSAE.X/FSAE_spi.h"

// Thresholds
#define RPM_ON_THRESHOLD 200.0 // rpm

// Timing constants (ms)
#define TEMP_SAMP_INTV     333
#define RAIL_SAMP_INTV     500
#define DIAG_SEND          1000
#define RAIL_SEND          500

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Pin definitions

#define SHDN_TRIS TRISCbits.TRISC3
#define SHDN_LAT  LATCbits.LATC3

#define ADC_VBAT_TRIS  TRISBbits.TRISB6
#define ADC_VBAT_ANSEL ANSELBbits.ANSB6
#define ADC_VBAT_CSS   ADCCSS1bits.CSS1
#define ADC_VBAT_CHN   46
#define ADC_VBAK_TRIS  TRISBbits.TRISB7
#define ADC_VBAK_ANSEL ANSELBbits.ANSB7
#define ADC_VBAK_CSS   ADCCSS1bits.CSS2
#define ADC_VBAK_CHN   47
#define ADC_VSUP_TRIS  TRISAbits.TRISA9
#define ADC_VSUP_ANSEL ANSELAbits.ANSA9
#define ADC_VSUP_CSS   ADCCSS1bits.CSS27
#define ADC_VSUP_CHN   27
#define ADC_5V_TRIS    TRISAbits.TRISA10
#define ADC_5V_ANSEL   ANSELAbits.ANSA10
#define ADC_5V_CSS     ADCCSS1bits.CSS28
#define ADC_5V_CHN     28
#define ADC_3V3_TRIS   TRISFbits.TRISF13
#define ADC_3V3_ANSEL  ANSELFbits.ANSF13
#define ADC_3V3_CSS    ADCCSS1bits.CSS30
#define ADC_3V3_CHN    30

#define SD_CS_TRIS           TRISGbits.TRISG9
#define SD_CS_LAT            LATGbits.LATG9
#define SD_CS_LATBITS        (uint32_t*) (&LATGbits)
#define SD_CS_LATNUM         9 


/**
 * Function definitions
 */

void main(void);

// Logic functions
void process_CAN_msg(CAN_message msg);

// ADC sample functions
void sample_temp(void);
void sample_rail(void);

// CAN message sending functions
void send_diag_can(void);
void send_rail_can(void);

// Utility functions
void init_adc_logger(void);

typedef union sdControlReg {
  struct {
    unsigned THING1:8;
    unsigned THING2:8;
  };
  uint16_t reg;
} SDControlReg;

SPIConn* init_sd(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);
void sd_write(SPIConn *conn,uint8_t cmd,uint8_t crc, uint32_t input);
void sd_write_CAN(SPIConn *conn,uint64_t message[], uint64_t rec[]);


#endif /* LOGGER_H */
