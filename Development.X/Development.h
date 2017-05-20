#ifndef _DEVELOPMENT_H    /* Guard against multiple inclusion */
#define _DEVELOPMENT_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"


#define PIC_LED_TRIS  TRISBbits.TRISB6
#define PIC_LED_LAT LATBbits.LATB6
#define TEMP_CS_TRIS  TRISBbits.TRISB3
#define TEMP_CS_LAT LATBbits.LATB3

typedef struct _kTemp{
  double temp;
  double jTemp;
  uint8_t fault;
} kTemp;

int main(void);
void init_spi();
uint32_t get_temp_spi();
void get_temp(kTemp *kt);

#endif /* _DEVELOPMENT_H */
