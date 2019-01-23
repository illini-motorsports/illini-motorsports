#ifndef _DEVELOPMENT_H    /* Guard against multiple inclusion */
#define _DEVELOPMENT_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"
#include "../PDM_Node.X/PDM.h"


#define PIC_LED_TRIS  TRISBbits.TRISB6
#define PIC_LED_LAT LATBbits.LATB6
#define TEMP_CS_TRIS  TRISBbits.TRISB3
#define TEMP_CS_LAT LATBbits.LATB3
#define PWM_TRIS TRISBbits.TRISB2
#define PWM_LAT LATBbits.LATB2

#define PERD 0x03E8
volatile uint32_t millis;
volatile uint32_t periods;

typedef struct _kTemp{
  double temp;
  double jTemp;
  uint8_t fault;
} kTemp;

int main(void);
void init_spi();
uint32_t get_temp_spi();
void get_temp(kTemp *kt);
void pwm_set(uint16_t duty_cycle, uint8_t OC);
//int delay(int num);

#endif /* _DEVELOPMENT_H */
