#ifndef _TEMPLATE_H    /* Guard against multiple inclusion */
#define _TEMPLATE_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"


#define PIC_LED_TRIS  TRISBbits.TRISB6
#define PIC_LED_LAT LATBbits.LATB6

int main(void);
void process_CAN_msg(CAN_message msg);

#endif /* _TEMPLATE_H */
