#ifndef _DRS_H    /* Guard against multiple inclusion */
#define _DRS_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_pwm.h"

volatile uint32_t millis;
volatile uint32_t periods;
volatile uint32_t enable;

int main(void);
void process_CAN_msg(CAN_message msg);

#endif /* _DRS_H */

