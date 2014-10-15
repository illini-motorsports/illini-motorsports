/*
 * Shift Lights Source
 *
 * File Name:   ShiftLights.c
 * Processor:   PIC18F46K80
 * Compiler:    Microchip C18
 * Author:      Andrew Mass
 * Author:      George Schwieters
 * Created:     2013-2014
 */
#include <stdlib.h>
#include "p18F46K80.h"
#include "timers.h"
#include "ECAN.h"
#include "FSAE.h"
#include "CAN.h"
#include "ShiftLights.h"

//TODO: What to do if millis isn't updating.
//TODO: What to do if rmp isn't updating.
//TODO: Add "animations". (Scrolling, expanding, alternating, etc.)
//TODO: Add AMERICA sequence. Make it look 'murican.
//TODO: Optimize everything.

/*
 * PIC18F46K80 Configuration Bits
 */

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#ifdef INTERNAL
    #pragma config FOSC = INTIO2  // Oscillator (Internal RC oscillator)
#else
    #pragma config FOSC = HS1     // Oscillator (HS oscillator (Medium power, 4 MHz - 16 MHz))
#endif

#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = OFF      // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

/*
 * Global Variable Declarations
 */

volatile unsigned int millis = 0; // Holds timer0 rollover count.

volatile signed int rpm = 0;
volatile signed int oilPressure = 0;
volatile signed int oilTemp = 0;
volatile signed int engineTemp = 0;
volatile signed int batteryVoltage = 0;

typedef enum {
    true, false
};
#define true 1;
#define false 0;
typedef char bool;
bool errorVolts = 0, errorRPM = 0, errorOilTemp = 0, errorEngineTemp = 0, changedErrorState;

// ECAN variables
unsigned long id; // holds CAN msgID
BYTE data[8]; // holds CAN data bytes
BYTE dataLen; // holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags; // holds information about recieved message

/*
 * Interrupts
 */

/*
 * Description: Function to service interrupts.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: Transmits messages on CAN.
 *     Reloads the timer0 registers and increments millis.
 *     Resets the interrupt flags after servicing them.
 */
#pragma code high_vector = 0x08

void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr

void high_isr(void) {

    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(0x85); // Load timer registers (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // Check for recieved CAN message.
    if(PIR5bits.RXB1IF) {
        PIR5bits.RXB1IF = FALSE; // Reset the flag.

        // Get data from recieve buffer.
        ECANReceiveMessage(&id, data, &dataLen, &flags);

        if(id == RPM_ID) {
            ((BYTE*) & rpm)[0] = data[RPM_BYTE + 1];
            ((BYTE*) & rpm)[1] = data[RPM_BYTE];
        }
        if(id == ENGINE_TEMP_ID) {
            ((BYTE*) & engineTemp)[0] = data[ENGINE_TEMP_BYTE + 1];
            ((BYTE*) & engineTemp)[1] = data[ENGINE_TEMP_BYTE];
        }
        if(id == OIL_TEMP_ID) {
            ((BYTE*) & oilTemp)[0] = data[OIL_TEMP_BYTE + 1];
            ((BYTE*) & oilTemp)[1] = data[OIL_TEMP_BYTE];
        }
        if(id == OIL_PRESS_ID) {
            ((BYTE*) & oilPressure)[0] = data[OIL_PRESS_BYTE + 1];
            ((BYTE*) & oilPressure)[1] = data[OIL_PRESS_BYTE];
        }
        if(id == VOLTAGE_ID) {
            ((BYTE*) & batteryVoltage)[0] = data[VOLTAGE_BYTE + 1];
            ((BYTE*) & batteryVoltage)[1] = data[VOLTAGE_BYTE];
        }
    }
}

/*
 * Functions
 */

/*
 * Description: Sets specified LED to specified color.
 * Input(s): led - The index of the led to change.
 *     color - The color value to change the LED to. Color values come from the
 *             header file.
 * Return Values(s): none
 * Side Effects: Will modify various LAT bits.
 */
void set_led_to_color(unsigned char led, unsigned char color) {
    if(led == 0) {
        RED0_LAT = color & RED ? 0 : 1;
        GREEN0_LAT = color & GREEN ? 0 : 1;
        BLUE0_LAT = color & BLUE ? 0 : 1;
    } else if(led == 1) {
        RED1_LAT = color & RED ? 0 : 1;
        GREEN1_LAT = color & GREEN ? 0 : 1;
        BLUE1_LAT = color & BLUE ? 0 : 1;
    } else if(led == 2) {
        RED2_LAT = color & RED ? 0 : 1;
        GREEN2_LAT = color & GREEN ? 0 : 1;
        BLUE2_LAT = color & BLUE ? 0 : 1;
    } else if(led == 3) {
        RED3_LAT = color & RED ? 0 : 1;
        GREEN3_LAT = color & GREEN ? 0 : 1;
        BLUE3_LAT = color & BLUE ? 0 : 1;
    } else if(led == 4) {
        RED4_LAT = color & RED ? 0 : 1;
        GREEN4_LAT = color & GREEN ? 0 : 1;
        BLUE4_LAT = color & BLUE ? 0 : 1;
    }
}

/*
 * Description: Sets all LEDs to specified color.
 * Input(s): color - The color value to change the LED to. Color values come
 *                   from the header file.
 * Return Value(s): none
 * Side Effects: none
 */
void set_all(unsigned char color) {
    unsigned char i = 0;
    for(; i < 5; i++) {
        set_led_to_color(i, color);
    }
}

/*
 * Description: Sets lights to NONE or REV_COLOR based on parameter.
 * Input(s): max - The highest number LED to illuminate.
 * Return Value(s): none
 * Side Effects: none
 */
void set_lights(unsigned char max) {
    unsigned char i = 0;
    for(; i < 5; i++) {
        if(max > i) {
            set_led_to_color(i, REV_COLOR);
        } else {
            set_led_to_color(i, NONE);
        }
    }
}

//typedef enum {Dtrue, false} _Bool;

/*
 Function checks the volts from the battery.
 Outputs: True (1) if volts are in balance, above the threshold.
          False(0) if volts are below the threshold.
 */
void checkVolts() {
    int level = 13; //get the value here.
    if(level < 13) {
        errorVolts = 1; //Battery is low, error flag set to true
    }
    //return 0;
}

void excess(int type) {
    switch(type) {
        case 0:
            //excess oil temp
            errorOilTemp = 0;
            break;
        case 1:
            //excess engine temp
            errorEngineTemp = 0;
            break;
        default:
            //code error
            break;

    }
}

void display() {
    if(changedErrorState) {
        if(errorRPM && (!errorOilTemp && !errorEngineTemp && !errorVolts)) {
            //Only RPM Error
        } else if(errorOilTemp && (!errorRPM && !errorEngineTemp && !errorVolts)) {
            //only Oil Temp error
        } else if(errorEngineTemp && (!errorOilTemp && !errorRPM && !errorVolts)) {
            //only Engine Temp
        } else if(errorVolts && (!errorOilTemp && !errorEngineTemp && !errorRPM)) {
            //only Battery error
        } else {
            //Multiple Error Singles
        }
    } else {
        //Normal RPM display here
    }
}

/*
 * Main Loop
 */

void main(void) {

    /*
     * Variable Declarations
     */

    unsigned int blink_tmr = 0;

    /*
     * Oscillator Setup
     */

    init_oscillator();

    /*
     * Peripherals Setup
     */

    ANCON0 = 0x00; // Default all pins to digital
    ANCON1 = 0x00; // Default all pins to digital

    // Turn on and configure the TIMER1 oscillator
    OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
    WriteTimer0(0x82); // Load timer register
    millis = 0; // Clear milliseconds count
    INTCONbits.TMR0IE = 1; // Turn on timer0 interupts

    TRISCbits.TRISC6 = OUTPUT; // programmable termination
    TERM_LAT = FALSE;

    ECANInitialize(); // setup ECAN

    // interrupts setup
    INTCONbits.GIE = 1; // Global Interrupt Enable (1 enables)
    INTCONbits.PEIE = 1; // Peripheral Interrupt Enable (1 enables)
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)

    init_unused_pins();

    // Set LED pins as outputs.
    RED0_TRIS = OUTPUT;
    GREEN0_TRIS = OUTPUT;
    BLUE0_TRIS = OUTPUT;
    RED1_TRIS = OUTPUT;
    GREEN1_TRIS = OUTPUT;
    BLUE1_TRIS = OUTPUT;
    RED2_TRIS = OUTPUT;
    GREEN2_TRIS = OUTPUT;
    BLUE2_TRIS = OUTPUT;
    RED3_TRIS = OUTPUT;
    GREEN3_TRIS = OUTPUT;
    BLUE3_TRIS = OUTPUT;
    RED4_TRIS = OUTPUT;
    GREEN4_TRIS = OUTPUT;
    BLUE4_TRIS = OUTPUT;

    // Default all LEDs to off.
    RED0_LAT = 1;
    GREEN0_LAT = 1;
    BLUE0_LAT = 1;
    RED1_LAT = 1;
    GREEN1_LAT = 1;
    BLUE1_LAT = 1;
    RED2_LAT = 1;
    GREEN2_LAT = 1;
    BLUE2_LAT = 1;
    RED3_LAT = 1;
    GREEN3_LAT = 1;
    BLUE3_LAT = 1;
    RED4_LAT = 1;
    GREEN4_LAT = 1;
    BLUE4_LAT = 1;

    /* Startup - Show full RED lights for a while before doing anything else.
    blink_tmr = millis;
    while(1) {
      if(millis - blink_tmr < BLINK_TIME)
        set_all(RED);
      else if(millis - blink_tmr < BLINK_TIME * 2)
        set_all(RGB);
      else if(millis - blink_tmr < BLINK_TIME * 3)
        set_all(BLUE);
      else if(millis - blink_tmr < BLINK_TIME * 4)
        set_all(RED);
      else if(millis - blink_tmr < BLINK_TIME * 5)
        set_all(RGB);
      else if(millis - blink_tmr < BLINK_TIME * 6)
        set_all(BLUE);
      else if(millis - blink_tmr < BLINK_TIME * 7)
        set_all(RED);
      else if(millis - blink_tmr < BLINK_TIME * 8)
        set_all(RGB);
      else if(millis - blink_tmr < BLINK_TIME * 9)
        set_all(BLUE);
      else
        break;
    }*/

    blink_tmr = millis;
    while(1) {

        // Sets certain lights to NONE or REV_COLOR based on rpm value.
        if(rpm >= REV_RANGE_LIMIT) {
            if(millis - blink_tmr < BLINK_TIME) {
                set_all(REV_LIMIT_COLOR);
            } else if(millis - blink_tmr < BLINK_TIME * 2) {
                set_all(NONE);
            } else {
                blink_tmr = millis;
            }
        } else if(rpm >= REV_RANGE_5) {
            set_lights(5);
        } else if(rpm >= REV_RANGE_4) {
            set_lights(4);
        } else if(rpm >= REV_RANGE_3) {
            set_lights(3);
        } else if(rpm >= REV_RANGE_2) {
            set_lights(2);
        } else if(rpm >= REV_RANGE_1) {
            set_lights(1);
        } else {
            set_all(NONE);
        }
    }
}
