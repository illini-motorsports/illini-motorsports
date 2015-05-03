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
#include <stdio.h>
#include "timers.h"
#include "ECAN.h"
#include "FSAE.h"
#include "CAN.h"
#include "ShiftLights.h"
#include "delays.h"

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

volatile unsigned long millis = 0; // Holds timer0 rollover count.

/*
 * MAIN INFORMATION:
 */
volatile signed int rpm = 0;
volatile signed int oilTemp = 0;
volatile signed int engineTemp = 0;
volatile unsigned long lastInterrupt = 0; //variable used to give last time interrupt occured for the main information.
volatile unsigned long rpmLastAccess = 0, oilLastAccess = 0,
                        engineLastAccess = 0;
volatile unsigned long recieveMsgInterval = 500; //milliseconds
volatile unsigned long capturedTimeForBlink = 0;
volatile unsigned long motecErrTime = 0;
volatile unsigned char white_blink_et = 0, all_white_et = 0, 
                        white_blink_ot = 0, all_white_ot = 0;
volatile signed long long can_grabbed_time;
volatile signed long long motec_grabbed_time;
char timeout[2] = {0, 0};


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
        lastInterrupt = millis;

        PIR5bits.RXB1IF = FALSE; // Reset the flag.

        // Get data from recieve buffer.
        ECANReceiveMessage(&id, data, &dataLen, &flags);

        if(id == RPM_ID) {
            //lastRPMAccess
            rpmLastAccess = millis;
            ((BYTE*) & rpm)[0] = data[RPM_BYTE + 1];
            ((BYTE*) & rpm)[1] = data[RPM_BYTE];
        }
        if(id == ENGINE_TEMP_ID) {
            //lastEngineTempAccess
            engineLastAccess = millis;
            ((BYTE*) & engineTemp)[0] = data[ENGINE_TEMP_BYTE + 1];
            ((BYTE*) & engineTemp)[1] = data[ENGINE_TEMP_BYTE];
        }
        if(id == OIL_TEMP_ID) {
            //lastOilTempAccess;
            oilLastAccess = millis;
            ((BYTE*) & oilTemp)[0] = data[OIL_TEMP_BYTE + 1];
            ((BYTE*) & oilTemp)[1] = data[OIL_TEMP_BYTE];
        }

        //Configuation of status flags
        if(engineTemp <= ENGINE_TEMP_LOW){
            white_blink_et = 0;
            all_white_et = 0;
        }else if(engineTemp <= ENGINE_TEMP_MED && engineTemp > ENGINE_TEMP_LOW)
            all_white_et = 0;
        else if(engineTemp <= ENGINE_TEMP_HIGH && engineTemp > ENGINE_TEMP_MED)
            white_blink_et = 1;
        else if(engineTemp > ENGINE_TEMP_HIGH)
            all_white_et = 1;

        if(oilTemp <= OIL_TEMP_LOW){
            white_blink_ot = 0;
            all_white_ot = 0;
        }else if(oilTemp <= OIL_TEMP_MED && oilTemp > OIL_TEMP_LOW)
            all_white_ot = 0;
        else if(oilTemp <= OIL_TEMP_HIGH && oilTemp > OIL_TEMP_MED)
            white_blink_ot = 1;
        else if(oilTemp > OIL_TEMP_HIGH)
            all_white_ot = 1;
    }
}
/*
 * Description: Sets specified LED to specified color.
 * Input(s): led - The index of the led to change.
 *     color - The color value to change the LED to. Color values come from the
 *             header file.
 * Return Values(s): none
 * Side Effects: Will modify various LAT bits.
 */
void set_led_to_color(unsigned char led, unsigned char color) {
    switch(led){
        case 0:
            RED0_LAT = color & RED ? 0 : 1;
            GREEN0_LAT = color & GREEN ? 0 : 1;
            BLUE0_LAT = color & BLUE ? 0 : 1;
            break;
        case 1:
            RED1_LAT = color & RED ? 0 : 1;
            GREEN1_LAT = color & GREEN ? 0 : 1;
            BLUE1_LAT = color & BLUE ? 0 : 1;
            break;
        case 2:
            RED2_LAT = color & RED ? 0 : 1;
            GREEN2_LAT = color & GREEN ? 0 : 1;
            BLUE2_LAT = color & BLUE ? 0 : 1;
            break;
        case 3:
            RED3_LAT = color & RED ? 0 : 1;
            GREEN3_LAT = color & GREEN ? 0 : 1;
            BLUE3_LAT = color & BLUE ? 0 : 1;
            break;
        case 4:
            RED4_LAT = color & RED ? 0 : 1;
            GREEN4_LAT = color & GREEN ? 0 : 1;
            BLUE4_LAT = color & BLUE ? 0 : 1;
            break;
        default:
            break;
    }
}

void set_lights(unsigned char max, unsigned char color) {
    unsigned char i = 0;
    for(; i < 5; i++) {
        if(i < max) {
            set_led_to_color(i, color);
        } else {
            set_led_to_color(i, NONE);
        }
    }
}
/* FUNCTION:
 *
 */
void RPMDisplayer(){
    int short_shift_active = (white_blink_et || white_blink_ot || all_white_et || all_white_ot) ?  SHORT_SHIFT_OFFSET : 0;
    if(rpm >= (REV_RANGE_LIMIT - short_shift_active)) {
        BLINKDisplayer(short_shift_active ? ERRBLINKCOLOR : STDBLINKCOLOR);
    }else if(rpm >= (REV_RANGE_5 - short_shift_active)) {
        set_lights(5, (all_white_et || all_white_ot) ? ERRREVCOLOR : STDREVCOLOR);
    }else if(rpm >= (REV_RANGE_4 - short_shift_active)) {
        set_lights(4, (all_white_et || all_white_ot) ? ERRREVCOLOR : STDREVCOLOR);
    }else if(rpm >= (REV_RANGE_3 - short_shift_active)) {
        set_lights(3, (all_white_et || all_white_ot) ? ERRREVCOLOR : STDREVCOLOR);
    }else if(rpm >= (REV_RANGE_2 - short_shift_active)) {
        set_lights(2, (all_white_et || all_white_ot) ? ERRREVCOLOR : STDREVCOLOR);
    }else if(rpm >= (REV_RANGE_1 - short_shift_active)) {
        set_lights(1, (all_white_et || all_white_ot) ? ERRREVCOLOR : STDREVCOLOR);
    }else{
        set_lights(0,NONE);
    }
}
void BLINKDisplayer(char color){
    if(capturedTimeForBlink == 0 || millis - capturedTimeForBlink > BLINK_TIME * 2)
        capturedTimeForBlink = millis;
    if(millis - capturedTimeForBlink < BLINK_TIME){
        set_lights(5, color);
    }else if(millis - capturedTimeForBlink > BLINK_TIME){
        set_lights(0,NONE);
    }
}

/* FUNCTION: Gives a pretty startup animation. Just makin' things look pretty here.
 *
 * ***\KEEP THIS FUNCTION.\***
 */
void startup(long currentTime){
    while(true){
        //Moving in towards the center then back out and final blinks twice
        if(millis - currentTime < BLINK_TIME){
            //turn two outer lights to red
            set_led_to_color(0, RED);
            set_led_to_color(4, RED);
        }else if(millis - currentTime < BLINK_TIME * 2){
            //turn next outer two to red
            set_led_to_color(1, RED);
            set_led_to_color(3, RED);
        }else if(millis - currentTime < BLINK_TIME * 3){
            //turn middle light to red
            set_led_to_color(2,RED);
        }else if(millis - currentTime < BLINK_TIME * 4){
            //turn middle to green light
            set_led_to_color(2,GREEN);
        }else if(millis - currentTime < BLINK_TIME * 5){
            //turn next outer two green light
            set_led_to_color(1, GREEN);
            set_led_to_color(3, GREEN);
        }else if(millis - currentTime < BLINK_TIME * 6){
            //turn most outer two to green light
            set_led_to_color(0, GREEN);
            set_led_to_color(4, GREEN);
        }else if(millis - currentTime < BLINK_TIME * 9){
            //Blink twice
            BLINKDisplayer(GREEN);
        }else{
            set_lights(0,NONE);
            break;
        }
    }
}
/*
 * Main Loop
 */
void main(void) {

        // Variable Declarations
        unsigned int blink_tmr = 0;

        //Oscillator Setup
        init_oscillator();

        //Peripherals Setup
        ANCON0 = 0x00;                      // Default all pins to digital
        ANCON1 = 0x00;                      // Default all pins to digital

        // Turn on and configure the TIMER1 oscillator
        OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
        WriteTimer0(0x82);                  // Load timer register
        millis = 0;                         // Clear milliseconds count
        INTCONbits.TMR0IE = 1;              // Turn on timer0 interupts

        TRISCbits.TRISC6 = OUTPUT;          // programmable termination
        TERM_LAT = FALSE;

        ECANInitialize();                   //setup ECAN

        // interrupts setup
        INTCONbits.GIE = 1;                 // Global Interrupt Enable (1 enables)
        INTCONbits.PEIE = 1;                // Peripheral Interrupt Enable (1 enables)
        RCONbits.IPEN = 0;                  // Interrupt Priority Enable (1 enables)

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

        //Begin startup animation and RPM display
        startup(millis); //nice startup animation
        while(true){
            //simulateDataPush();
            if(motecError() || canError()){
                BLINKDisplayer(ERRBLINKCOLOR);
            }else{
                RPMDisplayer();
            }
        }
}

bool canError(){
    //everything is all good, no can error
    if(abs((signed long long)millis - (signed long long)lastInterrupt) <= CAN_RECIEVE_MAX && !COMSTATbits.TXBO){
        timeout[CAN_ERR] = 0;
    }else if(timeout[CAN_ERR]){
        if(abs((signed long long)millis - can_grabbed_time) >= CAN_RECIEVE_MAX)
            return true;
    }else{
        timeout[CAN_ERR] = 1;
        can_grabbed_time = (signed long long)millis;
    }
    return false;
}

bool motecError(){
    if(abs((signed long long)millis - (signed long long)rpmLastAccess) <= recieveMsgInterval &&
       abs((signed long long)millis - (signed long long)engineLastAccess) <= recieveMsgInterval &&
       abs((signed long long)millis - (signed long long)oilLastAccess) <= recieveMsgInterval){
        //Entering here means you are recieving oil temp, rpm, and engine temp in a timely manner
        timeout[MOTEC_ERR] = 0;
    }else if(timeout[MOTEC_ERR]){
        if(abs((signed long long)millis - motec_grabbed_time) >= MOTEC_RECIEVE_MAX)
            return true;
    }else{
        timeout[MOTEC_ERR] = 1;
        motec_grabbed_time = (signed long long)millis;
    }
    return false;
}

char climb = 1;
void simulateDataPush(){
    Delay10KTCYx(1);
    millis++;

    lastInterrupt = millis;
    rpmLastAccess = millis;
    engineLastAccess = millis;
    oilLastAccess = millis;

    engineTemp = 50;
    oilTemp = 50;

    if(rpm == 0){
        climb = 1;
    }
    
    if(climb)
        rpm += 5;
    else
        rpm -= 5;

    if(rpm > REV_RANGE_LIMIT + (REV_RANGE_LIMIT >> 2))
        climb = 0;

    //Configuation of status flags
    /*if(engineTemp <= 1000){
        white_blink_et = 0;
        all_white_et = 0;
    }else if(engineTemp <= 1050 && engineTemp > 1000)
        all_white_et = 0;
     else if(engineTemp <= 1150 && engineTemp > 1050)
        white_blink_et = 1;
     else if(engineTemp > 1150)
        all_white_et = 1;

     if(oilTemp <= 1600){
        white_blink_ot = 0;
        all_white_ot = 0;
     }else if(oilTemp <= 1700 && oilTemp > 1600)
        all_white_ot = 0;
      else if(oilTemp <= 1800 && oilTemp > 1700)
        white_blink_ot = 1;
      else if(oilTemp > 1800)
        all_white_ot = 1;*/

    //Configuation of status flags
    if(engineTemp <= ENGINE_TEMP_LOW){
        white_blink_et = 0;
        all_white_et = 0;
    }else if(engineTemp <= ENGINE_TEMP_MED && engineTemp > ENGINE_TEMP_LOW)
        all_white_et = 0;
     else if(engineTemp <= ENGINE_TEMP_HIGH && engineTemp > ENGINE_TEMP_MED)
        white_blink_et = 1;
     else if(engineTemp > ENGINE_TEMP_HIGH)
        all_white_et = 1;

    if(oilTemp <= OIL_TEMP_LOW){
        white_blink_ot = 0;
        all_white_ot = 0;
    }else if(oilTemp <= OIL_TEMP_MED && oilTemp > OIL_TEMP_LOW)
        all_white_ot = 0;
     else if(oilTemp <= OIL_TEMP_HIGH && oilTemp > OIL_TEMP_MED)
        white_blink_ot = 1;
     else if(oilTemp > OIL_TEMP_HIGH)
        all_white_ot = 1;
}
void arrayOfColors(){
    //Not for formal use. just viewing.
    int x = millis;
    set_lights(5, 0b001);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b010);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b011);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b100);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b101);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b110);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
    set_lights(5, 0b111);
    while(millis - x <= BLINK_TIME * 10){}
    x = millis;
}