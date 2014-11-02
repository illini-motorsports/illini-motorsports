/*
 * Wheel Node Main File
 *
 * File Name:       Wheel.c
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18
 * Version:         1.00
 * Author:          George Schwieters
 * Created:         2013-2014
 */

#include "Wheel.h"
#include "ECAN.h"
#include "FSAE.h"
#include "spi.h"

/*
 *  PIC18F46K80 Configuration Bits
 */

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#ifdef INTERNAL
    #pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#else
    #pragma config FOSC = HS1       // Oscillator (HS oscillator (Medium power, 4 MHz - 16 MHz))
#endif

#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2L
#pragma config PWRTEN = OFF      // Power Up Timer (Disabled)
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
 * Global Variables
 */

static volatile int millis; // holds timer0 rollover count

// timing
static unsigned int refreshTime[2], blinkTimer[2], holdTimer[2];
static unsigned char blinkStates[2], holdText[2], displayStates[2];
static unsigned int CANint_tmr;

// ECAN variables
static unsigned long id; // holds CAN msgID
static unsigned char data[8]; // holds CAN data bytes
static unsigned char dataLen; // holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags; // holds information about recieved message

static volatile int chan[NUM_CHAN];
static volatile int gear;
static int CANerror_flag;

static const unsigned char d_place_arr[NUM_CHAN] = {
    2, // oil temperature
    2, // engine temperature
    2, // battery voltage
    2, // oil pressure
    2, // ground speed
    0 // engine RPM
};
static const unsigned char num_arr[12] = {
    NUM_0, NUM_1, NUM_2, NUM_3, NUM_4,
    NUM_5, NUM_6, NUM_7, NUM_8, NUM_9,
    BLANK, CHAR_N
};
static const unsigned char text_arr[NUM_CHAN + 2][3] = {
    {BLANK, CHAR_O, CHAR_t}, // oil temperature
    {BLANK, CHAR_E, CHAR_t}, // engine temmperature
    {CHAR_b, CHAR_A, CHAR_t}, // battery voltage
    {BLANK, CHAR_O, CHAR_P}, // oil pressure
    {CHAR_S, CHAR_P, CHAR_d}, // ground speed
    {CHAR_t, CHAR_A, CHAR_c}, // engine RPM
    {CHAR_E, CHAR_r, CHAR_r}, // CAN error
    {CHAR_C, CHAR_A, CHAR_N}
}; // CAN error

/*
 * Interrupts
 */

/*
 *  void high(void)
 *
 *  Description: This interrupt will service all low priority interrupts.
 *  Input(s): none
 *  Reaturn Value(s): none
 *  Side Effects: This will modify INTCON0 & PIR5. Also it modify the ECAN variables.
 */
#pragma code high_vector = 0x08

void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr

void high_isr(void) {

    // check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }
    // check for recieved CAN message
    if(PIR5bits.RXB1IF) {
        PIR5bits.RXB1IF = FALSE; // reset the flag
        CANint_tmr = millis; //"reset" interrupt timer
        CANerror_flag = 0; //reset flag
        // get data from recieve buffer
        //ECANReceiveMessage(&id, data, &dataLen, &flags);
        ECANReceiveMessage(&id, data, &dataLen, &flags);


        bufferData(); // put data in an array
    }

    return;
}

void main(void) {

    /*
     * Variable Declarations
     */

    unsigned char radio_sw[2], drs_over_sw[2], fan_over_sw[2], fuel_map_sw[2],
            paddle_l_sw[2], paddle_r_sw[2];
    unsigned char ADLmsg[8];
    unsigned char cycleStates[2], intensity;
    unsigned int bounceTimer[2];
    unsigned int CAN_tmr;
    unsigned long CANTRANSMISSIONRATE = 20000;

    /*
     * Variable Initialization
     */

    // intialize states
    cycleStates[LEFT] = CYCLE_L;
    cycleStates[RIGHT] = CYCLE_R;
    holdText[LEFT] = holdText[RIGHT] = TRUE;
    refreshTime[LEFT] = refreshTime[RIGHT] = holdTimer[LEFT] = holdTimer[RIGHT] =
            blinkTimer[LEFT] = blinkTimer[RIGHT] = bounceTimer[LEFT] =
            bounceTimer[RIGHT] = millis;
    CANint_tmr = millis;
    CANerror_flag = 0;

    displayStates[LEFT] = OIL_T;
    displayStates[RIGHT] = ENGINE_T;

    /*
     * Peripheral Initialization
     */

    init_ADC();

    // configure the A/D converter module
    ANCON0 = 0b00100111; // AN0 - 2 and AN5 are analog
    ANCON1 = 0x00; // rest are digital
    TRISAbits.TRISA0 = INPUT;
    TRISAbits.TRISA1 = INPUT;
    TRISAbits.TRISA2 = INPUT;
    TRISAbits.TRISA5 = INPUT;

    init_timer0();

    // SPI setup
    SSPSTATbits.CKE = 1; // SPI Clock Select, 1 = transmit on active to idle
    SSPCON1bits.CKP = 0; // Clock Polarity Select, 0 = low level is idle state
    SSPCON1bits.SSPM = 0b1010; // Clk Frequecy (Note: FOSC = 64MHz)
    SSPCON1bits.SSPEN = 1; // SPI Enable, 1 enables

    // SPI pin I/O setup
    TRISCbits.TRISC3 = OUTPUT; // SCK
    TRISCbits.TRISC5 = OUTPUT; // SDO
    TRISDbits.TRISD3 = OUTPUT; // CS
    CS = 1;

    // driver set up
    intensity = 0x0F;
    driver_write(DISP_MODE, NORMAL); // leave test mode
    driver_write(SHUTDOWN, SHUTDOWN_OFF); // leave shutdown mode
    driver_write(INTENSITY, intensity); // set brightness to highest
    driver_write(SCAN, FULL_SCAN); // Set scan to all digits
    driver_write(DECODE, NO_DECODE); // Decoding disabled

    // set displays to display zero
    write_gear(0);
    write_num(0, 2, LEFT);
    write_num(0, 2, RIGHT);

    ECANInitialize(); // setup ECAN

    // interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    TRISCbits.TRISC6 = OUTPUT; // programmable termination
    TERM_LAT = FALSE;

    while(1) {

        //Check that can is sending messages
        if(millis - CANint_tmr > CANTRANSMISSIONRATE) //CANTRANSMISSIONRATE is a placeholder for the expected CAN data update rate.
        {
            write_CANerror();
            CANerror_flag = 1;
        }

        // check for change in button state
        if(cycleStates[LEFT] != CYCLE_L & millis - bounceTimer[LEFT] > BOUNCE_TIME) {
            // save new state
            cycleStates[LEFT] = CYCLE_L;
            bounceTimer[LEFT] = millis;
            // only change display if button is low
            if(!cycleStates[LEFT]) {
                if(++displayStates[LEFT] == NUM_CHAN)
                    displayStates[LEFT] = 0;
                // put the appropriate text on the displays and
                // get the current time for timing logic
                if(CANerror_flag != 1);
                updateText(LEFT, displayStates);
                holdText[LEFT] = TRUE;
                blinkTimer[LEFT] = holdTimer[LEFT] = millis;
            }
        }
        if(cycleStates[RIGHT] != CYCLE_R & millis - bounceTimer[RIGHT] > BOUNCE_TIME) {
            cycleStates[RIGHT] = CYCLE_R;
            bounceTimer[RIGHT] = millis;
            if(!cycleStates[RIGHT]) {
                if(++displayStates[RIGHT] == NUM_CHAN)
                    displayStates[RIGHT] = 0;
                if(CANerror_flag != 1)
                    updateText(RIGHT, displayStates);
                holdText[RIGHT] = TRUE;
                blinkTimer[RIGHT] = holdTimer[RIGHT] = millis;
            }
        }

        // update left and right displays with text or numerical data
        if(CANerror_flag != 1) {
            updateDisp(LEFT);
            updateDisp(RIGHT);
        }
        write_gear(gear);

        // radio button
        if(!RADIO) {
            radio_sw[0] = 0x13;
            radio_sw[1] = 0x88;
        } else
            *(int *) radio_sw = 0;
#if 0
        // paddle switches
        if(PADDLE_L) {
            paddle_l_sw[0] = 0x13;
            paddle_l_sw[1] = 0x88;
        } else
            *(int *) paddle_l_sw = 0;
        if(PADDLE_R) {
            paddle_r_sw[0] = 0x13;
            paddle_r_sw[1] = 0x88;
        } else
            *(int *) paddle_r_sw = 0;
#endif
        // DRS override switch
        if(DRS_OVER) {
            drs_over_sw[0] = 0x13;
            drs_over_sw[1] = 0x88;
        } else
            *(int *) drs_over_sw = 0;
        // fan override switch
        if(FAN_OVER) {
            fan_over_sw[0] = 0x13;
            fan_over_sw[1] = 0x88;
        } else
            *(int *) fan_over_sw = 0;
        // fuel map switch
        if(FUEL_MAP) {
            fuel_map_sw[0] = 0x13;
            fuel_map_sw[1] = 0x88;
        } else
            *(int *) fuel_map_sw = 0;

        if(millis - CAN_tmr > CAN_PER) {
            CAN_tmr = millis;
            // send out the first three sampled switches
            ADLmsg[0] = 0x00;
            ADLmsg[1] = 0x00;
            ADLmsg[ADL1_BYTE] = radio_sw[0];
            ADLmsg[ADL1_BYTE + 1] = radio_sw[1];
            ADLmsg[ADL2_BYTE] = fan_over_sw[0];
            ADLmsg[ADL2_BYTE + 1] = fan_over_sw[1];
            ADLmsg[ADL3_BYTE] = fuel_map_sw[0];
            ADLmsg[ADL3_BYTE + 1] = fuel_map_sw[1];
            ECANSendMessage(ADL_ID, ADLmsg, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);

            // send out first three rotary encoders
            ADLmsg[0] = 0x01;
            ADLmsg[1] = 0x00;
            ADLsample(ADLmsg, ADL4_BYTE, LAUNCH_ROT);
            ADLsample(ADLmsg, ADL5_BYTE, TRAC_ROT);
            ADLsample(ADLmsg, ADL6_BYTE, DRS_ROT);
            ECANSendMessage(ADL_ID, ADLmsg, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
        }
    } // end main loop

    return;
}

/*
 *  Local Functions
 */

/*
 *  void driver_write(void)
 *
 *  Description:    This function sends out data to a specified address to the
 *                  MAXIM LED driver.
 *  Input(s):   addr - LED driver address to transmit
 *              data - LED driver data value to transmit
 *  Return Value(s): none
 *  Side Effects: none
 */
void driver_write(unsigned char addr, unsigned char data) {

    // first select the driver
    CS = 0;
    // send out address byte then the data byte
    WriteSPI(addr);
    WriteSPI(data);
    // deselect the device
    CS = 1;

    return;
}

/*
 *  void write_gear(void)
 *
 *  Description:    This function writes a gear value to the wheel's middle
 *                  seven segment display.
 *  Input(s): gear - numerical value of gear to display
 *  Return Value(s): none
 *  Side Effects: none
 */
void write_gear(unsigned char gear) {

    // convert gear value to data to be sent to display driver
    if(gear != 7)
        gear = num_arr[gear];
    else
        gear = num_arr[11];

    // write the gear position
    driver_write(DIG6, gear);

    return;
}

/*
 *  void write_CANerror(void)
 *
 *  Description:
 *  Input(s): none
 *  Return Value(s): none
 *  Side Effects: none
 */
void write_CANerror(void) {

    // force CANERR messaged on displays
    driver_write(DIG2, text_arr[CANERR1][0]);
    driver_write(DIG1, text_arr[CANERR1][1]);
    driver_write(DIG0, text_arr[CANERR1][2]);
    driver_write(DIG3, text_arr[CANERR2][0]);
    driver_write(DIG4, text_arr[CANERR2][1]);
    driver_write(DIG5, text_arr[CANERR2][2]);
    ///////////////////////
    //code for LED indicator
    //
    //
    //
    ///////////////////////
    return;
}

/*
 *  void blank_display(unsigned char side)
 *
 *  Description: This will clear the displays.
 *  Input(s): side - specifies which three seven segment displays to clear
 *  Return Value(s): none
 *  Side Effects: none
 */
void blank_display(unsigned char side) {

    int i;

    // check which side to blank
    if(side == BOTH) {
        // loop through digits and blank them
        for(i = 1; i < 8; i++) {
            driver_write(i, BLANK);
        }
    } else if(side == LEFT) {
        for(i = 1; i < 4; i++) {
            driver_write(i, BLANK);
        }
    }        // right side
    else {
        for(i = 4; i < 7; i++) {
            driver_write(i, BLANK);
        }
    }

    return;
}

/*
 *  void write_num(int data, unsigned char d_place, unsigned char side)
 *
 *  Description: This will write a number to a set of displays.
 *  Input(s):   d_place - where to place the decimal place
 *              side - which side to write string of numbers
 *              data - numerical value to write to the displays
 *  Return Value(s): none
 *  Side Effects: none
 */
void write_num(int data, unsigned char d_place, unsigned char side) {

    unsigned char num_0, num_1, num_2;

    // get individual digits of full number
    num_2 = data % 10;
    num_1 = (data % 100) / 10;
    num_0 = (data % 1000) / 100;

    // convert values to data bytes for display driver
    num_0 = num_arr[num_0];
    num_1 = num_arr[num_1];
    num_2 = num_arr[num_2];

    // add decimal points to numbers
    if(d_place) {
        if(d_place == 1) {
            num_0 |= NUM_DP;
        } else if(d_place == 2) {
            num_1 |= NUM_DP;
        } else {
            num_2 |= NUM_DP;
        }
    }

    // write the number to the indicated side
    if(!side) {
        driver_write(DIG2, num_0);
        driver_write(DIG1, num_1);
        driver_write(DIG0, num_2);
    } else {
        driver_write(DIG3, num_0);
        driver_write(DIG4, num_1);
        driver_write(DIG5, num_2);
    }

    return;
}

/*
 *  void updateText(unsigned char side, unsigned char *state)
 *
 *  Description: This will send text to a set of displays.
 *  Input(s):   side - which side to put the text
 *              state - which type of data we are displaying
 *  Return Value(s): none
 *  Side Effects: none
 */
void updateText(unsigned char side, unsigned char *state) {

    // update left or right display with text
    if(!side) {
        // send out characters one at a time
        driver_write(DIG2, text_arr[state[side]][0]);
        driver_write(DIG1, text_arr[state[side]][1]);
        driver_write(DIG0, text_arr[state[side]][2]);
    } else {
        driver_write(DIG3, text_arr[state[side]][0]);
        driver_write(DIG4, text_arr[state[side]][1]);
        driver_write(DIG5, text_arr[state[side]][2]);
    }

    return;
}

/*
 *  void updateDisp(unsigned char side)
 *
 *  Description:    This will determine what should be displayed on the displays
 *                  based on timers.
 *  Input(s): side - which side to update
 *  Return Value(s): none
 *  Side Effects: none
 */
void updateDisp(unsigned char side) {

    // check if we are holding text
    if(holdText[side]) {
        // check if the hold time has passed
        if(millis - holdTimer[side] > HOLD_TIME) {
            holdText[side] = FALSE;
        }            // carry out blinking of text
        else if(millis - blinkTimer[side] > BLINK_TIME) {
            // redisplay the text
            if(blinkStates[side]) {
                blinkStates[side] = FALSE;
                updateText(side, displayStates);
                blinkTimer[side] = millis;
            }                // blank the displays
            else {
                blinkStates[side] = TRUE;
                blank_display(side);
                blinkTimer[side] = millis;
            }
        }
    }        // data is being displayed
    else {
        if(millis - refreshTime[side] > REFRESH_TIME) {
            refreshTime[side] = millis;
            write_num(chan[displayStates[side]], d_place_arr[displayStates[side]], side);
        }
    }

    return;
}

/*
 *  void updateDisp(unsigned char side)
 *
 *  Description:    This will determine what should be displayed on the displays
 *                  based on timers.
 *  Input(s): none
 *  Return Value(s): none
 *  Side Effects: The chan and gear variables will be modified.
 */
void bufferData(void) {

    if(id == MOTEC_ID) {
        ((unsigned char*) &(chan[RPM]))[0] = data[RPM_BYTE + 1];
        ((unsigned char*) &(chan[RPM]))[1] = data[RPM_BYTE];
        ((unsigned char*) &(chan[OIL_P]))[0] = data[OIL_PRESS_BYTE + 1];
        ((unsigned char*) &(chan[OIL_P]))[1] = data[OIL_PRESS_BYTE];
        ((unsigned char*) &(chan[OIL_T]))[0] = data[OIL_TEMP_BYTE + 1];
        ((unsigned char*) &(chan[OIL_T]))[1] = data[OIL_TEMP_BYTE];
        chan[RPM] = chan[RPM] / 100;
    } else if(id == MOTEC_ID + 1) {
        ((unsigned char*) &(chan[VOLTAGE]))[0] = data[VOLTAGE_BYTE + 1];
        ((unsigned char*) &(chan[VOLTAGE]))[1] = data[VOLTAGE_BYTE];
        ((unsigned char*) &(chan[ENGINE_T]))[0] = data[ENGINE_TEMP_BYTE + 1];
        ((unsigned char*) &(chan[ENGINE_T]))[1] = data[ENGINE_TEMP_BYTE];
        chan[VOLTAGE] = chan[VOLTAGE] / 10;
    } else if(id == MOTEC_ID + 4) {
        ((unsigned char*) &(chan[SPEED]))[0] = data[GDN_SPD_BYTE + 1];
        ((unsigned char*) &(chan[SPEED]))[1] = data[GDN_SPD_BYTE];
        ((unsigned char*) &gear)[0] = data[GEAR_BYTE + 1];
        ((unsigned char*) &gear)[1] = data[GEAR_BYTE];
    }

    return;
}

/*
 *  ADLsample(unsigned char *data, const unsigned char ADLoffset, const unsigned char ch)
 *
 *  Description: This will sample an analog pin and place the value in an array.
 *  Input(s): none
 *  Return Value(s): none
 *  Side Effects: The memory pointed to by data will be modified.
 */
void ADLsample(unsigned char *data, const unsigned char ADLoffset, const unsigned char ch) {

    unsigned int temp;
    SelChanConvADC(ch); // configure which pin you want to read and start A/D converter

    while(BusyADC()); // wait for complete conversion

    // put result in data array in accordance with specified byte location
    temp = (unsigned int) ReadADC();
    modifyRotary(&temp);
    data[ADLoffset] = ((unsigned char *) &temp)[1];
    data[ADLoffset + 1] = ((unsigned char *) &temp)[0];

    return;
}

/*
 *  void modifyRotary(unsigned int * sample)
 *
 *  Description:    This is a function to modify the currently sampled voltage to
 *                  a more suitable value for Motec to read.
 *  Input(s): sample - pointer to raw data from sampling of a rotary's output
 *  Return Value(s): none
 *  Side Effects: This will modify the memory that is pointed to by sample.
 */
void modifyRotary(unsigned int * sample) {
    // depending on the value of the sampled rotary postion
    // we will assign a new position that matches the pysical position
    if(*sample >= RANGE_LOW && *sample < RANGE_0)
        *sample = POS_6;
    else if(*sample >= RANGE_0 && *sample < RANGE_1)
        *sample = POS_5;
    else if(*sample >= RANGE_1 && *sample < RANGE_2)
        *sample = POS_4;
    else if(*sample >= RANGE_2 && *sample < RANGE_3)
        *sample = POS_3;
    else if(*sample >= RANGE_3 && *sample < RANGE_4)
        *sample = POS_7;
    else if(*sample >= RANGE_4 && *sample < RANGE_5)
        *sample = POS_2;
    else if(*sample >= RANGE_5 && *sample < RANGE_6)
        *sample = POS_1;
    else if(*sample >= RANGE_6 && *sample < RANGE_7)
        *sample = POS_0;
    else if(*sample >= RANGE_7 && *sample < RANGE_8)
        *sample = POS_9;
    else if(*sample >= RANGE_8 && *sample <= RANGE_HIGH)
        *sample = POS_8;
    return;
}
