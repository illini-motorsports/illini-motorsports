/*
 * DAQ Node Main File
 *
 * File Name:       DAQ.c
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18
 * Version:         3.00
 * Author:          George Schwieters
 * Author:          Andrew Mass
 * Created:         2012-2013
 */

#include "DAQ.h"
#include "ECAN.h"
#include "FSAE.h"
#include "capture.h"
#include "FSIO.h"

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

// User data buffers A and B
#pragma udata large_udataA = 0x700 // This specifies the location of the buffers in memory
volatile unsigned char WriteBufferA[BUFFER_SIZE];
#pragma udata large_udataB = 0x900
volatile unsigned char WriteBufferB[BUFFER_SIZE];
#pragma udata

static volatile unsigned int buffer_a_len;
static volatile unsigned int buffer_b_len;
static volatile unsigned char swap;
static volatile unsigned char written;
static volatile unsigned char buffer_a_full;
static volatile unsigned char num_read; // 0 -> 7
static volatile unsigned char msg_num; // 0 -> 3 - Holds index for CCP2 values

static volatile unsigned char can_err;

static volatile BUFFER_TUPLE buffer_tuple;

// timestamp variables
static unsigned int stamp_reg[MSGS_READ];
static unsigned int stamp_sec[MSGS_READ];
static volatile unsigned int curr_stamp_reg[MSGS_READ];
static volatile unsigned int curr_stamp_sec[MSGS_READ];

static volatile signed int rpm; // Engine RPM
static volatile unsigned int seconds; // Timer1 rollover count
static volatile unsigned long millis; // Timer0 rollover count
static volatile unsigned long rpm_tmr; // Last update time of rpm

/*
 * Interrupts
 */

#pragma code high_vector = 0x08

void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma code low_vector = 0x18

void interrupt_at_low_vector(void) {
    _asm goto low_isr _endasm
}
#pragma code

/*
 * void low_isr(void)
 *
 * Description: This interrupt will service all low priority interrupts. This includes
 *              servicing the ECAN FIFO buffer and handling ECAN errors. This is not
 *              a traditional interrupt due do the nature of the project. The interrupt is
 *              by no means short but it must be this way in order to be able to buffer
 *              data at all times even when the uSD card is being written to in the main
 *              loop.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify PIR5, B0CON, B1CON, B2CON, B3CON, B4CON, B5CON,
 *               COMSTAT, RXB0CON & RXB1CON. This will also modify the flags in Main
 */
#pragma interruptlow low_isr

void low_isr(void) {

    // Service FIFO RX buffers
    if(PIR5bits.FIFOWMIF) {
        PIR5bits.FIFOWMIF = 0;
        can_err = 0;

        // Sometimes a FIFO interrupt is triggered before 4 messages have arrived; da fuk?
        if(num_read < MSGS_READ) {
            return;
        }

        num_read = 0;

        if(swap) {
            buffer_a_len = 0; // Reset buffer A length
            swap = 0; // Clear swap flag

            // We have been caching data in buffer B if this expression is true
            if(buffer_b_len != 0) {
                swap_len();
                swap_buff();
            }
        }

        // Process data in CAN FIFO
        read_CAN_buffers();
    }

    // Check for an error with the bus
    if(PIR5bits.ERRIF) {
        PIR5bits.ERRIF = 0;

        // Receive buffer overflow occurred
        if(COMSTATbits.RXB0OVFL || COMSTATbits.RXB1OVFL) {
            //Clear out all the buffers
            COMSTATbits.RXB0OVFL = 0;
            COMSTATbits.RXB1OVFL = 0;
            RXB0CONbits.RXFUL = 0;
            RXB1CONbits.RXFUL = 0;
            B0CONbits.RXFUL = 0;
            B1CONbits.RXFUL = 0;
            B2CONbits.RXFUL = 0;
            B3CONbits.RXFUL = 0;
            B4CONbits.RXFUL = 0;
            B5CONbits.RXFUL = 0;
        }

        // Transmit bus-off state (<255 errors)
        if(COMSTATbits.TXBO) {
            can_err = 1;
        }
    }
}

/*
 * void high_isr(void)
 *
 * Description: This interrupt will service all high priority interrupts which includes
 *              timer 1 rollover and the capture 2 module.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify TMR1H, TMR1L, PIR1 & PIR3. Also seconds,
 *               stamp, stamp_2, time_stamp, time_stamp_2 & Main variables will be written to.
 */
#pragma interrupt high_isr

void high_isr(void) {
    unsigned int temp;
    unsigned char k = 0;

    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // Load timer registers (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // Check for timer1 rollover
    if(PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;

        /*
         * Load timer value such that the most significant bit is set so it
         * takes exactly one second for a 32.768kHz crystal to trigger a rollover interrupt
         *
         * WriteTimer1(TMR1H_RELOAD * 256 + TMR1L_RELOAD);
         */
        TMR1H = TMR1H_RELOAD;
        TMR1L = TMR1L_RELOAD;
        seconds++;
    }

    // Check for incoming message to give timestamp
    if(PIR3bits.CCP2IF) {
        PIR3bits.CCP2IF = 0;

        /*
         * Read CAN capture register and place in timestamp array keeping track
         * of how many are in the array
         *
         * ReadCapture2();
         */
        stamp_reg[msg_num] = CCPR2H * 256 + CCPR2L;
        stamp_sec[msg_num] = seconds;
        msg_num = msg_num == 3 ? 0 : msg_num + 1;

        // Check if four messages have come in so far
        if(msg_num == 0) {
            // swap the data byte by byte
            for(k = 0; k < MSGS_READ; k++) {
                temp = curr_stamp_reg[k];
                curr_stamp_reg[k] = stamp_reg[k];
                stamp_reg[k] = temp;

                temp = curr_stamp_sec[k];
                curr_stamp_sec[k] = stamp_sec[k];
                stamp_sec[k] = temp;
            }

            num_read = MSGS_READ;
        }
    }
}

void main(void) {

    /*
     * Variable Declarations
     */

    FSFILE* outfile; // Pointer to open file
    const char write = 'w'; // For opening file (must use variable for passing value in PIC18 when not using pgm function)
    SearchRec rec; // Holds search parameters and found file info
    const unsigned char attributes = ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN;

    char fname[9] = "0000.txt"; // Holds name of file
    unsigned int fnum = 0; // Holds number of filename
    unsigned int temp = 0;
    unsigned char filename_msg[2];
    unsigned int fnum_max = 9999;
    unsigned int fnum_min = 0;

    unsigned long CAN_send_tmr = 0;

    init_unused_pins();

    /*
     * Variable Initialization
     */

    /*
     * Initialize MDD I/O variables but don't allow data collection yet since
     * file creation can take a while and buffers will fill up immediately.
     */
    buffer_tuple.left = WriteBufferA;
    buffer_tuple.right = WriteBufferB;
    buffer_a_len = MEDIA_SECTOR_SIZE;
    buffer_b_len = MEDIA_SECTOR_SIZE;
    swap = 0;
    buffer_a_full = 1;
    written = 0;

    can_err = 0;

    msg_num = 0;
    num_read = 0;

    rpm = 0;
    seconds = 0;
    millis = 0;
    rpm_tmr = 0;

    /*
     * Peripheral Initialization
     */

    init_oscillator();

    ANCON0 = 0x00; // Default all pins to digital
    ANCON1 = 0x00; // Default all pins to digital

    SD_CS_TRIS = OUTPUT; // Set card select pin to output
    SD_CS = 1; // Card deselected

#ifdef LOGGING_0
    TRISCbits.TRISC6 = OUTPUT; // programmable termination
    TERM_LAT = FALSE;
#endif

    // Setup milliseconds interrupt
    init_timer0();

    // Setup seconds interrupt
    init_timer1();

    // Turn on and configure capture module
    OpenCapture2(CAP_EVERY_FALL_EDGE & CAPTURE_INT_ON);
    IPR3bits.CCP2IP = 1; // High priority

    while(!MDD_MediaDetect()); // Wait for card presence
    while(!FSInit()); // Setup file system library
    ECANInitialize(); // Setup ECAN module

    ECANCONbits.FIFOWM = 0; // FIFO High Water Mark (Will cause FIFO interrupt when four receive buffers remain)

    // Configure interrupts
    RCONbits.IPEN = 1; // Interrupt Priority Enable (1 enables)
    PIE5bits.ERRIE = 1; // Enable CAN bus error interrupt
    PIE5bits.FIFOWMIE = 1; // Enables CAN FIFO high watermark interrupt
    STI();

    // Run binary search to find the largest filename
    while(fnum_max >= fnum_min) {
        fnum = (fnum_max + fnum_min) / 2;

        temp = fnum;
        fname[3] = (fnum % 10) + 0x30;
        fnum /= 10;
        fname[2] = (fnum % 10) + 0x30;
        fnum /= 10;
        fname[1] = (fnum % 10) + 0x30;
        fnum /= 10;
        fname[0] = (fnum % 10) + 0x30;
        fnum = temp;

        // Look for file with proposed name
        if(FindFirst(fname, attributes, &rec)) {
            if(FSerror() == CE_FILE_NOT_FOUND) {
                // Filename is unique
                fnum_max = fnum - 1;
                continue;
            } else {
                //TODO: Handle this error gracefully.
                abort();
            }
        }

        // Filename is not unique
        fnum_min = fnum + 1;
    }

    /*
     * Main Loop
     */

    while(1) {
        // Check if we want to start data logging
        CLI(); // Begin critical section
        if(!can_err && millis - rpm_tmr < RPM_WAIT && rpm > RPM_THRESH) {
            STI(); // End critical section

            while(!MDD_MediaDetect()); // Wait for card presence

            // Run filename loop again (should only run once) to get the next available filename
            while(1) {
                if(fnum > 9999) {
                    /**
                     * We have reached the 10000th logfile. In practice, this
                     * will almost never happen.
                     *
                     * TODO: Handle this error gracefully.
                     */
                    abort();
                }

                temp = fnum;
                fname[3] = (fnum % 10) + 0x30;
                fnum /= 10;
                fname[2] = (fnum % 10) + 0x30;
                fnum /= 10;
                fname[1] = (fnum % 10) + 0x30;
                fnum /= 10;
                fname[0] = (fnum % 10) + 0x30;
                fnum = temp;

                // Look for file with proposed name
                if(FindFirst(fname, attributes, &rec)) {
                    if(FSerror() == CE_FILE_NOT_FOUND) {
                        break; // Exit loop, file name is unique
                    } else {
                        //TODO: Handle this error gracefully.
                        abort();
                    }
                }

                fnum++;
            }

            // Create csv data file
            outfile = FSfopen(fname, &write);
            if(outfile == NULL) {
                abort();
            }

            // Setup buffers and flags to begin data collection
            CLI(); // Begin critical section
            buffer_a_len = 0;
            buffer_b_len = 0;
            buffer_a_full = 0;
            STI(); // End critical section

            // Logging loop
            while(1) {
                // Write buffer A to file
                CLI(); // Begin critical section
                if(buffer_a_full) {
                    STI(); // End critical section
                    if(FSfwrite(&buffer_tuple, outfile, &swap, &buffer_a_full) != BUFFER_SIZE) {
                        //TODO: Handle this more gracefully
                        abort();
                    }
                }
                STI(); // End critical section

                CLI(); // Begin critical section
                // Check if we should stop logging
                if(can_err || rpm < RPM_THRESH || millis - rpm_tmr > RPM_WAIT) {
                    STI(); // End critical section

                    // Close csv data file
                    if(FSfclose(outfile)) {
                        abort();
                    }

                    CLI(); // Begin critical section
                    // Stop collecting data in the buffers
                    buffer_a_len = BUFFER_SIZE;
                    buffer_b_len = BUFFER_SIZE;
                    swap = 0;
                    buffer_a_full = 1;
                    STI(); // End critical section

                    break;
                }
                STI(); // End critical section

                CLI(); // Begin critical section
                // Send filename on CAN
                if(millis - CAN_send_tmr > CAN_PERIOD) {
                    CAN_send_tmr = millis;
                    STI(); // End critical section

                    filename_msg[0] = ((unsigned char*)&fnum)[0];
                    filename_msg[1] = ((unsigned char*)&fnum)[1];
                    ECANSendMessage(LOGGING_ID, filename_msg, 2,
                            ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
                }
                STI(); // End critical section
            }
        }
        STI(); // End critical section

        CLI(); // Begin critical section
        // Send "-1" as filename on CAN
        if(millis - CAN_send_tmr > CAN_PERIOD) {
            CAN_send_tmr = millis;
            STI(); // End critical section

            filename_msg[0] = 0xFF;
            filename_msg[1] = 0xFF;
            ECANSendMessage(LOGGING_ID, filename_msg, 2,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
        }
        STI(); // End critical section
    }
}

/*
 *  Local Functions
 */

/*
 * void abort(void)
 *
 * Description: This function will be called when a serious error is encountered
 *              and will halt execution of all other code.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This halts main program execution.
 */
void abort(void) {
    CLI(); // Disable all interrupts
    while(1); // To infinity and beyond
}

/*
 * void read_CAN_buffers(void)
 *
 * Description: This function will read messages from the ECAN buffers and package
 *              the information with a timestamp. It also reads RPM data for
 *              logging initiation.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify rpm. Also it clears the ECAN receive buffers.
 */
void read_CAN_buffers(void) {
    unsigned char i, j;

    unsigned long id; // CAN msgID
    unsigned char dlc; // Number of CAN data bytes
    unsigned char data[8]; // CAN data bytes
    unsigned char msg[14]; // Entire CAN message

    ECAN_RX_MSG_FLAGS flags; // Information about received message

    // Loop through messages in CAN buffers
    for(i = 0; i < MSGS_READ; i++) {
        id = 0; // Clear ID in case ECANReceiveMessage() fails silently
        dlc = 0; // Clear DLC in case ECANReceiveMessage() fails silently

        ECANReceiveMessage(&id, data, &dlc, &flags);

        if(id == RPM_ID) {
            ((unsigned char*)&rpm)[0] = data[RPM_BYTE + 1];
            ((unsigned char*)&rpm)[1] = data[RPM_BYTE];
            rpm_tmr = millis;
        }

        if(id == BEACON_ID) {
            // Discard the message if the dlc does not match the DLC for beacon messages
            if(dlc != 6) {
                dlc = 0;
            }
        }

        // Ensure there's data to record
        if(dlc > 0) {

            // Message ID
            msg[0] = ((unsigned char*)(&id))[0];
            msg[1] = ((unsigned char*)(&id))[1];

            // CAN Data
            for(j = 0; j < dlc; j++) {
                msg[2 + j] = data[j];
            }

            // Timestamp
            msg[2 + dlc + 0] = ((unsigned char*)curr_stamp_reg)[0 + (i * 2)];
            msg[2 + dlc + 1] = ((unsigned char*)curr_stamp_reg)[1 + (i * 2)];
            msg[2 + dlc + 2] = ((unsigned char*)curr_stamp_sec)[0 + (i * 2)];
            msg[2 + dlc + 3] = ((unsigned char*)curr_stamp_sec)[1 + (i * 2)];

            // Write msg to a buffer
            append_write_buffer(msg, dlc + 6);
        }
    }
}

/*
 * void append_write_buffer(static const unsigned char * temp,
 *                              static unsigned char applen)
 *
 * Description:    This function will decide how to append data to the user buffers.
 *                 It will try to put the entire message in BufferA if possible.
 *                 Otherwise it will either put a partial message in BufferA
 *                 and BufferB or the entire message in BufferB.
 * Input(s):   temp - data array that holds one whole CAN message with its timestamp
 *             applen - the length of the data array
 * Return Value(s): none
 * Side Effects: This will modify Main.
 */
void append_write_buffer(const unsigned char* temp, unsigned char applen) {
    unsigned char offset = 0;
    unsigned char holder = 0;

    written = 0;

    // Message is dropped
    if(applen > (2 * BUFFER_SIZE - (buffer_a_len + buffer_b_len))) {
        return;
    }

    // Try writing to buffer A first
    if(!buffer_a_full) {
        // Room for all of data to write
        if(BUFFER_SIZE - buffer_a_len > applen) {
            written = 1;
        }// Not enough room for all the data
        else if(BUFFER_SIZE - buffer_a_len < applen) {
            buffer_a_full = 1;
            // Recalculate writing parameters for partial write
            offset = applen - (BUFFER_SIZE - buffer_a_len);
        }// Exactly enough room for the data
        else {
            buffer_a_full = 1;
            written = 1;
        }

        // Add message to buffer
        buff_cat(buffer_tuple.left, temp, &buffer_a_len, applen - offset, 0);
    }

    // Write to buffer B if couldn't write any or all data to buffer A
    if(!written) {
        // Only use offset if there has been a partial write to buffer A
        if(offset != 0) {
            holder = applen;
            applen = offset;
            offset = holder - offset;
        }
        // Add message to buffer
        buff_cat(buffer_tuple.right, temp, &buffer_b_len, applen, offset);
    }
}

/*
 * void buff_cat(unsigned char *WriteBuffer, const unsigned char *writeData,
 *              unsigned int *bufflen, const unsigned char applen,
 *              const unsigned char offset)
 *
 * Description: This function will append data to the user buffers byte by byte.
 * Input(s):   WriteBuffer - pointer to the buffer we will write to
 *             writeData - pointer to the data array we will write
 *             bufflen - pointer to the variable holding the buffer length of the buffer we are writing to
 *             applen - the length of the data to be written
 *             offset - the index offset for the data to write
 * Return Value(s): none
 * Side Effects: This will modify BufferA & BufferB.
 */
void buff_cat(unsigned char* WriteBuffer, const unsigned char* writeData,
        unsigned int* bufflen, const unsigned char applen,
        const unsigned char offset) {
    unsigned char i;

    // Increment through the data bytes sending them to the write buffer
    for(i = 0; i < applen; i++) {
        WriteBuffer[*bufflen + i] = writeData[i + offset];
    }

    // Increment the data length count for the writing buffer
    *bufflen += (unsigned int)applen;
}

/*
 * void swap_len(void)
 *
 * Description: Swaps the buffer length members of Main.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will affect buffer_a_len and buffer_b_len.
 */
void swap_len(void) {
    unsigned int temp = buffer_a_len;
    buffer_a_len = buffer_b_len;
    buffer_b_len = temp;
}

/*
 * void swap_buff(void)
 *
 * Description: Swaps the buffer pointer members of Buff.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify Buff.
 */
void swap_buff(void) {
    unsigned char* temp = buffer_tuple.left;
    buffer_tuple.left = buffer_tuple.right;
    buffer_tuple.right = temp;
}
