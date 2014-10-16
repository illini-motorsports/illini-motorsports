/******************************************************************************
 *
 *               Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC18
 * Compiler:        C18
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
/******************************************************************************
 Change History:
  Rev            Description
  ----           -----------------------
  1.3.4          Added support for PIC18F8722,PIC24FJ256DA210,
                 dsPIC33E & PIC24E microcontrollers.
                 Added macro "SPI_INTERRUPT_FLAG_ASM" for PIC18F
                 microcontrollers to support SD card SPI driver.

*******************************************************************************
    USER REVISON HISTORY
    note: modified to work with just PIC18F46K80
//
// 10/22/12 - removed extra macros for other platfroms and confirmed/set I/O pins and registers
//
// 10/30/12 - removed uart2 header include line
//
// 10/31/12 - changed SPICLOCKPORT and SPICLOCKLAT defines to match pinouts in Eagle
//
//

*******************************************************************************/

#ifndef _HARDWAREPROFILE_H_
#define _HARDWAREPROFILE_H_

// Define clock speed:
//
// Sample clock speed for PIC18
#define GetSystemClock()        64000000                        // System clock frequency (Hz)
#define GetPeripheralClock()    GetSystemClock()                // Peripheral clock freq.
#define GetInstructionClock()   (GetSystemClock() / 4)          // Instruction clock freq.

// Select your interface type
// Description: Macro used to enable the SD-SPI physical layer (library files SD-SPI.c and .h)
#define USE_SD_INTERFACE_WITH_SPI


/*********************************************************************/
/******************* Pin and Register Definitions ********************/
/*********************************************************************/

/////////////////////////////////////////////////////
// SD Card definitions: set to fit the application //
/////////////////////////////////////////////////////

#define USE_PIC18
#define USE_SD_INTERFACE_WITH_SPI

// Registers for the SPI module to be used
//
// Description: The main SPI control register
#define SPICON1             SSPCON1
// Description: The SPI status register
#define SPISTAT             SSPSTAT
// Description: The SPI buffer
#define SPIBUF              SSPBUF
// Description: The receive buffer full bit in the SPI status register
#define SPISTAT_RBF         SSPSTATbits.BF
// Description: The bitwise define for the SPI control register (i.e. _____bits)
#define SPICON1bits         SSPCON1bits
// Description: The bitwise define for the SPI status register (i.e. _____bits)
#define SPISTATbits         SSPSTATbits
// Description: The interrupt flag
#define SPI_INTERRUPT_FLAG  PIR1bits.SSPIF
#define SPI_INTERRUPT_FLAG_ASM  PIR1, 3
// Description: The enable bit
#define SPIENABLE           SSPCON1bits.SSPEN

// TRIS pins for SCK/SDI/SDO/CS lines
#define SPICLOCK            TRISCbits.TRISC3
#define SPIIN               TRISCbits.TRISC4
#define SPIOUT              TRISCbits.TRISC5
#define SD_CS_TRIS          TRISDbits.TRISD3

// Latch pins for SCK/SDI/SDO/CS lines
#define SPICLOCKLAT         LATCbits.LATC3
#define SPIINLAT            LATCbits.LATC4
#define SPIOUTLAT           LATCbits.LATC5
#define SD_CS               LATDbits.LATD3

// Port pins for SCK/SDI/SDO lines
#define SPICLOCKPORT        PORTCbits.RC3
#define SPIINPORT           PORTCbits.RC4
#define SPIOUTPORT          PORTCbits.RC5

// Will generate an error if the clock speed is too low to interface to the card
#if (GetSystemClock() < 400000)
   #error System clock speed must exceed 400 kHz
#endif

#endif
