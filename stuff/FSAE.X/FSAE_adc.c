/**
 * FSAE Library ADC
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_adc.h"

/**
 * void init_adc(void (*specific_init)(void))
 *
 * Sets up the ADC module to be able to read any combination of channels using
 * the input scanning method. Will also set up the ADC module to read the
 * on-chip temperature sensor and the external temperature sensor common to all
 * PIC32-based boards.
 *
 * Note: The configuration values for the clock and timing settings still need
 * to be evaluated and tested.
 *
 * @param specific_init A callback function intended to run additional
 *     initialization code for board-specific ADC configuration
 */
void init_adc(void (*specific_init)(void)) {

  // Clear control registers
  ADCCON1 = 0;
  ADCCON2 = 0;
  ADCCON3 = 0;
  ADCANCON = 0;
  ADCTRGSNS = 0;

  // Configure all channels as single-ended, unipolar inputs
  ADCIMCON1 = 0;
  ADCIMCON2 = 0;
  ADCIMCON3 = 0;

  // Disable all global interrupts
  ADCGIRQEN1 = 0;
  ADCGIRQEN2 = 0;

  // Disable all early interrupts
  ADCEIEN1 = 0;
  ADCEIEN2 = 0;

  // Disable digital comparators
  ADCCMPCON1 = 0;
  ADCCMPCON2 = 0;
  ADCCMPCON3 = 0;
  ADCCMPCON4 = 0;
  ADCCMPCON5 = 0;
  ADCCMPCON6 = 0;

  // Disable oversampling filters
  ADCFLTR1 = 0;
  ADCFLTR2 = 0;
  ADCFLTR3 = 0;
  ADCFLTR4 = 0;
  ADCFLTR5 = 0;
  ADCFLTR6 = 0;

  // ADCCON1
  ADCCON1bits.FRACT = 0;   // Fractional Data Output Format (Integer)
  ADCCON1bits.SELRES = 3;  // ADC7 Resolution (12 bits)
  ADCCON1bits.STRGSRC = 1; // Scan Trigger Source Select (Global Software Trigger (GSWTRG))

  // ADCCON2
  ADCCON2bits.SAMC = 68;  // ADC7 Sample Time (70 * Tad7)
  ADCCON2bits.ADCDIV = 1; // ADC7 Clock Divider (Divide by 2)

  // ADCCON3
  ADCCON3bits.ADCSEL = 0b01;   // ADC Clock Source <Tclk> (FRC or SYSCLK?)
  ADCCON3bits.CONCLKDIV = 10;  // ADC Control Clock <Tq> Divider (Divide by 20)
  ADCCON3bits.VREFSEL = 0b000; // Voltage Reference Input Selection (AVdd, AVss)

  // ADCANCON
  ADCANCONbits.WKUPCLKCNT = 7; // Wake-Up Clock Count (2^7 Clocks)

  ADCCON1bits.ON = 1; // ADC Module Enable (Enabled)

  // Wait for a stable reference voltage
  while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
  while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage

  // Enable ADC7
  ADCANCONbits.ANEN7 = 1;      // ADC7 Analog and Bias Circuitry Enable (Enabled)
  while(!ADCANCONbits.WKRDY7); // Wait until ADC7 analog and bias circuitry is ready
  ADCCON3bits.DIGEN7 = 1;      // ADC7 Digital Enable (Enabled)

  // Configure PCB/Junction temp channels
  ADC_PTEMP_TRIS = INPUT;
  ADC_PTEMP_ANSEL = AN_INPUT;
  ADC_PTEMP_TRG = SCAN_TRIGGER;
  //TODO: ADC_JTEMP_TRG = SCAN_TRIGGER;
  ADC_PTEMP_CSS = 1;
  ADC_JTEMP_CSS = 1;

  // Call board-specific ADC init function if one was provided
  if(specific_init != NULL) {
    specific_init();
  }
}

/**
 * Manual method of reading analog channel:
 *
 *  ADCCON3bits.ADINSEL = x; // ANx
 *  ADCCON3bits.SAMP = 1; // Start sampling
 *  // Delay proper amount of time here
 *  ADCCON3bits.SAMP = 0; // Finish sampling
 *  ADCCON3bits.RQCNVRT = 1; // Request convert
 *  while(ADCDSTAT1bits.ARDY9 == 0);
 *  readData = ADCDATAx;
 */

/**
 * uint32_t read_adc_chn(uint8_t chn)
 *
 * Reads a single channel from the ADC and returns the value.
 *
 * Note: This function assumes that the ADC channel sampling process has already
 * been triggered in the background. This should be handled by a timer interrupt.
 *
 * @param chn The channel number to read
 * @returns The value reported by the ADC module of the specified channel
 */
uint32_t read_adc_chn(uint8_t chn) {
  // If a channel is missing, it is not available on the 100-pin uC
  switch(chn) {
    case 0: return ADCDATA0;
    case 1: return ADCDATA1;
    case 2: return ADCDATA2;
    case 3: return ADCDATA3;
    case 4: return ADCDATA4;
    case 5: return ADCDATA5;
    case 6: return ADCDATA6;
    case 7: return ADCDATA7;
    case 8: return ADCDATA8;
    case 9: return ADCDATA9;
    case 10: return ADCDATA10;
    case 11: return ADCDATA11;
    case 12: return ADCDATA12;
    case 13: return ADCDATA13;
    case 14: return ADCDATA14;
    case 15: return ADCDATA15;
    case 16: return ADCDATA16;
    case 17: return ADCDATA17;
    case 18: return ADCDATA18;
    case 19: return ADCDATA19;
    case 20: return ADCDATA20;
    case 21: return ADCDATA21;
    case 22: return ADCDATA22;
    case 23: return ADCDATA23;
    case 24: return ADCDATA24;
    case 25: return ADCDATA25;
    case 26: return ADCDATA26;
    case 27: return ADCDATA27;
    case 28: return ADCDATA28;
    case 29: return ADCDATA29;
    case 30: return ADCDATA30;
    case 31: return ADCDATA31;
    case 32: return ADCDATA32;
    case 33: return ADCDATA33;
    case 34: return ADCDATA34;
    case 43: return ADCDATA43;
    case 44: return ADCDATA44;

    case 46: return ADCDATA1;
    case 47: return ADCDATA2;

    default: return 0;
  }
}
