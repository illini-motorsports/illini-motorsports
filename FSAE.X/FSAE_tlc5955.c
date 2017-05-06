/**
 * FSAE Library TLC5955
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_tlc5955.h"

uint8_t pending_register[NUM_BYTES] = {0};
uint16_t bit_ctr = 0;

uint64_t current_colors[16] = {0}; // Red:16, Green:16, Blue:16
uint8_t led_mapping[16] = {12, 11, 10, 5, 13, 14, 2, 0, 9, 7, 1, 3, 8, 6, 15, 4};

uint8_t left_cluster_warn = 0;
uint64_t left_cluster_color = 0x0;
uint32_t left_cluster_warn_tmr = 0;
uint8_t left_cluster_idx = 0;

uint8_t right_cluster_warn = 0;
uint64_t right_cluster_color = 0x0;
uint32_t right_cluster_warn_tmr = 0;
uint8_t right_cluster_idx = 0;

uint8_t main_blink = 0;
uint64_t main_blink_color = 0x0;
uint32_t main_blink_tmr = 0;
uint8_t main_blink_flag = 0;

uint8_t startup = 0;
uint8_t startup_frame = 0;
uint32_t startup_tmr = 0;

/**
 * Performs a startup sequence
 */
void tlc5955_startup(void) {
  startup = 1;
  startup_frame = 0;
  _tlc5955_startup_frame();
}

/**
 * Gets the current state of the startup variable
 */
uint8_t tlc5955_get_startup(void) {
  return startup;
}

/**
 * Toggles the main LED row blink state
 */
void tlc5955_set_main_blink(uint8_t on, uint64_t color, uint8_t ovr) {
  if (startup && !ovr) return;
  if (on && !main_blink) {
    main_blink = on;
    main_blink_color = color;
    main_blink_tmr = millis;
    tlc5955_write_main_color(color, ovr);
  } else if (!on && main_blink) {
    main_blink = on;
  }
}

/**
 * Toggles the left/right cluster triangle warning state
 *
 * Precondition: Must be called after startup sequence is over
 */
void tlc5955_set_cluster_warn(uint8_t which, uint8_t on, uint64_t color,
    uint8_t ovr) {
  if (startup && !ovr) return;
  switch(which) {
    case CLUSTER_LEFT:
      if (on && !left_cluster_warn) {
        current_colors[0] = left_cluster_idx != 0 ? color : 0x0;
        current_colors[1] = left_cluster_idx != 1 ? color : 0x0;
        current_colors[2] = left_cluster_idx != 2 ? color : 0x0;
        left_cluster_warn_tmr = millis;
        _tlc5955_write_gs();
      } else if (!on && left_cluster_warn) {
        current_colors[0] = 0x0;
        current_colors[1] = 0x0;
        current_colors[2] = 0x0;
        _tlc5955_write_gs();
      }

      left_cluster_warn = on;
      left_cluster_color = color;
      break;

    case CLUSTER_RIGHT:
      if (on && !right_cluster_warn) {
        current_colors[12] = right_cluster_idx != 0 ? color : 0x0;
        current_colors[13] = right_cluster_idx != 1 ? color : 0x0;
        current_colors[14] = right_cluster_idx != 2 ? color : 0x0;
        right_cluster_warn_tmr = millis;
        _tlc5955_write_gs();
      } else if (!on && right_cluster_warn) {
        current_colors[12] = 0x0;
        current_colors[13] = 0x0;
        current_colors[14] = 0x0;
        _tlc5955_write_gs();
      }

      right_cluster_warn = on;
      right_cluster_color = color;
      break;
  }
}

/**
 * Gets current state of the cluster warning
 */
uint8_t tlc5955_get_cluster_warn(uint8_t which) {
  switch(which) {
    case CLUSTER_LEFT:
      return left_cluster_warn;
    case CLUSTER_RIGHT:
      return right_cluster_warn;
  }
}

/**
 * Writes a color to a set of lights specified by onMap
 */
void tlc5955_write_color(uint64_t color, uint16_t onMap, uint8_t ovr) {
  if (startup && !ovr) { return; }
  uint8_t i;
  for (i = 0; i < 16; i++) {
    uint16_t on = onMap & (1 << led_mapping[i]);
    current_colors[led_mapping[i]] = on ? color : 0x0;
  }
  _tlc5955_write_gs();
}

/**
 * Sets a specific set of LEDs to the specified color
 */
void tlc5955_set_leds(uint64_t color, uint16_t setMap, uint8_t ovr) {
  if (startup && !ovr) { return; }
  uint8_t i;
  for (i = 0; i < 16; i++) {
    uint16_t set = setMap & (1 << led_mapping[i]);
    if (set) {
      current_colors[led_mapping[i]] = color;
    }
  }
  _tlc5955_write_gs();
}

/**
 * Writes a color to the main row of LEDs
 */
void tlc5955_write_main_color(uint64_t color, uint8_t ovr) {
  if (startup && !ovr) return;
  uint8_t i;
  for (i = 0; i < NUM_LED_MAIN; i++) {
    current_colors[i + 3] = color;
  }
  _tlc5955_write_gs();
}

/**
 * Writes the colors specified to the main row of LEDs
 */
void tlc5955_write_main_colors(uint64_t* colors) {
  if (startup) return;
  uint8_t i;
  for (i = 0; i < NUM_LED_MAIN; i++) {
    current_colors[i + 3] = colors[i];
  }
  _tlc5955_write_gs();
}

/**
 * Check various timer-based events and respond if necessary
 */
void tlc5955_check_timers() {
  if (startup && (millis - startup_tmr > STARTUP_FRAME_INTV)) {
    _tlc5955_startup_frame();
    return;
  }

  if (main_blink && (millis - main_blink_tmr > MAIN_BLINK_INTV)) {
    tlc5955_write_main_color(main_blink_flag ? main_blink_color : 0x0, startup);
    main_blink_tmr = millis;
    main_blink_flag = !main_blink_flag;
  }

  if (left_cluster_warn && (millis - left_cluster_warn_tmr > CLUSTER_WARN_INTV)) {
    left_cluster_idx++;
    if (left_cluster_idx >= 3) { left_cluster_idx = 0; }

    current_colors[0] = left_cluster_idx != 0 ? left_cluster_color : 0x0;
    current_colors[1] = left_cluster_idx != 1 ? left_cluster_color : 0x0;
    current_colors[2] = left_cluster_idx != 2 ? left_cluster_color : 0x0;

    left_cluster_warn_tmr = millis;
    _tlc5955_write_gs();
  }

  if (right_cluster_warn && (millis - right_cluster_warn_tmr > CLUSTER_WARN_INTV)) {
    right_cluster_idx++;
    if (right_cluster_idx >= 3) { right_cluster_idx = 0; }

    current_colors[12] = right_cluster_idx != 0 ? right_cluster_color : 0x0;
    current_colors[13] = right_cluster_idx != 1 ? right_cluster_color : 0x0;
    current_colors[14] = right_cluster_idx != 2 ? right_cluster_color : 0x0;

    right_cluster_warn_tmr = millis;
    _tlc5955_write_gs();
  }
}

/**
 * Initializes the TLC5955 module
 */
void init_tlc5955(void) {
  // GSCLK PWM Signal
  PWM_TLC5955_TRIS = OUTPUT;
  PWM_TLC5955_LAT = 1;

  // Latch - SHFLAT
  LAT_TLC5955_TRIS = OUTPUT;
  LAT_TLC5955_LAT = 0;

  _tlc5955_init_spi();

  _tlc5955_write_control();
  tlc5955_write_color(OFF, 0xFFFF, NO_OVR); // Set all off
}

/**
 * Sets the next frame of the startup sequence
 */
void _tlc5955_startup_frame(void) {
  if (startup_frame >= MAX_STARTUP_FRAMES) {
    startup = 0;
    tlc5955_set_cluster_warn(CLUSTER_LEFT, 0, OFF, OVR);
    tlc5955_set_cluster_warn(CLUSTER_RIGHT, 0, OFF, OVR);
    tlc5955_set_main_blink(0, OFF, OVR);
    tlc5955_write_color(0x0, 0x0, OVR); // Turn off all LEDs
    return;
  }

  switch(startup_frame) {
    case 0:
      tlc5955_set_cluster_warn(CLUSTER_LEFT, 1, WHT, OVR);
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, 1, WHT, OVR);
      break;
    case 1:
      tlc5955_set_leds(RED, 0b000100000001000, OVR);
      break;
    case 2:
      tlc5955_set_leds(RED, 0b000010000010000, OVR);
      break;
    case 3:
      tlc5955_set_leds(RED, 0b000001000100000, OVR);
      break;
    case 4:
      tlc5955_set_leds(RED, 0b000000101000000, OVR);
      break;
    case 5:
      tlc5955_set_leds(RED, 0b000000010000000, OVR);
      break;

    case 6:
      tlc5955_set_cluster_warn(CLUSTER_LEFT, 1, BLU, OVR);
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, 1, BLU, OVR);
      break;

    case 7:
      tlc5955_set_leds(WHT, 0b000100000001000, OVR);
      break;
    case 8:
      tlc5955_set_leds(WHT, 0b000010000010000, OVR);
      break;
    case 9:
      tlc5955_set_leds(WHT, 0b000001000100000, OVR);
      break;
    case 10:
      tlc5955_set_leds(WHT, 0b000000101000000, OVR);
      break;
    case 11:
      tlc5955_set_leds(WHT, 0b000000010000000, OVR);
      break;

    case 12:
      tlc5955_set_cluster_warn(CLUSTER_LEFT, 1, RED, OVR);
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, 1, RED, OVR);
      break;

    case 13:
      tlc5955_set_leds(BLU, 0b000100000001000, OVR);
      break;
    case 14:
      tlc5955_set_leds(BLU, 0b000010000010000, OVR);
      break;
    case 15:
      tlc5955_set_leds(BLU, 0b000001000100000, OVR);
      break;
    case 16:
      tlc5955_set_leds(BLU, 0b000000101000000, OVR);
      break;
    case 17:
      tlc5955_set_leds(BLU, 0b000000010000000, OVR);
      break;

    case 18:
      tlc5955_set_main_blink(1, BLU, OVR);
      break;

    default:
      break;
  }

  startup_frame++;
  startup_tmr = millis;
}

/**
 * Writes default values to the TLC5955 control registers
 */
void _tlc5955_write_control(void) {
  uint8_t i, j;

  bit_ctr = 0;
  for (i = 0; i < NUM_BYTES; i++) {
    pending_register[i] = 0;
  }

  _tlc5955_reg_append(7, 0x0); // Shift in 7 bits to account for 776 bit SPI shifts

  _tlc5955_reg_append(1, 0b1); // MSB indicates control register write
  _tlc5955_reg_append(8, 0x96); // More indication for control register write

  // Write DC registers
  for (i = 0; i < 16; i++) {
    for (j = 0; j < 3; j++) {
      _tlc5955_reg_append(7, 0b1111111);
    }
  }

  // Write MC registers
  _tlc5955_reg_append(3, 0b011); // Red
  _tlc5955_reg_append(3, 0b101); // Green
  _tlc5955_reg_append(3, 0b101); // Blue

  // Write BC registers
  _tlc5955_reg_append(7, 0b0111111); // Red
  _tlc5955_reg_append(7, 0b1111111); // Green
  _tlc5955_reg_append(7, 0b0111111); // Blue

  // Write FC registers
  _tlc5955_reg_append(1, 0b1); // DSPRPT
  _tlc5955_reg_append(1, 0b0); // TMGRST
  _tlc5955_reg_append(1, 0b0); // RFRESH
  _tlc5955_reg_append(1, 0b1); // ESPWM
  _tlc5955_reg_append(1, 0b0); // LSDVLT

  _tlc5955_send_register();
}

/**
 * Writes the values from the current_colors array into the GS data registers
 */
void _tlc5955_write_gs(void) {
  uint8_t i;

  bit_ctr = 0;
  for (i = 0; i < NUM_BYTES; i++) {
    pending_register[i] = 0;
  }

  _tlc5955_reg_append(7, 0x0); // Shift in 7 bits to account for 776 bit SPI shifts
  _tlc5955_reg_append(1, 0b0); // MSB indicates GS write

  for (i = 0; i < 16; i++) {
    uint64_t color = current_colors[led_mapping[i]];
    _tlc5955_reg_append(8, (color >> 0) & 0xFF); // Blue LW
    _tlc5955_reg_append(8, (color >> 8) & 0xFF); // Blue LW
    _tlc5955_reg_append(8, (color >> 16) & 0xFF); // Green LW
    _tlc5955_reg_append(8, (color >> 24) & 0xFF); // Green LW
    _tlc5955_reg_append(8, (color >> 32) & 0xFF); // Red LW
    _tlc5955_reg_append(8, (color >> 40) & 0xFF); // Red LW
  }

  _tlc5955_send_register();
}

/**
 * Appends <num_bits> of <data> to the pending_register array in preparation
 * for sending a register to the chip.
 *
 * TODO: Also make this less bad
 */
void _tlc5955_reg_append(uint8_t num_bits, uint8_t data) {
  uint8_t i;
  for (i = 0; i < num_bits; i++) {
    uint8_t bit = data & (1 << (num_bits-1-i)); // selects bits in reverse order
    uint8_t idx = (bit_ctr + i) >> 3; // faster divide by 8
    uint8_t pos = (bit_ctr + i) % 8;

    if (bit) {
      pending_register[idx] |= (1 << pos);
    } else {
      pending_register[idx] &= ~(1 << pos);
    }
  }

  bit_ctr += num_bits;
}

/**
 * Sends the pending_register array over the SPI bus as a continuous steam of
 * <NUM_BYTES> messages.
 */
void _tlc5955_send_register(void) {

  /*
   * Need to send data from MSbit to LSbit. Then, after correct number of
   * pulses, the control bit (bit 769) will have made it to the end of the
   * register.
   *
   * Then latch at the end.
   *
   * Also account for the fact that we're shifting more than 769 bits
   */
  uint8_t i;
  uint8_t resp;

  for (i = 0; i < NUM_BYTES; i++) {
    SPI2BUF = pending_register[i];
    while(!SPI2STATbits.SPIRBF);
    resp = SPI2BUF;
  }

  // Send latch pulse
  LAT_TLC5955_LAT = 1;
  for (i = 0; i < 6; i++);
  LAT_TLC5955_LAT = 0;
}

/**
 * Initializes the SPI2 communication bus for use with the TLC5955 chip
 */
void _tlc5955_init_spi(void) {
  unlock_config();

  // Initialize SDO2 PPS pin
  CFGCONbits.IOLOCK = 0;
  TRISGbits.TRISG7 = OUTPUT;
  RPG7Rbits.RPG7R = 0b0110; // SDO2
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK2
  TRISGbits.TRISG6 = OUTPUT; // SCK2

  // Disable interrupts
  IEC4bits.SPI2EIE = 0;
  IEC4bits.SPI2RXIE = 0;
  IEC4bits.SPI2TXIE = 0;

  // Disable SPI2 module
  SPI2CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI2BUF;

  // Use standard buffer mode
  SPI2CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI2BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI2BRG = 4;

  SPI2STATbits.SPIROV = 0;

  SPI2CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI2CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI2CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.DISSDI = 0;
  SPI2CONbits.DISSDO = 0;
  SPI2CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI2CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI2CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI2CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI2 module
  SPI2CONbits.ON = 1;

  lock_config();
}

