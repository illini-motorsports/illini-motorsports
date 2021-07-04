/**
 * GCM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "GCM.h"


// Current GCM mode
volatile gcm_mode mode = NORMAL_MODE;

// Auto-upshift switch enabled

volatile uint8_t auto_upshift_disable_override = 0;
volatile uint8_t attempting_auto_upshift = 0;
volatile uint8_t throttle_pos_passed_min_auto = 0;
volatile uint8_t radio_button = 0;
volatile uint8_t acknowledge_button = 0;
volatile uint8_t auxiliary_button = 0;
volatile uint8_t night_day_switch, auto_upshift_switch = 0;

// Internal (milli)second timers)
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

volatile uint8_t gear = GEAR_FAIL; // Current gear
double shift_force = 0.0; // Shift force sensor reading

volatile uint8_t queue_up = 0; // Number of queued upshifts #CHECK
volatile uint8_t queue_dn = 0; // Number of queued downshifts
volatile uint8_t queue_nt = 0; // Number of queued neutral shifts

uint8_t retry_up = 0; // Number of retried upshifts
uint8_t retry_dn = 0; // Number of retried downshifts
uint8_t retry_nt = 0; // Number of retried neutral shifts

uint8_t switch_debounced = 0; // Debounced (safe) switch state
uint8_t switch_prev = 0; // Previous sampled value of switches

volatile uint8_t prev_switch_up = 0; // Previous switch state of SHIFT_UP #CHECK
volatile uint8_t prev_switch_dn = 0; // Previous switch state of SHIFT_DN

volatile double eng_rpm = 0; // Engine RPM (from ECU) #CHECK
volatile double bat_volt = 0; // Battery voltage (from ECU)
volatile double throttle_pos = 0; // Throttle position (from ECU) #CHECK
volatile uint16_t shift_force_ecu = 0; // Spoofed shift force (from ECU)
volatile uint8_t kill_sw = 0; // Holds state of KILL_SW (from PDM)
volatile uint8_t wheel_fl_speed = 0;
volatile uint8_t wheel_fr_speed = 0;
volatile uint8_t wheel_rl_speed = 0;
volatile uint8_t wheel_rr_speed = 0;
volatile double lat_accel = 0;
volatile uint16_t conv_lat_accel = 0;

// Ignition cut flags
volatile uint8_t ignition_cut_upshift = IGNITION_CUT_DISABLE;
volatile uint8_t ignition_cut_downshift = IGNITION_CUT_DISABLE;
static uint8_t ignition_cut = 0;

int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

uint16_t shift_force_spoof = 0; // Holds desired value of gear shift force

volatile uint8_t gear_fail_nt_shift =
        SHIFT_ENUM_NT; // Stores direction of neutral shift while in gear failure
// mode

// Timing interval variables
volatile uint32_t CAN_recv_tmr = 0;
volatile uint32_t ignition_cut_tmr = 0;
volatile uint32_t gear_status_tmr;
uint32_t temp_samp_tmr, sensor_samp_tmr = 0;
uint32_t diag_send_tmr, state_send_tmr = 0;
uint32_t switch_debounce_tmr = 0;
volatile uint32_t lockout_tmr = 0; // Holds millis value of last lockout set
uint32_t act_tmr = 0; // Records the time the actuator was fired
uint32_t pwr_cut_tmr = 0; // Records when the power cut was initiated
uint32_t pwr_cut_retry_tmr =
        0; // Records when the ADL power cut message was last sent


/**
 * Main function
 */
void main(void)
{
    init_general(); // Set general runtime configuration bits
    init_gpio_pins(); // Set all I/O pins to low outputs
    // init_peripheral_modules(); // Disable unused peripheral modules
    init_oscillator(0); // Initialize oscillator configuration bits
    init_timer2(); // Initialize timer2 (millis)
    init_adc(NULL); // Initialize ADC module
    init_termination(NOT_TERMINATING); // Initialize programmable CAN termination
    init_can(); // Initialize CAN

   
    // Initialize pins
    SHIFT_UP_TRIS = INPUT;
    SHIFT_UP_ANSEL = DIG_INPUT;
    SHIFT_DN_TRIS = INPUT;
    SHIFT_DN_ANSEL = DIG_INPUT;
    SHIFT_NT_TRIS = INPUT;
    SHIFT_NT_ANSEL = DIG_INPUT;
    ACT_UP_TRIS = OUTPUT;
    ACT_UP_LAT = ACT_OFF;
    ACT_DN_TRIS = OUTPUT;
    ACT_DN_LAT = ACT_OFF;
    ADC_GEAR_TRIS = INPUT;
    ADC_GEAR_ANSEL = AN_INPUT;
    ADC_GEAR_CSS = 1;
    ADC_FORCE_TRIS = INPUT;
    ADC_FORCE_ANSEL = AN_INPUT;
    ADC_FORCE_CSS = 1;

    // Trigger initial ADC conversion
    ADCCON3bits.GSWTRG = 1;

    STI(); // Enable interrupts

    // Main loop
    while (1) {
        STI(); // Enable interrupts (in case anything disabled without re-enabling)

        // Debounce switches
        debounce_switches();

        // Analog sampling functions
        sample_temp();
        sample_sensors(NOT_SHIFTING);

        // Do upshift sequence if queued
        if (queue_up > 0) {
            do_shift(SHIFT_ENUM_UP);
        }

        // Do downshift sequence if queued
        if (queue_dn > 0) {
            do_shift(SHIFT_ENUM_DN);
        }

        // Do neutral shift sequence if queued
        if (queue_nt > 0) {
            do_shift(SHIFT_ENUM_NT);
        }

        
        // Send status over CAN
        send_diag_can();
        send_state_can(NO_OVERRIDE);
        send_gear_status_can(NO_OVERRIDE);
        send_ignition_cut_status_can(NO_OVERRIDE);
    }
}

//=============================== INTERRUPTS ===================================

/**
 * CAN1 Interrupt Handler
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL4SRS)))
can_inthnd(void)
{   
    // Process all available CAN messages
    if (C1INTbits.RBIF) {
        CAN_recv_messages(process_CAN_msg); 
    }
    
    // CAN overflow error
    if (C1INTbits.RBOVIF) {
        CAN_rx_ovf++;
    }
    
    // Clear CAN1 Interrupt Flag
    IFS4CLR = _IFS4_CAN1IF_MASK;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 * 
 * Checks for and handles shifting-related inputs.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS)))
timer2_inthnd(void)
{
    // Update counters
    ++millis;
    if (millis % 1000 == 0)
        ++seconds;
    
    // Trigger an ADC conversion, if necessary
    if (ADCCON2bits.EOSRDY)
        ADCCON3bits.GSWTRG = 1; 
    
    // Update can status if someone is shifting
    if (SHIFT_UP_SW != prev_switch_up || SHIFT_DN_SW != prev_switch_dn) {
        send_state_can(OVERRIDE);
    }
    

    // Check to see if GCM mode should change
    update_gcm_mode();

    // Check for an auto upshift if in the correct mode
    if (mode == AUTO_UPSHIFT_MODE) {
        process_auto_upshift();
    }

    // Check for a new shift_up switch press
    if (SHIFT_UP_SW && !prev_switch_up) {
        
        process_upshift_press();
    }
    prev_switch_up = SHIFT_UP_SW;

    // Check for a new shift_dn switch press
    if (SHIFT_DN_SW && !prev_switch_dn) {
        process_downshift_press();
    }
    prev_switch_dn = SHIFT_DN_SW;

    IFS0CLR = _IFS0_T2IF_MASK; // Clear TMR2 Interrupt Flag
}

/**
 * NMI Handler
 *
 * This interrupt handler will reset the device when a clock failure occurs.
 */
void _nmi_handler(void)
{
    // Perform a software reset
    unlock_config();
    RSWRSTSET = 1;
    uint16_t dummy = RSWRST;
    while (1)
        ;
    asm volatile("eret;"); // Should never be called
}

//============================= ADC FUNCTIONS ==================================

/**
 * void sample_temp(void)
 *
 * Samples the PCB temp sensor and internal die temp sensor, then updates
 * variables if the interval has passed.
 */
void sample_temp(void)
{
    if (millis - temp_samp_tmr >= TEMP_SAMP_INTV) {

        /**
         * PCB Temp [C] = (Sample [V] - 0.75 [V]) / 10 [mV/C]
         * PCB Temp [C] = ((3.3 * (pcb_temp_samp / 4095)) [V] - 0.75 [V]) / 0.01
         * [V/C] PCB Temp [C] = (3.3 * (pcb_temp_samp / 40.95)) - 75) [C] PCB Temp
         * [C] = (pcb_temp_samp * 0.080586080586) - 75 [C] PCB Temp [C / 0.005] =
         * 200 * ((pcb_temp_samp * 0.080586080586) - 75) [C / 0.005] PCB Temp [C /
         * 0.005] = (temp_samp * 16.1172161172) - 15000 [C / 0.005]
         */
        uint32_t pcb_temp_samp = read_adc_chn(ADC_PTEMP_CHN);
        pcb_temp = (((double) pcb_temp_samp) * 16.1172161172) - 15000.0;

        /**
         * Junc Temp [C] = 200 [C/V] * (1 [V] - Sample [V])
         * Junc Temp [C] = 200 [C/V] * (1 - (3.3 * (junc_temp_samp / 4095))) [V]
         * Junc Temp [C] = 200 [C/V] * (1 - (junc_temp_samp / 1240.9090909)) [V]
         * Junc Temp [C] = 200 - (junc_temp_samp * 0.161172161172) [C]
         * Junc Temp [C / 0.005] = 40000 - (junc_temp_samp * 32.234432234432) [C /
         * 0.005]
         */

        uint32_t junc_temp_samp = read_adc_chn(ADC_JTEMP_CHN);
        junc_temp =
                (int16_t) (40000.0 - (((double) junc_temp_samp) * 32.234432234432));

        temp_samp_tmr = millis;
    }
}

/**
 * void sample_sensors(uint8_t is_shifting)
 *
 * Samples the gear position sensor and shift force sensor and updates
 * variables if the interval has passed.
 *
 * @param is_shifting- Whether or not the car is currently mid-shift
 */
void sample_sensors(uint8_t is_shifting)
{
    if (millis - sensor_samp_tmr >= SENSOR_SAMP_INTV) {
        uint32_t gear_samp = read_adc_chn(ADC_GEAR_CHN);
        double gear_voltage = ((((double) gear_samp) / 4095.0) * 5.0);
        uint16_t gear_voltage_can = (uint16_t) (gear_voltage * 10000.0);

        if (abs(gear_voltage - GEAR_VOLT_1) <= GEAR_VOLT_RIPPLE) {
            gear = 1;
        } else if (abs(gear_voltage - GEAR_VOLT_NEUT) <= GEAR_VOLT_RIPPLE) {
            gear = GEAR_NEUT;
        } else if (abs(gear_voltage - GEAR_VOLT_2) <= GEAR_VOLT_RIPPLE) {
            gear = 2;
        } else if (abs(gear_voltage - GEAR_VOLT_3) <= GEAR_VOLT_RIPPLE) {
            gear = 3;
        } else if (abs(gear_voltage - GEAR_VOLT_4) <= GEAR_VOLT_RIPPLE) {
            gear = 4;
        } else if (abs(gear_voltage - GEAR_VOLT_5) <= GEAR_VOLT_RIPPLE) {
            gear = 5;
        } else if (abs(gear_voltage - GEAR_VOLT_6) <= GEAR_VOLT_RIPPLE) {
            gear = 6;
        } else {
            gear = GEAR_FAIL;
        }

        // Only send gear failure error message if we aren't shifting
        if (!is_shifting && gear == GEAR_FAIL) {
            send_errno_CAN_msg(GCM_ID, ERR_GCM_GEARFAIL);
        }

        // Sample shift force sensor
        uint32_t force_samp = read_adc_chn(ADC_FORCE_CHN);
        double force_voltage = ((((double) force_samp) / 4095.0) * 5.0);
        shift_force = (force_voltage - 2.5) * 96.347;
        int16_t shift_force_can = (int16_t) (shift_force / FORCE_SCL);

        CAN_data data = {0};
        data.byte0 = gear;
        data.byte1 = mode;
        // data.byte1 = auto_upshift_switch;
        data.halfword1 = gear_voltage_can;
        data.halfword2 = shift_force_can;
        CAN_send_message(GCM_ID + 0x1, 6, data);

        sensor_samp_tmr = millis;
    }
}

//============================= CAN FUNCTIONS ==================================

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg)
{ // Add brake pressure #CHECK
    uint16_t *lsbArray = (uint16_t *) msg.data;

    uint8_t switch_bitmap;
    uint8_t button_bitmap;

    switch (msg.id) {
    case MS6_ID + 0x0:
        eng_rpm =
                ((double) ((msg.data[ENG_RPM_BYTE] << 8) | msg.data[ENG_RPM_BYTE + 1])) *
                ENG_RPM_SCL;
        throttle_pos = ((double) ((msg.data[THROTTLE_POS_BYTE] << 8) |
                msg.data[THROTTLE_POS_BYTE + 1])) *
                THROTTLE_POS_SCL;
        bat_volt = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) |
                msg.data[VOLT_ECU_BYTE + 1])) *
                VOLT_ECU_SCL;
        CAN_recv_tmr = millis;
        break;

        //    case ABS_WS_ID:
        //      wheel_fl_speed = lsbArray[WHEELSPEED_FL_BYTE/2] * WHEELSPEED_FL_SCL;
        //      wheel_fr_speed = lsbArray[WHEELSPEED_FR_BYTE/2] * WHEELSPEED_FR_SCL;
        //      wheel_rl_speed = lsbArray[WHEELSPEED_RL_BYTE/2] * WHEELSPEED_RL_SCL;
        //      wheel_rr_speed = lsbArray[WHEELSPEED_RR_BYTE/2] * WHEELSPEED_RR_SCL;
        //
        //      CAN_recv_tmr = millis;
        //      break;

    case 0x6FF:
        wheel_fl_speed = (msg.data[0]) * WHEELSPEED_FL_SCL;
        wheel_fr_speed = (msg.data[1]) * WHEELSPEED_FR_SCL;
        wheel_rl_speed = (msg.data[2]) * WHEELSPEED_RL_SCL;
        wheel_rr_speed = (msg.data[3]) * WHEELSPEED_RR_SCL;
        auto_upshift_switch = msg.data[4];

        CAN_recv_tmr = millis;
        break;

    case MS6_ID + 0x7:
        shift_force_ecu = (uint16_t) ((msg.data[SHIFT_FORCE_BYTE] << 8) |
                msg.data[SHIFT_FORCE_BYTE + 1]);
        CAN_recv_tmr = millis;
        break;

    case PDM_ID + 0x1:
        switch_bitmap = msg.data[PDM_SWITCH_BYTE];
        kill_sw = switch_bitmap & KILL_PDM_SW_MASK;
        CAN_recv_tmr = millis;
        break;

    case WHEEL_ID + 0x1:
        button_bitmap = msg.data[0];
        radio_button = button_bitmap & 0x01;
        acknowledge_button = (uint8_t) ((button_bitmap & 0x02) >> 1);
        auxiliary_button = ((uint8_t) (button_bitmap & 0x04) >> 2);
        // auto_upshift_switch = ((uint8_t) (button_bitmap & 0x10) >> 4);
        night_day_switch = ((uint8_t) (button_bitmap & 0x80) >> 7);
        CAN_recv_tmr = millis;
        break;

    /*case IMU_FIRST_ID: // IMU to ADL for Motec Traction control
        lat_accel = ((double) ((msg.data[LATERAL_G_BYTE + 1] << 8) |
                msg.data[LATERAL_G_BYTE])) *
                LATERAL_G_SCL +
                LATERAL_G_OFFSET;
        conv_lat_accel =
                (int) abs(lat_accel * 1000); // scalar multiple for high motec accuracy
        uint16_t byte1 = (conv_lat_accel & 0xFF00) >> 8;
        uint16_t byte2 = (conv_lat_accel & 0xFF) << 8;
        conv_lat_accel = byte1 | byte2;
        send_lat_accel();*/
    }
}

/**
 * void send_diag_can(void)
 *
 * Sends the diagnostic CAN message if the interval has passed.
 */
void send_diag_can(void)
{
    if (millis - diag_send_tmr >= DIAG_MSG_SEND) {
        CAN_data data = {0};
        data.halfword0 = (uint16_t) seconds;
        data.halfword1 = pcb_temp;
        data.halfword2 = junc_temp;
       
        CAN_send_message(GCM_ID + 0, 6, data);
        
        diag_send_tmr = millis;
    }
}

/**
 * void send_state_can(uint8_t override)
 *
 * Sends the switch/queue state CAN message if the interval has passed.
 *
 * @param override - Whether to override the interval
 */
void send_state_can(uint8_t override)
{
    if ((millis - state_send_tmr >= STATE_MSG_SEND) || override) {
        CAN_data data = {0};
        data.byte0 = 0x0 | (SHIFT_NT_SW << 2) | (SHIFT_DN_SW << 1) | SHIFT_UP_SW;
        data.byte1 = queue_up;
        data.byte2 = queue_dn;
        data.byte3 = queue_nt;

        CAN_send_message(GCM_ID + 2, 4, data);
        
        state_send_tmr = millis;
    }
}

//=========================== LOGIC FUNCTIONS ==================================

/**
 * void check_gcm_mode(void)
 *
 * Check various CAN data to determine the correct GCM mode
 */
void update_gcm_mode(void)
{   
    // enter auto-upshift mode
    if (auto_upshift_switch >= 2 && mode != AUTO_UPSHIFT_MODE) {// && wheel_fl_speed > 10 && wheel_fr_speed > 10) { 
        mode = AUTO_UPSHIFT_MODE; 
        queue_up = 0;
        queue_dn = 0;
    }
    if (auto_upshift_switch < 2 && mode == AUTO_UPSHIFT_MODE) {
            mode = NORMAL_MODE;
            queue_up = 0;
            queue_dn = 0;
    }
       
    /*
        // engage auto-upshifting
    } else if (night_day_switch == 0) { // if auto-upshifting is engaged and
        // dead-man switch is not pressed

        mode = NORMAL_MODE; // disengage auto-upshifting
        queue_up = 0; // remove queued upshift
    } */
    
    
    if (mode == AUTO_UPSHIFT_MODE) {
        
        if(kill_sw ) {
            mode = NORMAL_MODE;
            queue_up = 0;
            queue_dn = 0;
        }
        
 
        /*
      if (throttle_pos_passed_min_auto == 0) { // check to see if throttle position
      did not cross minimum while in auto-upshifting mode if (throttle_pos > 75) {
      // check if throttle position crossed minimum throttle_pos_passed_min_auto =
      1; // throttle position crossed minimum while in auto-upshifting mode
        }
      } else if (throttle_pos <= 75) { // check to see if throttle position has
      dropped below minimum while in auto-upshifting mode mode = NORMAL_MODE; //
      disengage auto-upshifting queue_up = 0; // remove queued upshift
      }
         */
    }
}

// Add auto-upshift logic #CHECK

/**
 * void process_auto_upshift(void)
 *
 * Check to see whether an automatic upshift should take place,
 * and increment the queue if neccesary
 */
volatile uint32_t auto_upshift_rpm_tmr = 0;
void process_auto_upshift(void)
{
    // float speed = (wheel_fl_speed + wheel_fr_speed) / 2.0;
    float speed = wheel_fr_speed;
    if (eng_rpm >= get_threshold_rpm(gear) &&  // RPM threshold
        speed >= get_threshold_speed(gear) && // Speed threshold
        queue_up == 0 && // Not already shifting
        gear < MAX_AUTO_GEAR // Not trying to get past 6th
        ) { 
        queue_up = 1;
        // auto_upshift_rpm_tmr = millis;
    }
}

/**
 * void process_upshift_press(void)
 *
 * After registering an upshift press, determine if we should increment the
 * queue
 */
void process_upshift_press(void)
{
    /*if (mode == AUTO_UPSHIFT_MODE) {
        mode = NORMAL_MODE;
        queue_up = 0;
        auto_upshift_disable_override = 1;
    }*/
    
    // If CAN is not stale (message received in past 10 seconds),
    // and RPM is less than 8000, don't do anything.
    if (millis - CAN_recv_tmr < 10 * 1000 && eng_rpm < 8000) {
        return;
    }

    if (queue_nt == 1) {
        return;
    }

    if (millis - lockout_tmr < LOCKOUT_DUR) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_LOCKOUT);
        return;
    }

    if (SHIFT_NT_SW) { // Pressed while holding the neutral button
        if (gear == GEAR_NEUT) {
            queue_dn = 1;
            queue_nt = queue_up = 0;
        } else if (gear == 1 || gear == 2) {
            queue_nt = 1;
            queue_up = queue_dn = 0;
        } else if (gear == GEAR_FAIL) {
            queue_nt = 1;
            queue_up = queue_dn = 0;
            gear_fail_nt_shift = SHIFT_ENUM_UP;
        } else {
            queue_dn = 1;
        }
        lockout_tmr = millis;
        send_state_can(OVERRIDE); // Send new queue values on CAN
    } else { // Pressed while not holding the neutral button
        if (gear == GEAR_NEUT) {
            queue_up = 1;
            queue_dn = queue_nt = 0;
            lockout_tmr = millis;
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        if (queue_dn > 0 || queue_nt > 0) {
            queue_dn = queue_up = queue_nt = 0;
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        if (gear == GEAR_FAIL || queue_up < 6 - gear) {
            queue_up++;
            lockout_tmr = millis;
            send_state_can(OVERRIDE); // Send new queue values on CAN
        }
    }
}

/**
 * void process_downshift_press(void)
 *
 * After registering an downshift press, determine if we should increment the
 * queue
 */
void process_downshift_press(void)
{
    /*if (mode == AUTO_UPSHIFT_MODE) {
        mode = NORMAL_MODE;
        queue_up = 0;
        auto_upshift_disable_override = 1;
    }*/

    if (queue_nt == 1) {
        return;
    }

    if (millis - lockout_tmr < LOCKOUT_DUR) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_LOCKOUT);
        return;
    }

    if (SHIFT_NT_SW) { // Pressed while holding the neutral button
        if (gear == GEAR_NEUT) {
            queue_nt = 0;
        } else if (gear == 1 || gear == 2) {
            queue_nt = 1;
            queue_up = queue_dn = 0;
        } else if (gear == GEAR_FAIL) {
            queue_nt = 1;
            queue_up = queue_dn = 0;
            gear_fail_nt_shift = SHIFT_ENUM_DN;
        } else {
            queue_dn = 1;
            queue_up = queue_nt = 0;
        }
        lockout_tmr = millis;
        send_state_can(OVERRIDE); // Send new queue values on CAN
    } else { // Pressed while not holding the neutral button
        if (queue_up > 0 || queue_nt > 0) {
            queue_dn = queue_up = queue_nt = 0;
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        if (gear == GEAR_FAIL || gear == GEAR_NEUT || queue_dn + 1 < gear) {
            queue_dn++;
            lockout_tmr = millis;
            send_state_can(OVERRIDE); // Send new queue values on CAN
        }
    }
}

/**
 * uint8_t check_shift_conditions(uint8_t shift_enum)
 *
 * Checks various conditions and determines if the requested shift is allowable.
 *
 * @param shift_enum - The type of shift being requested
 * @return 0 if restricted, 1 if allowable
 */
uint8_t check_shift_conditions(uint8_t shift_enum)
{
    return 1;
    // Prevent shifting past 1st gear
    if (gear == 1 && shift_enum == SHIFT_ENUM_DN) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_SHIFTPAST);
        return 0;
    }

    // Prevent shifting past 6th gear
    if (gear == 6 && shift_enum == SHIFT_ENUM_UP) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_SHIFTPAST);
        return 0;
    }

    return 1;

    // Prevent shifting into neutral from anything other than 1st or 2nd
    if (shift_enum == SHIFT_ENUM_NT &&
            !(gear == 1 || gear == 2 || gear == GEAR_FAIL)) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_BADNEUT);
        return 0;
    }

    // Only perform these checks if CAN data isn't stale
    uint8_t stale_CAN = (millis - CAN_recv_tmr >= CAN_STATE_WAIT);
    if (!stale_CAN) {
        // Prevent shifting while the kill switch is pressed
        if (kill_sw) {
            send_errno_CAN_msg(GCM_ID, ERR_GCM_KILLSW);
            return 0;
        }

        // Prevent shifting from neutral into gear at high rpm
        if (gear == GEAR_NEUT && eng_rpm >= RPM_NEUT_LOCK) {
            send_errno_CAN_msg(GCM_ID, ERR_GCM_NEUTLOCK);
            return 0;
        }

        // Check for over/under rev
        if (0 && shift_enum != SHIFT_ENUM_NT && ENG_ON && gear != GEAR_NEUT &&
                gear != GEAR_FAIL) {
            double output_speed = eng_rpm / gear_ratio[gear];

            if (shift_enum == SHIFT_ENUM_UP && gear != 6) {
                double new_rpm = output_speed * gear_ratio[gear + 1];
                if (new_rpm <= RPM_UNDERREV) {
                    send_errno_CAN_msg(GCM_ID, ERR_GCM_UNDERREV);
                    return 0;
                }
            } else if (shift_enum == SHIFT_ENUM_DN && gear != 1) {
                double new_rpm = output_speed * gear_ratio[gear - 1];
                if (new_rpm >= RPM_OVERREV) {
                    send_errno_CAN_msg(GCM_ID, ERR_GCM_OVERREV);
                    return 0;
                }
            }
        }
    } else if (!COMP) {
        // Prevent shifting if CAN state variables aren't updated
        send_errno_CAN_msg(GCM_ID, ERR_GCM_NOCAN);
        return 0;
    }

    // Only perform these checks if we aren't at competition and have good CAN
    // data
    if (!COMP && !stale_CAN) {
        // Prevent shifting up past neutral on low voltage
        if (gear == 1 && shift_enum == SHIFT_ENUM_UP) {
            if (bat_volt != 0.0 && bat_volt < LOW_VOLT) {
                send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
                return 0;
            }
        }

        // Prevent shifting down past neutral on low voltage
        if (gear == 2 && shift_enum == SHIFT_ENUM_DN) {
            if (bat_volt != 0 && bat_volt < LOW_VOLT) {
                send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
                return 0;
            }
        }

        // Prevent any shifting on very low voltage
        if (bat_volt != 0 && bat_volt < LOWER_VOLT) {
            send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWERVOLT);
            return 0;
        }
    }

    return 1;
}

/**
 * void do_shift(uint8_t shift_enum)
 *
 * Performs the requested shift sequence.
 *
 * @param shift_enum The type of requested shift
 */
void do_shift(uint8_t shift_enum)
{
    const uint8_t SHIFT_UP = shift_enum == SHIFT_ENUM_UP;
    const uint8_t SHIFT_DN = shift_enum == SHIFT_ENUM_DN;
    const uint8_t SHIFT_NT = shift_enum == SHIFT_ENUM_NT;

    uint8_t orig_gear = gear;
    uint8_t gear_target;

    // Fall back to gear sensor failure mode if necessary
    if (gear == GEAR_FAIL) {
        do_shift_gear_fail(shift_enum);
        return;
    }

    // Store the target gear
    if (SHIFT_UP) {
        gear_target = (gear == GEAR_NEUT) ? 2 : gear + 1;
    } else if (SHIFT_DN) {
        gear_target = (gear == GEAR_NEUT) ? 1 : gear - 1;
    } else if (SHIFT_NT) {
        gear_target = GEAR_NEUT;
    }

    if (SHIFT_UP || SHIFT_DN) {
        /**
         * Perform an upshift/downshift
         */

        if (!check_shift_conditions(shift_enum)) {
            switch (shift_enum) {
            case SHIFT_ENUM_UP:
                queue_up = 0;
                retry_up = 0;
                break;
            case SHIFT_ENUM_DN:
                queue_dn = 0;
                retry_dn = 0;
                break;
            }
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        // Fire actuator
        if (SHIFT_UP) {
            ACT_UP_LAT = ACT_ON;
        } else if (SHIFT_DN) {
            ACT_DN_LAT = ACT_ON;
        }
        act_tmr = millis;

        // Wait a bit and signal the ECU to cut power
        //while (millis - act_tmr < PWR_CUT_WAIT) {
        //    main_loop_misc();
        //}
        
        if (SHIFT_UP) {
            set_ignition_cut(IGNITION_CUT_UPSHIFT, IGNITION_CUT_ENABLE);
        } else if (SHIFT_DN) {
            set_ignition_cut(IGNITION_CUT_DOWNSHIFT, IGNITION_CUT_ENABLE);
        }
        send_ignition_cut_status_can(OVERRIDE);
        //send_power_cut(CUT_START);
        //send_ignition_cut();

        while (1) {
            sample_sensors(SHIFTING);

            if (gear == gear_target) {
                // Relax actuator
                if (SHIFT_UP) {
                    ACT_UP_LAT = ACT_OFF;
                    retry_up = 0;
                } else if (SHIFT_DN) {
                    ACT_DN_LAT = ACT_OFF;
                    retry_dn = 0;
                }

                set_ignition_cut(IGNITION_CUT_ALL, IGNITION_CUT_DISABLE);
                send_ignition_cut_status_can(OVERRIDE);
                send_gear_status_can(OVERRIDE);
                //send_power_cut(CUT_END);
                //send_ignition_cut();

                relax_wait();

                // Decrement queued shifts value
                if (SHIFT_UP) {
                    queue_up--;
                } else if (SHIFT_DN) {
                    queue_dn--;
                }
                send_state_can(OVERRIDE); // Send new queue values on CAN

                return;
            }

            if (millis - act_tmr >= MAX_SHIFT_DUR) {
                // Relax actuator
                if (SHIFT_UP) {
                    ACT_UP_LAT = ACT_OFF;
                    retry_up++;
                } else if (SHIFT_DN) {
                    ACT_DN_LAT = ACT_OFF;
                    retry_dn++;
                }

                set_ignition_cut(IGNITION_CUT_ALL, IGNITION_CUT_DISABLE);
                send_ignition_cut_status_can(OVERRIDE);

                relax_wait();

                send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXDUR);

                if (SHIFT_UP && retry_up > MAX_RETRY) {
                    queue_up = 0;
                    retry_up = 0;
                    send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
                    send_state_can(OVERRIDE); // Send new queue values on CAN
                } else if (SHIFT_DN && retry_dn > MAX_RETRY) {
                    queue_dn = 0;
                    retry_dn = 0;
                    send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
                    send_state_can(OVERRIDE); // Send new queue values on CAN
                }

                return;
            }

            // Also do main loop functions during "busy" loop
            main_loop_misc();
        }
    } else if (SHIFT_NT) {
        /**
         * Perform a neutral shift
         */

        if (!check_shift_conditions(shift_enum)) {
            queue_nt = 0;
            retry_nt = 0;
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        // Fire actuator
        if (orig_gear == 1) {
            ACT_UP_LAT = ACT_ON;
        } else if (orig_gear == 2) {
            ACT_DN_LAT = ACT_ON;
        }
        act_tmr = millis;

        while (1) {
            sample_sensors(SHIFTING);

            if (gear == GEAR_NEUT) {
                // Relax actuator
                ACT_DN_LAT = ACT_OFF;
                ACT_UP_LAT = ACT_OFF;
                relax_wait();

                retry_nt = 0;
                queue_nt = 0;
                send_state_can(OVERRIDE); // Send new queue values on CAN
                return;
            } else if (gear == GEAR_FAIL || gear == orig_gear) {
                if (millis - act_tmr >= MAX_SHIFT_DUR) {
                    // Relax actuator
                    ACT_DN_LAT = ACT_OFF;
                    ACT_UP_LAT = ACT_OFF;
                    relax_wait();

                    send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXDUR);

                    retry_nt++;
                    break;
                }
            } else {
                // Relax actuator
                ACT_DN_LAT = ACT_OFF;
                ACT_UP_LAT = ACT_OFF;
                relax_wait();

                send_errno_CAN_msg(GCM_ID, ERR_GCM_MISSNEUT);

                retry_nt++;
                break;
            }

            // Do main loop functions while waiting
            main_loop_misc();
        }

        if (retry_nt > MAX_RETRY) {
            queue_nt = 0;
            retry_nt = 0;
            send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
            send_state_can(OVERRIDE); // Send new queue values on CAN
        }
    }
}

/**
 * void do_shift_gear_fail(uint8_t shift_enum)
 *
 * Performs the requested shift sequence in gear sensor
 * failure mode.
 *
 * TODO: Combine this with normal do_shift function?
 *
 * @param shift_enum The type of requested shift
 */
void do_shift_gear_fail(uint8_t shift_enum)
{
    const uint8_t SHIFT_UP = shift_enum == SHIFT_ENUM_UP;
    const uint8_t SHIFT_DN = shift_enum == SHIFT_ENUM_DN;
    const uint8_t SHIFT_NT = shift_enum == SHIFT_ENUM_NT;

    if (SHIFT_UP || SHIFT_DN) {
        /**
         * Perform an upshift/downshift
         */

        if (!check_shift_conditions(shift_enum)) {
            switch (shift_enum) {
            case SHIFT_ENUM_UP:
                queue_up = 0;
                retry_up = 0;
                break;
            case SHIFT_ENUM_DN:
                queue_dn = 0;
                retry_dn = 0;
                break;
            }
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        // Fire actuator
        if (SHIFT_UP) {
            ACT_UP_LAT = ACT_ON;
        } else if (SHIFT_DN) {
            ACT_DN_LAT = ACT_ON;
        }
        act_tmr = millis;

        // Wait a bit and signal the ECU to cut power
        while (millis - act_tmr < PWR_CUT_WAIT) {
            main_loop_misc();
        }

        //if (SHIFT_UP)
        //  send_power_cut(CUT_START);
        //send_ignition_cut();

        while (1) {
            if ((SHIFT_UP && millis - act_tmr >= UP_SHIFT_DUR) ||
                    (SHIFT_DN && millis - act_tmr >= DN_SHIFT_DUR)) {
                // Relax actuator
                if (SHIFT_UP) {
                    ACT_UP_LAT = ACT_OFF;
                    retry_up = 0;
                } else if (SHIFT_DN) {
                    ACT_DN_LAT = ACT_OFF;
                    retry_dn = 0;
                }

                // send_power_cut(CUT_END);
                //send_ignition_cut();

                relax_wait();

                // Decrement queued shifts value
                if (SHIFT_UP) {
                    queue_up--;
                } else if (SHIFT_DN) {
                    queue_dn--;
                }
                send_state_can(OVERRIDE); // Send new queue values on CAN

                return;
            }

            // Also do main loop functions during "busy" loop
            main_loop_misc();
        }
    } else if (SHIFT_NT) {
        /**
         * Perform a neutral shift
         */

        if (!check_shift_conditions(shift_enum)) {
            queue_nt = 0;
            retry_nt = 0;
            send_state_can(OVERRIDE); // Send new queue values on CAN
            return;
        }

        // Fire actuator
        if (gear_fail_nt_shift == SHIFT_ENUM_UP) {
            ACT_UP_LAT = ACT_ON;
        } else if (gear_fail_nt_shift == SHIFT_ENUM_DN) {
            ACT_DN_LAT = ACT_ON;
        }
        act_tmr = millis;

        while (1) {
            if (millis - act_tmr >= NT_SHIFT_DUR) {
                // Relax actuator
                ACT_DN_LAT = ACT_OFF;
                ACT_UP_LAT = ACT_OFF;
                relax_wait();

                retry_nt = 0;
                queue_nt = 0;
                send_state_can(OVERRIDE); // Send new queue values on CAN
                return;
            }

            // Do main loop functions while waiting
            main_loop_misc();
        }
    }
}

//========================== UTILITY FUNCTIONS =================================

/**
 * void relax_wait(void)
 *
 * Waits for the actuator to return to the relaxed position
 */
void relax_wait(void)
{
    // Wait for RELAX_WAIT millis and do main loop functions in the process
    uint32_t relax_wait_tmr = millis;
    while (millis - relax_wait_tmr < RELAX_WAIT) {
        main_loop_misc();
    }
}

/**
 * void main_loop_misc(void)
 *
 * Do non-shift logic main-loop functions. Typically run
 * while waiting in a blocking section of the shifting logic.
 */
void main_loop_misc(void)
{
    debounce_switches();
    
    // Sample sensors
    sample_temp();
    sample_sensors(SHIFTING);
    
    // Send CAN messages
    send_diag_can();
    send_state_can(NO_OVERRIDE);
    send_gear_status_can(NO_OVERRIDE);
    send_ignition_cut_status_can(NO_OVERRIDE);
}

void send_ignition_cut_status_can(uint8_t override)
{
    if (override || millis - ignition_cut_tmr >= IGNITION_CUT_CAN_SEND) {
        CAN_data data = {0};
        data.doubleword = (ignition_cut_upshift << 1) | ignition_cut_downshift;
        //data.doubleword = 0b11;
        CAN_send_message(GCM_BOSCH_GEARCUT_ID, 8, data);
        ignition_cut_tmr = millis;
    }
}

void send_gear_status_can(uint8_t override)
{
    if (millis % 2 == 0) {
        CAN_data data = {0};
        int gear_can = (gear + 1.5)/0.5;
        data.byte0 = gear_can;
        CAN_send_message(GCM_BOSCH_GEARINFO_ID, 8, data);
        // gear_status_tmr = millis;
   }
}


/**
 * void debounce_switches(void)
 *
 * Keeps track of switch state in order to debounce switch inputs. The inputs
 * are debounced collectively, rather than debouncing each individual switch
 * input separately.
 */
void debounce_switches(void)
{
    uint8_t switch_raw =
            0x0 | SHIFT_UP_RAW << 2 | SHIFT_DN_RAW << 1 | SHIFT_NT_RAW << 0;

    if (switch_raw != switch_prev) {
        switch_debounce_tmr = millis;
    } else if (millis - switch_debounce_tmr >= DEBOUNCE_WAIT) {
        switch_debounced = switch_raw;
    }

    switch_prev = switch_raw;
}

/**
 * uint16_t get_threshold_rpm(uint8_t gear)
 *
 * Returns the optimal shift RPM for a given gear
 */
uint16_t get_threshold_rpm(uint8_t gear)
{

    if (gear == GEAR_FAIL) {
        return 20000; // If invalid gear, return threshold above redline
    }
    return shift_rpm[gear == GEAR_NEUT ? gear : gear - 1];
}

float get_threshold_speed(uint8_t gear) {
    if (gear == GEAR_FAIL) {
        return 30.0;
    }
    return shift_speed_min[gear == GEAR_NEUT ? gear : gear -1];
}

/**
 * uint8_t is_in_launch(void)
 *
 * returns 1 if the car is currently in a launch, 0 otherwise
 */
uint8_t is_in_launch(void)
{
    double front_ws = (wheel_fl_speed + wheel_fr_speed)/2.0;
    // double rear_ws = (wheel_rl_speed + wheel_rr_speed)/ 2.0;

    return front_ws < 40;
}

void send_lat_accel(void)
{
    CAN_data data = {0};
    data.halfword0 = ADL_IDX_7_9;
    data.halfword1 = conv_lat_accel;
    CAN_send_message(ADL_ID, 4, data);
}

void set_ignition_cut(uint8_t type, uint8_t status)
{
    if (type == IGNITION_CUT_ALL || type == IGNITION_CUT_UPSHIFT)
        ignition_cut_upshift = status;
    if (type == IGNITION_CUT_ALL || type == IGNITION_CUT_DOWNSHIFT)
        ignition_cut_downshift = status;
}