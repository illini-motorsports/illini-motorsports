
/**
 * FSAE Library CAN Defines
 *
 * Author:      George Schwieters
 * Created:     2014-2015
 */
#ifndef CAN_H
#define CAN_H

/**
 * CAN message IDs
 */
#define ERROR_ID            0x00F
#define ECU_ID              0x020
#define SPM_ID              0x050
#define MOTEC_ID            0x100
#define GCM_ID              0x200
#define SGH_ID              0x203
#define LOGGER_ID           0x300
#define WHEEL_ID            0x400
#define ADL_ID              0x500
#define PDM_ID              0x600
#define PDM_CONFIG_ID       0x620

#define BEACON_ID           0x0E5
#define TIRE_TEMP_FL_ID     0x001
#define TIRE_TEMP_FR_ID     0x002
#define TIRE_TEMP_RL_ID     0x003
#define TIRE_TEMP_RR_ID     0x004
#define IMU_FIRST_ID        0x070
#define IMU_SECOND_ID       0x080

/**
 * Byte position of channels in their CAN messages
 */

// From Diagnostic Message
#define UPTIME_BYTE         0
#define PCB_TEMP_BYTE       2
#define IC_TEMP_BYTE        4

// From Error Message
#define ORIGIN_BYTE         0
#define ERRNO_BYTE          2

// From Rear AnalogHub
#define SPRL_BYTE           0
#define SPRR_BYTE           2
#define EOS_BYTE            4
#define BCD_BYTE            6
#define CTRI_BYTE           0
#define CTRO_BYTE           2
#define CTSP_BYTE           4
#define CPSP_BYTE           0
#define MCD_BYTE            2

// From Front AnalogHub
#define SPFL_BYTE           0
#define SPFR_BYTE           2
#define BPF_BYTE            4
#define BPR_BYTE            6
#define STRP_BYTE           0
#define APPS0_BYTE          2
#define APPS1_BYTE          4
#define PTDP_BYTE           6

// From MoTeC
#define ENG_RPM_BYTE        0
#define THROTTLE_POS_BYTE   2
#define LAMBDA_BYTE         4
#define VOLT_ECU_BYTE       6

#define ENG_TEMP_BYTE       0
#define OIL_TEMP_BYTE       2
#define MANIFOLD_TEMP_BYTE  4
#define FUEL_TEMP_BYTE      6

#define AMBIENT_PRES_BYTE   0
#define OIL_PRES_BYTE       2
#define MANIFOLD_PRES_BYTE  4
#define FUEL_PRES_BYTE      6

#define WHEELSPEED_FL_BYTE  0
#define WHEELSPEED_FR_BYTE  2
#define WHEELSPEED_RL_BYTE  4
#define WHEELSPEED_RR_BYTE  6

#define DRIVE_SPEED_BYTE    0
#define GROUND_SPEED_BYTE   2
#define GPS_SPEED_BYTE      4
#define GPS_ALT_BYTE        6

#define GPS_LAT_BYTE        0
#define GPS_LONG_BYTE       4

#define GPS_TIME_BYTE       0
#define RUN_TIME_BYTE       4
#define FUEL_USED_BYTE      6

#define FUEL_INJ_DUTY_BYTE  0
#define FUEL_TRIM_BYTE      2
#define SHIFT_FORCE_BYTE    4
#define AIR_TEMP_BYTE       4

// From GCM
#define GEAR_BYTE           0
#define MODE_BYTE           1
#define GEAR_VOLT_BYTE      2
#define FORCE_BYTE          4

#define GCM_SWITCH_BYTE     0
#define QUEUE_UP_BYTE       1
#define QUEUE_DN_BYTE       2
#define QUEUE_NT_BYTE       3

// From Wheel
#define SWITCH_BITS_BYTE    0
#define BUTTON_BITS_BYTE    0
#define ROT1_BYTE           1
#define ROT2_BYTE           1
#define ROT3_BYTE           2

// From PDM Programming
#define LOAD_IDX_BYTE       0
#define PEAK_MODE_BYTE      1
#define CUTOFF_SETTING_BYTE 2

// From PDM
#define PDM_UPTIME_BYTE     0
#define PDM_PCB_TEMP_BYTE   2
#define PDM_IC_TEMP_BYTE    4

#define LOAD_ENABLITY_BYTE  0
#define LOAD_PEAK_BYTE      2
#define TOTAL_CURRENT_BYTE  4
#define PDM_SWITCH_BYTE     6
#define PDM_FLAG_BYTE       7

#define VBAT_RAIL_BYTE      0
#define V12_RAIL_BYTE       2
#define V5_RAIL_BYTE        4
#define V3V3_RAIL_BYTE      6

#define FUEL_DRAW_BYTE      0
#define IGN_DRAW_BYTE       2
#define INJ_DRAW_BYTE       4
#define ABS_DRAW_BYTE       6

#define PDLU_DRAW_BYTE      0
#define PDLD_DRAW_BYTE      2
#define FAN_DRAW_BYTE       4
#define WTR_DRAW_BYTE       6

#define ECU_DRAW_BYTE       0
#define AUX_DRAW_BYTE       2
#define BVBAT_DRAW_BYTE     4
#define STR_DRAW_BYTE       6

#define FUEL_CUT_BYTE       0
#define IGN_CUT_BYTE        2
#define INJ_CUT_BYTE        4
#define ABS_CUT_BYTE        6

#define PDLU_CUT_BYTE       0
#define PDLD_CUT_BYTE       2
#define FAN_CUT_BYTE        4
#define WTR_CUT_BYTE        6

#define ECU_CUT_BYTE        0
#define AUX_CUT_BYTE        2
#define BVBAT_CUT_BYTE      4

#define FUEL_CUT_P_BYTE     0
#define FAN_CUT_P_BYTE      2
#define WTR_CUT_P_BYTE      4
#define ECU_CUT_P_BYTE      6

#define FUEL_OC_COUNT_BYTE  0
#define IGN_OC_COUNT_BYTE   1
#define INJ_OC_COUNT_BYTE   2
#define ABS_OC_COUNT_BYTE   3
#define PDLU_OC_COUNT_BYTE  4
#define PDLD_OC_COUNT_BYTE  5
#define FAN_OC_COUNT_BYTE   6
#define WTR_OC_COUNT_BYTE   7

#define ECU_OC_COUNT_BYTE   0
#define AUX_OC_COUNT_BYTE   1
#define BVBAT_OC_COUNT_BYTE 2
#define STR_OC_COUNT_BYTE   3

// From Tire Temp Sensors
#define TIRE_TEMP_1_BYTE    0
#define TIRE_TEMP_2_BYTE    2
#define TIRE_TEMP_3_BYTE    4
#define TIRE_TEMP_4_BYTE    6

#define ANALOG_CHAN_0_BYTE    0
#define ANALOG_CHAN_1_BYTE    2
#define ANALOG_CHAN_2_BYTE    4
#define ANALOG_CHAN_3_BYTE    6
#define ANALOG_CHAN_4_BYTE    0
#define ANALOG_CHAN_5_BYTE    2
#define ANALOG_CHAN_6_BYTE    4
#define ANALOG_CHAN_7_BYTE    6
#define ANALOG_CHAN_8_BYTE    0
#define ANALOG_CHAN_9_BYTE    2
#define ANALOG_CHAN_10_BYTE   4
#define ANALOG_CHAN_11_BYTE   6
#define ANALOG_CHAN_12_BYTE   0
#define ANALOG_CHAN_13_BYTE   2
#define ANALOG_CHAN_14_BYTE   4
#define ANALOG_CHAN_15_BYTE   6
#define ANALOG_CHAN_16_BYTE   0
#define ANALOG_CHAN_17_BYTE   2
#define ANALOG_CHAN_18_BYTE   4
#define ANALOG_CHAN_19_BYTE   6
#define ANALOG_CHAN_20_BYTE   0
#define ANALOG_CHAN_21_BYTE   2
#define ANALOG_CHAN_22_BYTE   4
#define ANALOG_CHAN_23_BYTE   6
#define ANALOG_CHAN_24_BYTE   0
#define ANALOG_CHAN_25_BYTE   2
#define ANALOG_CHAN_26_BYTE   4
#define ANALOG_CHAN_27_BYTE   6
#define ANALOG_CHAN_28_BYTE   0
#define ANALOG_CHAN_29_BYTE   2
#define ANALOG_CHAN_30_BYTE   4
#define ANALOG_CHAN_31_BYTE   6
#define ANALOG_CHAN_32_BYTE   0
#define ANALOG_CHAN_33_BYTE   2
#define ANALOG_CHAN_34_BYTE   4
#define ANALOG_CHAN_35_BYTE   6
<<<<<<< Updated upstream
#define ANALOG_SGH_RAW_BYTE   0
#define STRAIN_CALC_SCL_BYTE  2
=======
#define ANALOG_SGH1_RAW_BYTE  0
#define STRAIN1_CALC_BYTE     2
#define ANALOG_SGH2_RAW_BYTE  4
#define STRAIN2_CALC_BYTE     6
>>>>>>> Stashed changes

#define TCOUPLE_0_BYTE        0
#define TCOUPLE_1_BYTE        2
#define TCOUPLE_2_BYTE        4
#define TCOUPLE_3_BYTE        6

#define TCOUPLE_4_BYTE        0
#define TCOUPLE_5_BYTE        2
#define AVG_JUNCT_TEMP_BYTE   4
#define TCOUPLE_FAULT_BYTE    6

#define DIGITAL_INPUT_BYTE    0
#define FREQ_COUNT_0_BYTE     2
#define FREQ_COUNT_1_BYTE     4
#define FREQ_COUNT_2_BYTE     6

#define PGA_SETTINGS_BYTE     0
#define FREQ_SETTINGS_BYTE    2

//from IMU sensor
#define YAW_ACCEL_BYTE        0
#define YAW_RATE_BYTE         0
#define LATERAL_G_BYTE        4
#define LONGITUDINAL_G_BYTE   4

/**
 * Scalars for channels
 */

// From Diagnostic Message
#define UPTIME_SCL          1
#define PCB_TEMP_SCL        0.005
#define IC_TEMP_SCL         0.005

// From Analog Hubs
#define SUS_POT_SCL         0.025
#define BRK_PRS_SCL         0.005
#define CUR_DRAW_SCL        0.01
#define PTDP_SCL            0.0005
#define APPS_SCL            0.01
#define STRP_SCL            0.01
#define CPSP_SCL            0.001
#define TEMP_SCL            0.01

// From Motec
#define ENG_RPM_SCL         1.0
#define THROTTLE_POS_SCL    0.1
#define LAMBDA_SCL          0.001
#define VOLT_ECU_SCL        0.01

#define ENG_TEMP_SCL        0.1
#define OIL_TEMP_SCL        0.1
#define MANIFOLD_TEMP_SCL   0.1
#define FUEL_TEMP_SCL       0.1

#define AMBIENT_PRES_SCL    0.001
#define OIL_PRES_SCL        0.001
#define MANIFOLD_PRES_SCL   0.001
#define FUEL_PRES_SCL       0.001

#define WHEELSPEED_FL_SCL   0.1
#define WHEELSPEED_FR_SCL   0.1
#define WHEELSPEED_RL_SCL   0.1
#define WHEELSPEED_RR_SCL   0.1

#define DRIVE_SPEED_SCL     0.1
#define GROUND_SPEED_SCL    0.1
#define GPS_SPEED_SCL       0.1
#define GPS_ALT_SCL         0.1 //??

#define GPS_LAT_SCL         0.0000001
#define GPS_LONG_SCL        0.0000001

#define GPS_TIME_SCL        1 //??
#define RUN_TIME_SCL        0.1
#define FUEL_USED_SCL       0.1 //??

#define FUEL_INJ_DUTY_SCL   0.1
#define FUEL_TRIM_SCL       0.1 //??
#define SHIFT_FORCE_SCL     1.0 //??
#define AIR_TEMP_SCL        1.0 //??

// From GCM
#define GEAR_SCL           1
#define GEAR_VOLT_SCL      0.0001
#define FORCE_SCL          0.05

#define QUEUE_UP_SCL       1
#define QUEUE_DN_SCL       1
#define QUEUE_NT_SCL       1

// From PDM 
#define TOTAL_CURRENT_SCL 0.01

#define VOLT_RAIL_SCL     0.001
#define LOAD_DRAW_SCL     0.001
#define STR_DRW_SCL       0.01
#define LOAD_CUT_SCL      0.0025

#define VBAT_RAIL_SCL     0.001
#define V12_RAIL_SCL      0.001
#define V5_RAIL_SCL       0.001
#define V3V3_RAIL_SCL     0.001

#define FUEL_DRAW_SCL     0.001
#define IGN_DRAW_SCL      0.001
#define INJ_DRAW_SCL      0.001
#define ABS_DRAW_SCL      0.001

#define PDLU_DRAW_SCL     0.001
#define PDLD_DRAW_SCL     0.001
#define FAN_DRAW_SCL      0.001
#define WTR_DRAW_SCL      0.001

#define ECU_DRAW_SCL      0.001
#define AUX_DRAW_SCL      0.001
#define BVBAT_DRAW_SCL    0.001
#define STR_DRAW_SCL      0.01

#define FUEL_CUT_SCL      0.0025
#define IGN_CUT_SCL       0.0025
#define INJ_CUT_SCL       0.0025
#define ABS_CUT_SCL       0.0025

#define PDLU_CUT_SCL      0.0025
#define PDLD_CUT_SCL      0.0025
#define WTR_CUT_SCL       0.0025
#define FAN_CUT_SCL       0.0025

#define ECU_CUT_SCL       0.0025
#define AUX_CUT_SCL       0.0025
#define BVBAT_CUT_SCL     0.0025

#define FUEL_CUT_P_SCL    0.0025
#define FAN_CUT_P_SCL     0.0025
#define WTR_CUT_P_SCL     0.0025
#define ECU_CUT_P_SCL     0.0025

// From Tire Temp Sensors
#define TIRE_TEMP_SCL       0.1

#define ANALOG_CHAN_0_SCL     0.001
#define ANALOG_CHAN_1_SCL     0.001
#define ANALOG_CHAN_2_SCL     0.001
#define ANALOG_CHAN_3_SCL     0.001
#define ANALOG_CHAN_4_SCL     0.001
#define ANALOG_CHAN_5_SCL     0.001
#define ANALOG_CHAN_6_SCL     0.001
#define ANALOG_CHAN_7_SCL     0.001
#define ANALOG_CHAN_8_SCL     0.001
#define ANALOG_CHAN_9_SCL     0.001
#define ANALOG_CHAN_10_SCL    0.001
#define ANALOG_CHAN_11_SCL    0.001
#define ANALOG_CHAN_12_SCL    0.001
#define ANALOG_CHAN_13_SCL    0.001
#define ANALOG_CHAN_14_SCL    0.001
#define ANALOG_CHAN_15_SCL    0.001
#define ANALOG_CHAN_16_SCL    0.001
#define ANALOG_CHAN_17_SCL    0.001
#define ANALOG_CHAN_18_SCL    0.001
#define ANALOG_CHAN_19_SCL    0.001
#define ANALOG_CHAN_20_SCL    0.001
#define ANALOG_CHAN_21_SCL    0.001
#define ANALOG_CHAN_22_SCL    0.001
#define ANALOG_CHAN_23_SCL    0.001
#define ANALOG_CHAN_24_SCL    0.001
#define ANALOG_CHAN_25_SCL    0.001
#define ANALOG_CHAN_26_SCL    0.001
#define ANALOG_CHAN_27_SCL    0.001
#define ANALOG_CHAN_28_SCL    0.001
#define ANALOG_CHAN_29_SCL    0.001
#define ANALOG_CHAN_30_SCL    0.001
#define ANALOG_CHAN_31_SCL    0.001
#define ANALOG_CHAN_32_SCL    0.001
#define ANALOG_CHAN_33_SCL    0.001
#define ANALOG_CHAN_34_SCL    0.001
#define ANALOG_CHAN_35_SCL    0.001
#define ANALOG_SGH_RAW_SCL    0.001
#define STRAIN_CALC_SCL       0.000001

#define TCOUPLE_SCL         0.25

#define TCOUPLE_4_SCL         0.25
#define TCOUPLE_5_SCL         0.25
#define AVG_JUNCT_TEMP_SCL    0.0625

#define FREQ_COUNT_0_SCL      1 // ???
#define FREQ_COUNT_1_SCL      1 // ???
#define FREQ_COUNT_2_SCL      1 // ???

//from IMU sensor
#define LATERAL_G_SCL         0.0001274
#define LONGITUDINAL_G_SCL    0.0001274
#define YAW_RATE_SCL          0.005
#define YAW_ACCEL_SCL         0.125

/**
 * Masks for bitmaps & packed messages
 */

// From Wheel
#define RADIO_BTN_BITPOS      1
#define ACK_BTN_BITPOS        2
#define AUX_BTN_BITPOS        3
#define FAN_OVR_BITPOS        5
#define WTR_OVR_BITPOS        6
#define FUEL_OVR_BITPOS       7
#define ND_SW_BITPOS          8

//PDM Bitmask bits
#define STR_ENBL_MASK         0x10
#define BVBAT_ENBL_MASK       0x20
#define AUX_ENBL_MASK         0x40
#define ECU_ENBL_MASK         0x80
#define WTR_ENBL_MASK         0x100
#define FAN_ENBL_MASK         0x200
#define PDLD_ENBL_MASK        0x400
#define PDLU_ENBL_MASK        0x800
#define ABS_ENBL_MASK         0x1000
#define INJ_ENBL_MASK         0x2000
#define IGN_ENBL_MASK         0x4000
#define FUEL_ENBL_MASK        0x8000

#define STR_PEAKM_MASK        0x10
#define BVBAT_PEAKM_MASK      0x20
#define AUX_PEAKM_MASK        0x40
#define ECU_PEAKM_MASK        0x80
#define WTR_PEAKM_MASK        0x100
#define FAN_PEAKM_MASK        0x200
#define PDLD_PEAKM_MASK       0x400
#define PDLU_PEAKM_MASK       0x800
#define ABS_PEAKM_MASK        0x1000
#define INJ_PEAKM_MASK        0x2000
#define IGN_PEAKM_MASK        0x4000
#define FUEL_PEAKM_MASK       0x8000

#define AUX2_PDM_SW_MASK      0x1
#define AUX1_PDM_SW_MASK      0x2
#define ABS_PDM_SW_MASK       0x4
#define KILL_PDM_SW_MASK      0x8
#define ACT_DN_PDM_SW_MASK    0x10
#define ACT_UP_PDM_SW_MASK    0x20
#define ON_PDM_SW_MASK        0x40
#define STR_PDM_SW_MASK       0x80

#define KILL_ENGINE_PDM_FLAG_MASK   0x10
#define KILL_CAR_PDM_FLAG_MASK      0x20
#define OVER_TEMP_PDM_FLAG_MASK     0x40
#define FUEL_PRIME_PDM_FLAG_MASK    0x80

#define PADDLE_UP_GCM_SW_MASK       0x1
#define PADDLE_DOWN_GCM_SW_MASK     0x2
#define NEUTRAL_GCM_SW_MASK         0x4

//SPM Bitmasks
#define TCOUPLE_0_FAULT_MASK        0x1
#define TCOUPLE_1_FAULT_MASK        0x2
#define TCOUPLE_2_FAULT_MASK        0x4
#define TCOUPLE_3_FAULT_MASK        0x8
#define TCOUPLE_4_FAULT_MASK        0x10
#define TCOUPLE_5_FAULT_MASK        0x20

#define DIGITAL_INPUT_0_MASK        0x1
#define DIGITAL_INPUT_1_MASK        0x2
#define DIGITAL_INPUT_2_MASK        0x4
#define DIGITAL_INPUT_3_MASK        0x8
#define DIGITAL_INPUT_4_MASK        0x10
#define DIGITAL_INPUT_5_MASK        0x20
#define DIGITAL_INPUT_6_MASK        0x40
#define DIGITAL_INPUT_7_MASK        0x80
#define DIGITAL_INPUT_8_MASK        0x100
#define DIGITAL_INPUT_9_MASK        0x200
#define DIGITAL_INPUT_10_MASK       0x400
#define DIGITAL_INPUT_11_MASK       0x800
#define DIGITAL_INPUT_12_MASK       0x1000
#define DIGITAL_INPUT_13_MASK       0x2000
#define DIGITAL_INPUT_14_MASK       0x4000
#define DIGITAL_INPUT_15_MASK       0x8000

#define PGA_0_SETTINGS_MASK         0x7
#define PGA_1_SETTINGS_MASK         0x38
#define PGA_2_SETTINGS_MASK         0x1C0
#define PGA_3_SETTINGS_MASK         0xE00
#define FREQ_0_SETTINGS_MASK        0x1F
#define FREQ_1_SETTINGS_MASK        0x3E0
#define FREQ_2_SETTINGS_MASK        0x7C00

#define PGA_0_SETTINGS_SHF          0
#define PGA_1_SETTINGS_SHF          3
#define PGA_2_SETTINGS_SHF          6
#define PGA_3_SETTINGS_SHF          9
#define FREQ_0_SETTINGS_SHF         0
#define FREQ_1_SETTINGS_SHF         5
#define FREQ_2_SETTINGS_SHF         10

/**
 * Offsets
 */
//from IMU sensor
#define LATERAL_G_OFFSET            (-4.17464)
#define LONGITUDINAL_G_OFFSET       (-4.17464)
#define YAW_RATE_OFFSET             (-163.84)
#define YAW_ACCEL_OFFSET            (-4096)

/**
 * ADL definitions
 */
#define ADL_IDX_1_3   0
#define ADL_IDX_4_6   1
#define ADL_IDX_7_9   2
#define ADL_IDX_10_12 3
#define ADL_IDX_BYTE  0
#define ADL1_BYTE     2
#define ADL2_BYTE     4
#define ADL3_BYTE     6
#define ADL4_BYTE     2
#define ADL5_BYTE     4
#define ADL6_BYTE     6
#define ADL7_BYTE     2
#define ADL8_BYTE     4
#define ADL9_BYTE     6
#define ADL10_BYTE    2
#define ADL11_BYTE    4
#define ADL12_BYTE    6

#endif /* CAN_H */