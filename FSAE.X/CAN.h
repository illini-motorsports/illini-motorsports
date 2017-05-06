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
#define ERROR_ID        0x00F
#define SPM_ID          0x050
#define MOTEC_ID        0x100
#define GCM_ID          0x200
#define LOGGER_ID       0X2AA
#define WHEEL_ID        0x400
#define ADL_ID          0x500
#define PDM_ID          0x600
#define PDM_CONFIG_ID   0x620

#define BEACON_ID       0x0E5
#define TIRE_TEMP_FL_ID 0x001
#define TIRE_TEMP_FR_ID 0x002
#define TIRE_TEMP_RL_ID 0x003
#define TIRE_TEMP_RR_ID 0x004

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

// From GCM
#define GEAR_BYTE           0
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

// From Tire Temp Sensors
#define TIRE_TEMP_1_BYTE    0
#define TIRE_TEMP_2_BYTE    2
#define TIRE_TEMP_3_BYTE    4
#define TIRE_TEMP_4_BYTE    6

/**
 * Scalars for channels
 */

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

// From Tire Temp Sensors
#define TIRE_TEMP_SCL       0.1

/**
 * Masks for bitmaps & packed messages
 */

// From Wheel
#define FAN_OVER_MASK  0x10
#define WTR_OVER_MASK  0x20
#define FUEL_OVER_MASK 0x40
#define RADIO_BTN_MASK 0x01
#define ROT1_MASK      0xF0
#define ROT2_MASK      0x0F
#define ROT3_MASK      0xF0

// From PDM
#define KILL_SW_MASK   0x08
#define ACT_DN_SW_MASK 0x10
#define ACT_UP_SW_MASK 0x20
#define ON_SW_MASK     0x40
#define STR_SW_MASK    0x80

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
