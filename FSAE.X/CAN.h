/**
 * FSAE Library CAN Defines
 *
 * Author:      George Schwieters
 * Created:     2014-2015
 */
#ifndef CAN_H
#define CAN_H

/**
 * Timing parameters for CAN control
 */
#define NO_DATA_WAIT    1000
#define NO_CAN_WAIT     1000

/**
 * CAN message IDs
 */
#define ERROR_ID        0x00F
#define ANALOG_REAR_ID  0x050
#define ANALOG_FRONT_ID 0x060
#define MOTEC_ID        0x100
#define PADDLE_ID       0x200
#define LOGGER_ID       0X2AA
#define WHEEL_ID        0x400
#define ADL_ID          0x500
#define PDM_ID          0x600
#define PDM_CONFIG_ID   0x620

#define BEACON_ID       0x0E5

/**
 * Byte position of channels in their CAN messages
 */

// From Diagnostic Message
#define UPTIME_BYTE         0
#define PCB_TEMP_BYTE       2

// From Error Message
#define ORIGIN_BYTE         0
#define ERRNO_BYTE          2

// From MoTeC
#define ENG_RPM_BYTE        0
#define THROTTLE_POS_BYTE   2
#define LAMBDA_BYTE					4
#define VOLT_ECU_BYTE       6

#define ENG_TEMP_BYTE       0
#define OIL_TEMP_BYTE       2
#define MANIFOLD_TEMP_BYTE	4
#define FUEL_TEMP_BYTE			6

#define AMBIENT_PRES_BYTE		0
#define OIL_PRES_BYTE       2
#define MANIFOLD_PRES_BYTE	4
#define FUEL_PRES_BYTE			6

#define WHEELSPEED_FL_BYTE	0
#define WHEELSPEED_FR_BYTE	2
#define WHEELSPEED_RL_BYTE	4
#define WHEELSPEED_RR_BYTE	6

#define DRIVE_SPEED_BYTE		0
#define GROUND_SPEED_BYTE		2
#define GPS_SPEED_BYTE			4
#define GPS_ALT_BYTE				6

#define GPS_LAT_BYTE				0
#define GPS_LONG_BYTE				4

#define GPS_TIME_BYTE				0
#define RUN_TIME_BYTE				4
#define FUEL_USED_BYTE			6

#define FUEL_INJ_DUTY_BYTE	0
#define FUEL_TRIM_BYTE			2

// From PaddleShifting
#define PADDLE_UPTIME_BYTE	0
#define PADDLE_TEMP_BYTE		2
#define QUEUE_NT_BYTE       4
#define QUEUE_UP_BYTE       5
#define QUEUE_DN_BYTE       6

#define GEAR_VOLT_BYTE      0
#define GEAR_BYTE           2

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
#define PDM_UPTIME_BYTE 		0
#define PDM_PCB_TEMP_BYTE 	2
#define PDM_IC_TEMP_BYTE		4
#define TOTAL_CURRENT_BYTE	6

#define LOAD_ENABLITY_BYTE  0
#define LOAD_PEAK_BYTE			2
#define PDM_SWITCH_BYTE			4

#define VBAT_RAIL_BYTE			0
#define V12_RAIL_BYTE				2
#define V5V5_RAIL_BYTE			4
#define V5_RAIL_BYTE				6

#define V3V3_RAIL_BYTE			0

#define IGN_DRAW_BYTE				0
#define INJ_DRAW_BYTE				2
#define FUEL_DRAW_BYTE			4
#define ECU_DRAW_BYTE				6

#define WTR_DRAW_BYTE				0
#define FAN_DRAW_BYTE				2
#define AUX_DRAW_BYTE				4
#define PDLU_DRAW_BYTE			6

#define PDLD_DRAW_BYTE			0
#define B5V5_DRAW_BYTE			2
#define VBAT_DRAW_BYTE			4

#define STR0_DRAW_BYTE			0
#define STR1_DRAW_BYTE			2
#define STR2_DRAW_BYTE			4
#define STR_DRAW_BYTE				6

#define IGN_CUT_BYTE				0
#define INJ_CUT_BYTE				2
#define AUX_CUT_BYTE				4
#define PDLU_CUT_BYTE				6

#define PDLD_CUT_BYTE				0
#define B5V5_CUT_BYTE				2
#define BVBAT_CUT_BYTE			4

#define STR0_CUT_BYTE				0
#define STR1_CUT_BYTE				2
#define STR2_CUT_BYTE				4

#define FUEL_CUT_N_BYTE			0
#define WTR_CUT_N_BYTE			2
#define FAN_CUT_N_BYTE			4
#define ECU_CUT_N_BYTE			6

#define FUEL_CUT_P_BYTE			0
#define WTR_CUT_P_BYTE			2
#define FAN_CUT_P_BYTE			4
#define ECU_CUT_P_BYTE			6

/**
 * Scalars for channels
 */

// From Motec
#define ENG_RPM_SCL					1.0
#define THROTTLE_POS_SCL		0.1
#define LAMBDA_SCL					0.001
#define VOLT_ECU_SCL				0.01

#define ENG_TEMP_SCL				0.1
#define OIL_TEMP_SCL				0.1
#define MANIFOLD_TEMP_SCL		0.1
#define FUEL_TEMP_SCL				0.1

#define AMBIENT_PRES_SCL		0.001
#define OIL_PRES_SCL       	0.001
#define MANIFOLD_PRES_SCL		0.001
#define FUEL_PRES_SCL				0.001

#define WHEELSPEED_FL_SCL		0.1
#define WHEELSPEED_FR_SCL		0.1
#define WHEELSPEED_RL_SCL		0.1
#define WHEELSPEED_RR_SCL		0.1

#define DRIVE_SPEED_SCL			0.1
#define GROUND_SPEED_SCL		0.1
#define GPS_SPEED_SCL				0.1
#define GPS_ALT_SCL					0.1 //??

#define GPS_LAT_SCL					0.0000001
#define GPS_LONG_SCL				0.0000001

#define GPS_TIME_SCL				1 //??
#define RUN_TIME_SCL				0.1
#define FUEL_USED_SCL				0.1 //??

#define FUEL_INJ_DUTY_SCL		0.1
#define FUEL_TRIM_SCL				0.1 //??

// From PaddleShifting
#define PADDLE_UPTIME_SCL		1
#define PADDLE_TEMP_SCL			0.005
#define QUEUE_NT_SCL       	1
#define QUEUE_UP_SCL       	1
#define QUEUE_DN_SCL       	1

#define GEAR_VOLT_SCL      	0.0001
#define GEAR_SCL           	1

// From PDM
#define PDM_UPTIME_SCL 			1
#define PDM_PCB_TEMP_SCL 		0.005
#define PDM_IC_TEMP_SCL			0.005
#define TOTAL_CURRENT_SCL		0.01

#define LOAD_ENABLITY_SCL  	1
#define LOAD_PEAK_SCL				1
#define PDM_SWITCH_SCL			1

#define VBAT_RAIL_SCL				0.001
#define V12_RAIL_SCL				0.001
#define V5V5_RAIL_SCL				0.001
#define V5_RAIL_SCL					0.001

#define V3V3_RAIL_SCL				0.001

#define IGN_DRAW_SCL				0.001
#define INJ_DRAW_SCL				0.001
#define FUEL_DRAW_SCL				0.001
#define ECU_DRAW_SCL				0.001

#define WTR_DRAW_SCL				0.001
#define FAN_DRAW_SCL				0.001
#define AUX_DRAW_SCL				0.001
#define PDLU_DRAW_SCL				0.001

#define PDLD_DRAW_SCL				0.001
#define B5V5_DRAW_SCL				0.001
#define BVBAT_DRAW_SCL			0.001

#define STR0_DRAW_SCL				0.01
#define STR1_DRAW_SCL				0.01
#define STR2_DRAW_SCL				0.01
#define STR_DRAW_SCL				0.01

#define IGN_CUT_SCL					0.0025
#define INJ_CUT_SCL					0.0025
#define AUX_CUT_SCL					0.0025
#define PDLU_CUT_SCL				0.0025

#define PDLD_CUT_SCL				0.0025
#define B5V5_CUT_SCL				0.0025
#define BVBAT_CUT_SCL				0.0025

#define STR0_CUT_SCL				0.0025
#define STR1_CUT_SCL				0.0025
#define STR2_CUT_SCL				0.0025

#define FUEL_CUT_N_SCL			0.0025
#define WTR_CUT_N_SCL				0.0025
#define FAN_CUT_N_SCL				0.0025
#define ECU_CUT_N_SCL				0.0025

#define FUEL_CUT_P_SCL			0.0025
#define WTR_CUT_P_SCL				0.0025
#define FAN_CUT_P_SCL				0.0025
#define ECU_CUT_P_SCL				0.0025

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
