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
#define VOLT_ECU_BYTE       6
#define ENG_TEMP_BYTE       0
#define OIL_TEMP_BYTE       2
#define OIL_PRES_BYTE       2
#define THROTTLE_POS_BYTE   2

// From PaddleShifting
#define GEAR_VOLT_BYTE      0
#define GEAR_BYTE           2
#define QUEUE_NT_BYTE       4
#define QUEUE_UP_BYTE       5
#define QUEUE_DN_BYTE       6

// From Wheel
#define SWITCH_BITS_BYTE    6

// From PDM Programming
#define LOAD_IDX_BYTE       0
#define PEAK_MODE_BYTE      1
#define CUTOFF_SETTING_BYTE 2

//TODO: Sort/change these
#define GDN_SPD_BYTE        0
#define LOGGING_BYTE        0

/**
 * Scalars for channels
 */
#define ENG_RPM_SCL  1.0
#define ENG_TEMP_SCL 0.1
#define OIL_PRES_SCL 0.001;
#define OIL_TEMP_SCL 0.1
#define VOLT_ECU_SCL 0.01

/**
 * Masks for bitmaps
 */
#define FAN_OVER_MASK 0x8
#define WTR_OVER_MASK 0x4

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
