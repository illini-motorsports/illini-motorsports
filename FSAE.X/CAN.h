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
#define MOTEC0_ID       0x100
#define MOTEC1_ID       0x101
#define MOTEC2_ID       0x102
#define MOTEC3_ID       0x103
#define MOTEC4_ID       0x104
#define MOTEC5_ID       0x105
#define PADDLE0_ID      0x200

#define BEACON_ID       0x0E5
#define LOGGING_ID      0X2AA
#define FAN_SW_ID       0x500
#define PDM_ID          0x300
#define ADL_ID          0x500
//#define FAN_SW_ADL_ID   0x00

/**
 * Byte position of channels in their CAN messages
 */
#define ENG_RPM_BYTE        0
#define OIL_PRES_BYTE       4
#define OIL_TEMP_BYTE       6
#define ENG_TEMP_BYTE       0
#define VOLT_ECU_BYTE       6

#define GDN_SPD_BYTE        0
#define FAN_SW_BYTE         4
#define WATER_SW_BYTE       6
#define LOGGING_BYTE        0
#define UPTIME_BYTE         0
#define PCB_TEMP_BYTE       2

#define GEAR_BYTE           4
#define QUEUE_NT_BYTE       5
#define QUEUE_UP_BYTE       6
#define QUEUE_DN_BYTE       7

/**
 * Byte position of ADL channels in their CAN messages
 */
#define ADL_IDX_BYTE 0
#define ADL1_BYTE 2
#define ADL2_BYTE 4
#define ADL3_BYTE 6
#define ADL4_BYTE 2
#define ADL5_BYTE 4
#define ADL6_BYTE 6
#define ADL7_BYTE 2
#define ADL8_BYTE 4
#define ADL9_BYTE 6
#define ADL10_BYTE 2
#define ADL11_BYTE 4
#define ADL12_BYTE 6

/**
 * Scalars for channels
 */
#define ENG_RPM_SCL  1.0
#define ENG_TEMP_SCL 0.1
#define OIL_PRES_SCL 0.1
#define OIL_TEMP_SCL 0.1
#define VOLT_ECU_SCL 0.01

#endif /* CAN_H */