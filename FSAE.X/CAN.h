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
#define BEACON_ID       0xE5L
#define MOTEC_ID        0x200L
#define RPM_ID          0x200L
#define VOLTAGE_ID      0x201L
#define ENGINE_TEMP_ID  0x201
#define LOGGING_ID      0X2AAL
#define OIL_TEMP_ID     0x200L
#define OIL_PRESS_ID    0x200L
#define FAN_SW_ID       0x500L
#define PDM_ID          0x300L
#define ADL_ID          0x500L
#define FAN_SW_ADL_ID   0x00

/**
 * Byte position of channels in their CAN messages
 */
#define RPM_BYTE            0
#define OIL_PRESS_BYTE      4
#define OIL_TEMP_BYTE       6
#define ENGINE_TEMP_BYTE    0
#define GEAR_BYTE           4
#define VOLTAGE_BYTE        6
#define GDN_SPD_BYTE        0
#define FAN_SW_BYTE         4
#define WATER_SW_BYTE       6
#define LOGGING_BYTE        0

/**
 * Byte position of ADL channels in their CAN messages
 */
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

#endif /* CAN_H */