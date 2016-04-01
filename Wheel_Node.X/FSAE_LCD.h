/**
 * FSAE LCD Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */

#ifndef _FSAE_LCD_H    /* Guard against multiple inclusion */
#define _FSAE_LCD_H

#include <xc.h>
#include <sys/types.h>
#include "RA8875_driver.h"

#define BACKGROUND_COLOR 0xFFFF
#define FOREGROUND_COLOR 0x0000
#define NUM_SCREENS 1
#define RACE_SCREEN 0

/*
 * Defines a data stream that is relevant to one or more screens
 * 
 * value - 					double value of stream
 * warnThreshold - 	Value where data will enter a warning state
 * errThreshold - 	Value where data will enter an error state
 * refreshInterval -Maximum refresh frequency
 */
typedef struct {
	double value;
	double warnThreshold;
	double errThreshold;
	uint32_t refreshInterval;
	uint8_t wholeDigits;
	uint8_t decDigits;
} dataItem;

/* 
 * Defines an item that will be displayed on a specific screen
 * 
 * x - 							X coordinate
 * y - 							Y coordinate
 * size - 					Width to be drawn in.  Also used to determine how much to white out
 * 										during refresh
 * currentValue - 	Current value being displayed
 * data - 					Pointer to corresponding dataItem
 * refreshTime -			Time that the value was previously refreshed
 */
typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t size;
	double currentValue;
	dataItem * data;
	uint32_t refreshTime;
} screenItem;

/*
 * Defines a screen
 * 
 * name - 		Screen Name
 * elements -	Array of screen Items that will be on that screen 
 * len - 			Length of element array
 */
typedef struct {
	screenItem * items;
	uint8_t len;
} screen;

void initDataItems(void);
void initDataItem(dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec);
void initAllScreens(void);
void initScreen(uint8_t num);
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, 
				dataItem* data);
void changeScreen(uint8_t num);
void refreshScreenItems(void);
void clearScreen(void);

#endif /* _FSAE_LCD_H */