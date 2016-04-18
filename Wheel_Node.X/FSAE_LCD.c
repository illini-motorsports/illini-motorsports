/**
 * FSAE LCD Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#include "RA8875_driver.h"
#include "Wheel.h"
#include "FSAE_LCD.h"

// Initialize all the data streams
// This fn must be run before CAN is initialized
void initDataItems(void){
	// Motec Vars
	// Refresh Intervals
	// All Temp Channels - 500
	initDataItem(&rpm,0,0,MIN_REFRESH,5,0);
	initDataItem(&throtPos,0,0,MIN_REFRESH,3,1);
	initDataItem(&oilPress,0,0,MIN_REFRESH,1,2);
	initDataItem(&oilTemp,0,0,MIN_REFRESH,3,0);
	initDataItem(&waterTemp,0,0,MIN_REFRESH,3,0);
	initDataItem(&lambda,0,0,MIN_REFRESH,1,3);
	initDataItem(&manifoldPress,0,0,MIN_REFRESH,1,2);
	initDataItem(&batVoltage,0,0,MIN_REFRESH,2,2);
	initDataItem(&wheelSpeedFL,0,0,MIN_REFRESH,2,1);
	initDataItem(&wheelSpeedFR,0,0,MIN_REFRESH,2,1);
	initDataItem(&wheelSpeedRL,0,0,MIN_REFRESH,2,1);
	initDataItem(&wheelSpeedRR,0,0,MIN_REFRESH,2,1);
	initDataItem(&gpsLong,0,0,MIN_REFRESH,2,1); // Don't need
	initDataItem(&gpsLat,0,0,MIN_REFRESH,2,1); // Don't need
	initDataItem(&groundSpeed,0,0,MIN_REFRESH,2,1);
	initDataItem(&driveSpeed,0,0,MIN_REFRESH,2,1);
	initDataItem(&gpsSpeed,0,0,MIN_REFRESH,2,1);
	initDataItem(&manifoldTemp,0,0,MIN_REFRESH,3,0);
	initDataItem(&ambientTemp,0,0,MIN_REFRESH,3,0);
	initDataItem(&ambientPress,0,0,MIN_REFRESH,1,2);
	initDataItem(&fuelTemp,0,0,MIN_REFRESH,3,0);
	initDataItem(&fuelPress,0,0,MIN_REFRESH,1,2);
	initDataItem(&lambda1,0,0,MIN_REFRESH,1,3);
	initDataItem(&lambda2,0,0,MIN_REFRESH,1,3);
	initDataItem(&lambda3,0,0,MIN_REFRESH,1,3);
	initDataItem(&lambda4,0,0,MIN_REFRESH,1,3);
	initDataItem(&lcEnablity,0,0,MIN_REFRESH,1,0);
	initDataItem(&fuelConsum,0,0,MIN_REFRESH,2,1);
	initDataItem(&gpsAltitude,0,0,MIN_REFRESH,2,1); // ?
	initDataItem(&gpsTime,0,0,MIN_REFRESH,2,1); // ?
	initDataItem(&runTime,0,0,MIN_REFRESH,4,0);
	initDataItem(&fuelInjDuty,0,0,MIN_REFRESH,3,1);
	initDataItem(&fuelTrim,0,0,MIN_REFRESH,3,1);

	// Tire Temps
	initDataItem(&ttFL1,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFL2,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFL3,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFL4,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFL,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFR1,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFR2,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFR3,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFR4,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttFR,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRL1,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRL2,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRL3,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRL4,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRL,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRR1,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRR2,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRR3,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRR4,0,0,MIN_REFRESH,2,1);
	initDataItem(&ttRR,0,0,MIN_REFRESH,2,1);

	// Steering Wheel
	initDataItem(&swTemp,0,0,MIN_REFRESH,2,1);
	initDataItem(&swSW1,0,0,MIN_REFRESH,2,1);
	initDataItem(&swSW2,0,0,MIN_REFRESH,2,1);
	initDataItem(&swSW3,0,0,MIN_REFRESH,2,1);
	initDataItem(&swSW4,0,0,MIN_REFRESH,2,1);
	initDataItem(&swROT1,0,0,MIN_REFRESH,2,1);
	initDataItem(&swROT2,0,0,MIN_REFRESH,2,1);
	initDataItem(&swROT3,0,0,MIN_REFRESH,2,1);
	initDataItem(&swTROT1,0,0,MIN_REFRESH,2,1);
	initDataItem(&swTROT2,0,0,MIN_REFRESH,2,1);
	initDataItem(&swBUT1,0,0,MIN_REFRESH,2,1);
	initDataItem(&swBUT2,0,0,MIN_REFRESH,2,1);
	initDataItem(&swBUT3,0,0,MIN_REFRESH,2,1);
	initDataItem(&swBUT4,0,0,MIN_REFRESH,2,1);

	// Paddle Shifting
	initDataItem(&paddleTemp,0,0,MIN_REFRESH,2,1);
	initDataItem(&gearPos,0,0,MIN_REFRESH,1,0);
	initDataItem(&neutQueue,0,0,MIN_REFRESH,1,0);
	initDataItem(&upQueue,0,0,MIN_REFRESH,1,0);
	initDataItem(&downQueue,0,0,MIN_REFRESH,1,0);
	initDataItem(&gearVoltage,0,0,MIN_REFRESH,1,2);

	// PDM
	initDataItem(&pdmTemp,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmICTemp,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmCurrentDraw,0,0,MIN_REFRESH,3,1);
	initDataItem(&pdmVBat,0,0,MIN_REFRESH,2,2);
	initDataItem(&pdm12v,0,0,MIN_REFRESH,2,2);
	initDataItem(&pdm5v5,0,0,MIN_REFRESH,1,2);
	initDataItem(&pdm5v,0,0,MIN_REFRESH,1,2);
	initDataItem(&pdm3v3,0,0,MIN_REFRESH,1,2);
	// Draw (^str) - 2,2
	// Cut (^str)) - 3,1
	initDataItem(&pdmIGNdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmIGNcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmINJdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmINJcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFUELdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFUELNcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFUELPcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmECUdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmECUNcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmECUPcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmWTRdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmWTRNcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmWTRPcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFANdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFANNcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmFANPcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmAUXdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmAUXcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmPDLUdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmPDLUcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmPDLDdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmPDLDcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdm5v5draw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdm5v5cut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmBATdraw,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmBATcut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmSTR0draw,0,0,MIN_REFRESH,3,1);
	initDataItem(&pdmSTR0cut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmSTR1draw,0,0,MIN_REFRESH,3,1);
	initDataItem(&pdmSTR1cut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmSTR2draw,0,0,MIN_REFRESH,3,1);
	initDataItem(&pdmSTR2cut,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmSTRdraw,0,0,MIN_REFRESH,3,1);

	// Uptimes
	// uptimes - 4,0
	initDataItem(&paddleUptime,0,0,MIN_REFRESH,2,1);
	initDataItem(&loggerUptime,0,0,MIN_REFRESH,2,1);
	initDataItem(&swUptime,0,0,MIN_REFRESH,2,1);
	initDataItem(&pdmUptime,0,0,MIN_REFRESH,2,1);

	// EndRace
	initDataItem(&endLogNum,0,0,1000,2,1);
	initDataItem(&endNumLaps,0,0,1000,2,1);
	initDataItem(&endFastLap,0,0,1000,2,1);
	initDataItem(&endTireTempFL,0,0,1000,2,1);
	initDataItem(&endTireTempFR,0,0,1000,2,1);
	initDataItem(&endTireTempRL,0,0,1000,2,1);
	initDataItem(&endTireTempRR,0,0,1000,2,1);
	initDataItem(&endAmbientTemp,0,0,1000,2,1);
	initDataItem(&endFuelConsum,0,0,1000,2,1);
}

void initDataItem(dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec){
	data->value = 2;
	data->warnThreshold = warn;
	data->errThreshold = err;
	data->refreshInterval = refresh;
	data->wholeDigits = whole;
	data->decDigits = dec;
}

// Initializes all the screen and screenitem variables in all
// the screens that might be displayed
void initAllScreens(void){
	// Race Screen Stuff
	allScreens[RACE_SCREEN] = &raceScreen;
	raceScreen.items = raceScreenItems;
	raceScreen.len = 5;
	initScreenItem(&raceScreen.items[0], 20, 30, 30, redrawDigit, &oilTemp);
	initScreenItem(&raceScreen.items[1], 330, 30, 30, redrawDigit, &waterTemp);
	initScreenItem(&raceScreen.items[2], 20, 150, 30, redrawDigit, &oilPress);
	initScreenItem(&raceScreen.items[3], 330, 180, 30, redrawDigit, 0x0);
	initScreenItem(&raceScreen.items[4], 170, 50, 100, redrawDigit, &gearPos);
	
	// PDM stuff
	allScreens[PDM_DRAW_SCREEN] = &pdmDrawScreen;
	pdmDrawScreen.items = pdmDrawItems;
	pdmDrawScreen.len = 20;
	initScreenItem(&pdmDrawScreen.items[0], 10, 30, 20, redrawDigit, &pdmCurrentDraw);
	initScreenItem(&pdmDrawScreen.items[1], 10, 30, 20, redrawDigit, &pdmIGNdraw);
	initScreenItem(&pdmDrawScreen.items[2], 10, 30, 20, redrawDigit, &pdmINJdraw);
	initScreenItem(&pdmDrawScreen.items[3], 10, 30, 20, redrawDigit, &pdmFUELdraw);
	initScreenItem(&pdmDrawScreen.items[4], 10, 30, 20, redrawDigit, &pdmECUdraw);
	initScreenItem(&pdmDrawScreen.items[5], 10, 30, 20, redrawDigit, &pdmWTRdraw);
	initScreenItem(&pdmDrawScreen.items[6], 10, 30, 20, redrawDigit, &pdmFANdraw);
	initScreenItem(&pdmDrawScreen.items[7], 10, 30, 20, redrawDigit, &pdmAUXdraw);
	initScreenItem(&pdmDrawScreen.items[8], 10, 30, 20, redrawDigit, &pdmPDLUdraw);
	initScreenItem(&pdmDrawScreen.items[9], 10, 30, 20, redrawDigit, &pdmPDLDdraw);
	initScreenItem(&pdmDrawScreen.items[10], 10, 30, 20, redrawDigit, &pdm5v5draw);
	initScreenItem(&pdmDrawScreen.items[11], 10, 30, 20, redrawDigit, &pdmBATdraw);
	initScreenItem(&pdmDrawScreen.items[12], 10, 30, 20, redrawDigit, &pdmSTR0draw);
	initScreenItem(&pdmDrawScreen.items[13], 10, 30, 20, redrawDigit, &pdmSTR1draw);
	initScreenItem(&pdmDrawScreen.items[14], 10, 30, 20, redrawDigit, &pdmSTR2draw);
	initScreenItem(&pdmDrawScreen.items[15], 10, 30, 20, redrawDigit, &pdmSTRdraw);
	initScreenItem(&pdmDrawScreen.items[16], 10, 30, 20, redrawDigit, &pdmFUELPcut);
	initScreenItem(&pdmDrawScreen.items[17], 10, 30, 20, redrawDigit, &pdmECUPcut);
	initScreenItem(&pdmDrawScreen.items[18], 10, 30, 20, redrawDigit, &pdmWTRPcut);
	initScreenItem(&pdmDrawScreen.items[19], 10, 30, 20, redrawDigit, &pdmFANPcut);
	
	allScreens[PDM_CUT_SCREEN] = &pdmCutScreen;
	pdmCutScreen.items = pdmCutItems;
	pdmCutScreen.len = 21;
	initScreenItem(&pdmDrawScreen.items[0], 10, 30, 20, redrawDigit, &pdmTemp);
	initScreenItem(&pdmDrawScreen.items[1], 10, 30, 20, redrawDigit, &pdmICTemp);
	initScreenItem(&pdmDrawScreen.items[2], 10, 30, 20, redrawDigit, &pdmVBat);
	initScreenItem(&pdmDrawScreen.items[3], 10, 30, 20, redrawDigit, &pdm12v);
	initScreenItem(&pdmDrawScreen.items[4], 10, 30, 20, redrawDigit, &pdm5v5);
	initScreenItem(&pdmDrawScreen.items[5], 10, 30, 20, redrawDigit, &pdm5v);
	initScreenItem(&pdmDrawScreen.items[6], 10, 30, 20, redrawDigit, &pdm3v3);
	initScreenItem(&pdmDrawScreen.items[7], 10, 30, 20, redrawDigit, &pdmIGNcut);
	initScreenItem(&pdmDrawScreen.items[8], 10, 30, 20, redrawDigit, &pdmINJcut);
	initScreenItem(&pdmDrawScreen.items[9], 10, 30, 20, redrawDigit, &pdmFUELNcut);
	initScreenItem(&pdmDrawScreen.items[10], 10, 30, 20, redrawDigit, &pdmECUNcut);
	initScreenItem(&pdmDrawScreen.items[11], 10, 30, 20, redrawDigit, &pdmWTRNcut);
	initScreenItem(&pdmDrawScreen.items[12], 10, 30, 20, redrawDigit, &pdmFANNcut);
	initScreenItem(&pdmDrawScreen.items[13], 10, 30, 20, redrawDigit, &pdmAUXcut);
	initScreenItem(&pdmDrawScreen.items[14], 10, 30, 20, redrawDigit, &pdmPDLUcut);
	initScreenItem(&pdmDrawScreen.items[15], 10, 30, 20, redrawDigit, &pdmPDLDcut);
	initScreenItem(&pdmDrawScreen.items[16], 10, 30, 20, redrawDigit, &pdm5v5cut);
	initScreenItem(&pdmDrawScreen.items[17], 10, 30, 20, redrawDigit, &pdmBATcut);
	initScreenItem(&pdmDrawScreen.items[18], 10, 30, 20, redrawDigit, &pdmSTR0cut);
	initScreenItem(&pdmDrawScreen.items[19], 10, 30, 20, redrawDigit, &pdmSTR1cut);
	initScreenItem(&pdmDrawScreen.items[20], 10, 30, 20, redrawDigit, &pdmSTR2cut);

	// MoTec Stuff
	allScreens[MOTEC_SCREEN] = &motecScreen;
	motecScreen.items = motecItems;
	motecScreen.len = 30;
	initScreenItem(&motecScreen.items[0], 10, 30, 20, redrawDigit, &rpm);
	initScreenItem(&motecScreen.items[1], 10, 30, 20, redrawDigit, &throtPos);
	initScreenItem(&motecScreen.items[2], 10, 30, 20, redrawDigit, &oilPress);
	initScreenItem(&motecScreen.items[3], 10, 30, 20, redrawDigit, &oilTemp);
	initScreenItem(&motecScreen.items[4], 10, 30, 20, redrawDigit, &waterTemp);
	initScreenItem(&motecScreen.items[5], 10, 30, 20, redrawDigit, &lambda);
	initScreenItem(&motecScreen.items[6], 10, 30, 20, redrawDigit, &manifoldPress);
	initScreenItem(&motecScreen.items[7], 10, 30, 20, redrawDigit, &batVoltage);
	initScreenItem(&motecScreen.items[8], 10, 30, 20, redrawDigit, &wheelSpeedFL);
	initScreenItem(&motecScreen.items[9], 10, 30, 20, redrawDigit, &wheelSpeedFR);
	initScreenItem(&motecScreen.items[10], 10, 30, 20, redrawDigit, &wheelSpeedRL);
	initScreenItem(&motecScreen.items[11], 10, 30, 20, redrawDigit, &wheelSpeedRR);
	initScreenItem(&motecScreen.items[12], 10, 30, 20, redrawDigit, &gpsLat);
	initScreenItem(&motecScreen.items[13], 10, 30, 20, redrawDigit, &gpsLong);
	initScreenItem(&motecScreen.items[14], 10, 30, 20, redrawDigit, &groundSpeed);
	initScreenItem(&motecScreen.items[15], 10, 30, 20, redrawDigit, &driveSpeed);
	initScreenItem(&motecScreen.items[16], 10, 30, 20, redrawDigit, &gpsSpeed);
	initScreenItem(&motecScreen.items[17], 10, 30, 20, redrawDigit, &manifoldTemp);
	initScreenItem(&motecScreen.items[18], 10, 30, 20, redrawDigit, &ambientTemp);
	initScreenItem(&motecScreen.items[19], 10, 30, 20, redrawDigit, &ambientPress);
	initScreenItem(&motecScreen.items[20], 10, 30, 20, redrawDigit, &fuelTemp);
	initScreenItem(&motecScreen.items[21], 10, 30, 20, redrawDigit, &fuelPress);
	initScreenItem(&motecScreen.items[22], 10, 30, 20, redrawDigit, &lambda1);
	initScreenItem(&motecScreen.items[23], 10, 30, 20, redrawDigit, &lambda2);
	initScreenItem(&motecScreen.items[24], 10, 30, 20, redrawDigit, &lambda3);
	initScreenItem(&motecScreen.items[25], 10, 30, 20, redrawDigit, &lambda4);
	initScreenItem(&motecScreen.items[26], 10, 30, 20, redrawDigit, &lcEnablity);
	initScreenItem(&motecScreen.items[27], 10, 30, 20, redrawDigit, &fuelConsum);
	initScreenItem(&motecScreen.items[28], 10, 30, 20, redrawDigit, &gpsAltitude);
	initScreenItem(&motecScreen.items[29], 10, 30, 20, redrawDigit, &gpsTime);
	initScreenItem(&motecScreen.items[30], 10, 30, 20, redrawDigit, &fuelInjDuty);
	initScreenItem(&motecScreen.items[31], 10, 30, 20, redrawDigit, &fuelTrim);

	// End Race Screen
	allScreens[END_RACE_SCREEN] = &endRaceScreen;
	endRaceScreen.items = endRaceItems;
	endRaceScreen.len = 9;
	initScreenItem(&endRaceItems[0], 10, 30, 20, redrawDigit, &endLogNum);
	initScreenItem(&endRaceItems[1], 10, 30, 20, redrawDigit, &endNumLaps);
	initScreenItem(&endRaceItems[2], 10, 30, 20, redrawDigit, &endFastLap);
	initScreenItem(&endRaceItems[3], 10, 30, 20, redrawDigit, &endTireTempFL);
	initScreenItem(&endRaceItems[4], 10, 30, 20, redrawDigit, &endTireTempFR);
	initScreenItem(&endRaceItems[5], 10, 30, 20, redrawDigit, &endTireTempRL);
	initScreenItem(&endRaceItems[6], 10, 30, 20, redrawDigit, &endTireTempRR);
	initScreenItem(&endRaceItems[3], 10, 30, 20, redrawTireTemp, &endTireTempFL);
	initScreenItem(&endRaceItems[4], 10, 30, 20, redrawTireTemp, &endTireTempFR);
	initScreenItem(&endRaceItems[5], 10, 30, 20, redrawTireTemp, &endTireTempRL);
	initScreenItem(&endRaceItems[6], 10, 30, 20, redrawTireTemp, &endTireTempRR);
	initScreenItem(&endRaceItems[7], 10, 30, 20, redrawDigit, &endAmbientTemp);
	initScreenItem(&endRaceItems[8], 10, 30, 20, redrawDigit, &endFuelConsum);

	allScreens[CHASSIS_SCREEN] = &chassisScreen;
	chassisScreen.items = chassisItems;
	chassisScreen.len = 20;
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttFL);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttFR);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttRL);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttRR);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, &ttFL);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, &ttFR);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, &ttRL);
	initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, &ttRR);

	// Lap Time Stuff
	lapTimeHead = 0;
	numLaps = 0;
	int i;
	for(i=0;i<20;i++){ 
		initDataItem(&(lapTimeBuffer[i]),0,0,1000,2,1);
		lapTimeBuffer[i].value = -1;
	}

	// error Stuff
	errBufferHead = 0;
	errBufferTail = 0;
	for(i=0;i<20;i++){
		errBuffer[i].errText = 0x0;
		errBuffer[i].item = 0x0;
		errBuffer[i].priority = 0;
	}
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, dataItem *), dataItem* data){
	item->info.x = x;
	item->info.y = y;
	item->info.size = size;
	item->currentValue = 1;
	item->data = data;
	item->refreshTime = millis;
	item->redrawItem = redrawItem;
}

// Helper function for drawing labels and such on a new screen
void initScreen(uint8_t num){
	if(num == RACE_SCREEN){
		textMode();
		textSetCursor(0, 0);
		textTransparent(FOREGROUND_COLOR);
		textEnlarge(0);
		textWrite("OIL TEMP", 0);
		textSetCursor(0, 100);
		textWrite("OIL PRESS", 0);
		textSetCursor(330, 0);
		textWrite("WTR TMP", 0);
		graphicsMode();
	}
}

// User called function to change which screen is being displayed
void changeScreen(uint8_t num){
	if( num < 0 || num > NUM_SCREENS) {return;}
	screenNumber = num;
	clearScreen();
	initScreen(num);
	refreshScreenItems();
}

// Function to asynchronously update all the items currently being displayed
// A number will only be redrawn if its dataItem value has changed, and it 
// has surpassed its refresh interval
void refreshScreenItems(void){
	screen *currScreen = allScreens[screenNumber];
	int i;
	for(i = 0;i<currScreen->len;i++){
		if(currScreen->items[i].data == 0x0){
			continue;
		}
		if((currScreen->items[i]).currentValue != (currScreen->items[i]).data->value && millis - (currScreen->items[i]).refreshTime >= (currScreen->items[i]).data->refreshInterval){
			currScreen->items[i].redrawItem(&(currScreen->items[i].info), currScreen->items[i].data);
			currScreen->items[i].currentValue = currScreen->items[i].data->value;
			currScreen->items[i].refreshTime = millis;
		}
	}
}

// Redraw Helper Function
void redrawDigit(screenItemInfo * item, dataItem * data){
	uint16_t fillColor;
	if(data->value >= data->warnThreshold){
		if(data->value >= data->errThreshold){
			fillColor = ERROR_COLOR;
		}else{
			fillColor = WARNING_COLOR;
		}
	}else{
		fillColor = BACKGROUND_COLOR;
	}
	uint16_t wholeNums, decNums;
	wholeNums = data->wholeDigits;
	decNums = data->decDigits;
	uint16_t fillWidth = (item->size)*(wholeNums + decNums) + (item->size)*0.2*(wholeNums + decNums - 1);
	if(decNums){
		fillWidth += (item->size)/5;
	}
	fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
	sevenSegmentDecimal(item->x, item->y, item->size, wholeNums + decNums, decNums, FOREGROUND_COLOR, data->value);
}

void redrawGearPos(screenItemInfo * item, dataItem * data){
	fillRect(item->x, item->y, item->size, item->size * 1.75, BACKGROUND_COLOR);
	if(data->value == 0){
		sevenSegment(item->x, item->y, item->size, FOREGROUND_COLOR, SEVEN_SEG_N);
	}
	else if(data->value == 7){
		sevenSegment(item->x, item->y, item->size, FOREGROUND_COLOR, SEVEN_SEG_E);
	}
	else{
		sevenSegmentDigit(item->x, item->y, item->size, FOREGROUND_COLOR, data->value);
	}
}

void redrawFanSw(screenItemInfo * item, dataItem * data){
	if(data->value){
		fillCircle(item->x, item->y, item->size, RA8875_RED);
	}
	else{
		fillCircle(item->x, item->y, item->size, BACKGROUND_COLOR);
	}
}

void redrawPumpSw(screenItemInfo * item, dataItem * data){
	if(data->value){
		fillCircle(item->x, item->y, item->size, RA8875_RED);
	}
	else{
		fillCircle(item->x, item->y, item->size, BACKGROUND_COLOR);
	}
}

void redrawLCSw(screenItemInfo * item, dataItem * data){
	if(data->value){
		fillCircle(item->x, item->y, item->size, RA8875_RED);
	}
	else{
		fillCircle(item->x, item->y, item->size, BACKGROUND_COLOR);
	}
}

void redrawTireTemp(screenItemInfo * item, dataItem * data){
	uint16_t fillColor = tempColor(data->value);
	fillCircleSquare(item->x, item->y, item->size, item->size*1.75, item->size/10, fillColor);
}

void redrawSPBar(screenItemInfo * item, dataItem * data){
	fillRect(item->x, item->y, item->size, item->size * 5, BACKGROUND_COLOR);
	if(data->value > MIN_SUS_POS){
		uint16_t height = ((item->size * 5)/(MAX_SUS_POS - MIN_SUS_POS))*(data->value - MIN_SUS_POS);
		if(height > MAX_SUS_POS){
			height = MAX_SUS_POS;
		}
		fillRect(item->x, item->y - (item->size * 5) + height, item->size, height, RA8875_RED);
	}
}

void clearScreen(void){
	fillScreen(BACKGROUND_COLOR);
}

void addLap(double lapTime){
	lapTimeHead = (lapTimeHead+1)%20;
	lapTimeBuffer[lapTimeHead].value= lapTime;
	numLaps++;
}

double getMinLap(void){
	uint8_t counter = lapTimeHead;
	uint8_t iters = 0;
	double minLap = lapTimeBuffer[counter].value;
	while(lapTimeBuffer[counter].value != -1 && iters < 20){
		if(minLap > lapTimeBuffer[counter].value){
			minLap = lapTimeBuffer[counter].value;
		}
		counter = (counter+1)%20;
		iters++;
	}
	return minLap;
}

void endRace(void){
	endLogNum.value = 0;
	endNumLaps.value = numLaps;
	endFastLap.value = getMinLap();
	endTireTempFL.value = ttFL.value;
	endTireTempFR.value = ttFR.value;
	endTireTempRL.value = ttRL.value;
	endTireTempRR.value = ttRR.value; 
	endAmbientTemp.value = ambientTemp.value;
	endFuelConsum.value = fuelConsum.value;
}

// Error Handling Stuff
void displayNoErrors(void){
	// Reset error area
	fillRect(0,200,WIDTH,HEIGHT-200,BACKGROUND_COLOR);
	drawChevron(30, 210, 30, 50, FOREGROUND_COLOR, BACKGROUND_COLOR);
	textMode();
	textSetCursor(100, 210);
	textTransparent(RA8875_BLACK);
	textEnlarge(1);
	textWrite("Illini Motorsports",0);
	graphicsMode();
}

void addError(char * errText, dataItem * item, uint8_t priority){
	errBufferHead = (errBufferHead + 1)%20;
	if(errBufferHead == errBufferTail){
		errBufferTail = (errBufferTail + 1)%20;
	}
	errBuffer[errBufferHead].errText = errText;
	errBuffer[errBufferHead].item = item;
	errBuffer[errBufferHead].priority = priority;
}

uint16_t tempColor(uint8_t temp){
	return RA8875_RED;
}

