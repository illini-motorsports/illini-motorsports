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
	data->value = 0;
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
	initScreenItem(&(raceScreen.items[0]), 10, 30, 30, &oilTemp);
	initScreenItem(&(raceScreen.items[1]), 300, 30, 30, &waterTemp);
	initScreenItem(&(raceScreen.items[2]), 10, 150, 30, &oilPress);
	initScreenItem(&(raceScreen.items[3]), 300, 180, 30, 0x0);
	initScreenItem(&(raceScreen.items[4]), 150, 50, 100, &gearPos);
	
	// PDM stuff
	allScreens[PDM_DRAW_SCREEN] = &pdmDrawScreen;
	pdmDrawScreen.items = pdmDrawItems;
	pdmDrawScreen.len = 20;
	initScreenItem(&(pdmDrawScreen.items[0]), 10, 30, 20, &pdmCurrentDraw);
	initScreenItem(&(pdmDrawScreen.items[1]), 10, 30, 20, &pdmIGNdraw);
	initScreenItem(&(pdmDrawScreen.items[2]), 10, 30, 20, &pdmINJdraw);
	initScreenItem(&(pdmDrawScreen.items[3]), 10, 30, 20, &pdmFUELdraw);
	initScreenItem(&(pdmDrawScreen.items[4]), 10, 30, 20, &pdmECUdraw);
	initScreenItem(&(pdmDrawScreen.items[5]), 10, 30, 20, &pdmWTRdraw);
	initScreenItem(&(pdmDrawScreen.items[6]), 10, 30, 20, &pdmFANdraw);
	initScreenItem(&(pdmDrawScreen.items[7]), 10, 30, 20, &pdmAUXdraw);
	initScreenItem(&(pdmDrawScreen.items[8]), 10, 30, 20, &pdmPDLUdraw);
	initScreenItem(&(pdmDrawScreen.items[9]), 10, 30, 20, &pdmPDLDdraw);
	initScreenItem(&(pdmDrawScreen.items[10]), 10, 30, 20, &pdm5v5draw);
	initScreenItem(&(pdmDrawScreen.items[11]), 10, 30, 20, &pdmBATdraw);
	initScreenItem(&(pdmDrawScreen.items[12]), 10, 30, 20, &pdmSTR0draw);
	initScreenItem(&(pdmDrawScreen.items[13]), 10, 30, 20, &pdmSTR1draw);
	initScreenItem(&(pdmDrawScreen.items[14]), 10, 30, 20, &pdmSTR2draw);
	initScreenItem(&(pdmDrawScreen.items[15]), 10, 30, 20, &pdmSTRdraw);
	initScreenItem(&(pdmDrawScreen.items[16]), 10, 30, 20, &pdmFUELPcut);
	initScreenItem(&(pdmDrawScreen.items[17]), 10, 30, 20, &pdmECUPcut);
	initScreenItem(&(pdmDrawScreen.items[18]), 10, 30, 20, &pdmWTRPcut);
	initScreenItem(&(pdmDrawScreen.items[19]), 10, 30, 20, &pdmFANPcut);
	
	allScreens[PDM_CUT_SCREEN] = &pdmCutScreen;
	pdmCutScreen.items = pdmCutItems;
	pdmCutScreen.len = 21;
	initScreenItem(&(pdmDrawScreen.items[0]), 10, 30, 20, &pdmTemp);
	initScreenItem(&(pdmDrawScreen.items[1]), 10, 30, 20, &pdmICTemp);
	initScreenItem(&(pdmDrawScreen.items[2]), 10, 30, 20, &pdmVBat);
	initScreenItem(&(pdmDrawScreen.items[3]), 10, 30, 20, &pdm12v);
	initScreenItem(&(pdmDrawScreen.items[4]), 10, 30, 20, &pdm5v5);
	initScreenItem(&(pdmDrawScreen.items[5]), 10, 30, 20, &pdm5v);
	initScreenItem(&(pdmDrawScreen.items[6]), 10, 30, 20, &pdm3v3);
	initScreenItem(&(pdmDrawScreen.items[7]), 10, 30, 20, &pdmIGNcut);
	initScreenItem(&(pdmDrawScreen.items[8]), 10, 30, 20, &pdmINJcut);
	initScreenItem(&(pdmDrawScreen.items[9]), 10, 30, 20, &pdmFUELNcut);
	initScreenItem(&(pdmDrawScreen.items[10]), 10, 30, 20, &pdmECUNcut);
	initScreenItem(&(pdmDrawScreen.items[11]), 10, 30, 20, &pdmWTRNcut);
	initScreenItem(&(pdmDrawScreen.items[12]), 10, 30, 20, &pdmFANNcut);
	initScreenItem(&(pdmDrawScreen.items[13]), 10, 30, 20, &pdmAUXcut);
	initScreenItem(&(pdmDrawScreen.items[14]), 10, 30, 20, &pdmPDLUcut);
	initScreenItem(&(pdmDrawScreen.items[15]), 10, 30, 20, &pdmPDLDcut);
	initScreenItem(&(pdmDrawScreen.items[16]), 10, 30, 20, &pdm5v5cut);
	initScreenItem(&(pdmDrawScreen.items[17]), 10, 30, 20, &pdmBATcut);
	initScreenItem(&(pdmDrawScreen.items[18]), 10, 30, 20, &pdmSTR0cut);
	initScreenItem(&(pdmDrawScreen.items[19]), 10, 30, 20, &pdmSTR1cut);
	initScreenItem(&(pdmDrawScreen.items[20]), 10, 30, 20, &pdmSTR2cut);

	// MoTec Stuff
	allScreens[MOTEC_SCREEN] = &motecScreen;
	motecScreen.items = motecItems;
	motecScreen.len = 30;
	initScreenItem(&(motecScreen.items[0]), 10, 30, 20, &rpm);
	initScreenItem(&(motecScreen.items[1]), 10, 30, 20, &throtPos);
	initScreenItem(&(motecScreen.items[2]), 10, 30, 20, &oilPress);
	initScreenItem(&(motecScreen.items[3]), 10, 30, 20, &oilTemp);
	initScreenItem(&(motecScreen.items[4]), 10, 30, 20, &waterTemp);
	initScreenItem(&(motecScreen.items[5]), 10, 30, 20, &lambda);
	initScreenItem(&(motecScreen.items[6]), 10, 30, 20, &manifoldPress);
	initScreenItem(&(motecScreen.items[7]), 10, 30, 20, &batVoltage);
	initScreenItem(&(motecScreen.items[8]), 10, 30, 20, &wheelSpeedFL);
	initScreenItem(&(motecScreen.items[9]), 10, 30, 20, &wheelSpeedFR);
	initScreenItem(&(motecScreen.items[10]), 10, 30, 20, &wheelSpeedRL);
	initScreenItem(&(motecScreen.items[11]), 10, 30, 20, &wheelSpeedRR);
	initScreenItem(&(motecScreen.items[12]), 10, 30, 20, &gpsLat);
	initScreenItem(&(motecScreen.items[13]), 10, 30, 20, &gpsLong);
	initScreenItem(&(motecScreen.items[14]), 10, 30, 20, &groundSpeed);
	initScreenItem(&(motecScreen.items[15]), 10, 30, 20, &driveSpeed);
	initScreenItem(&(motecScreen.items[16]), 10, 30, 20, &gpsSpeed);
	initScreenItem(&(motecScreen.items[17]), 10, 30, 20, &manifoldTemp);
	initScreenItem(&(motecScreen.items[18]), 10, 30, 20, &ambientTemp);
	initScreenItem(&(motecScreen.items[19]), 10, 30, 20, &ambientPress);
	initScreenItem(&(motecScreen.items[20]), 10, 30, 20, &fuelTemp);
	initScreenItem(&(motecScreen.items[21]), 10, 30, 20, &fuelPress);
	initScreenItem(&(motecScreen.items[22]), 10, 30, 20, &lambda1);
	initScreenItem(&(motecScreen.items[23]), 10, 30, 20, &lambda2);
	initScreenItem(&(motecScreen.items[24]), 10, 30, 20, &lambda3);
	initScreenItem(&(motecScreen.items[25]), 10, 30, 20, &lambda4);
	initScreenItem(&(motecScreen.items[26]), 10, 30, 20, &lcEnablity);
	initScreenItem(&(motecScreen.items[27]), 10, 30, 20, &fuelConsum);
	initScreenItem(&(motecScreen.items[28]), 10, 30, 20, &gpsAltitude);
	initScreenItem(&(motecScreen.items[29]), 10, 30, 20, &gpsTime);
	initScreenItem(&(motecScreen.items[30]), 10, 30, 20, &fuelInjDuty);
	initScreenItem(&(motecScreen.items[31]), 10, 30, 20, &fuelTrim);

	// Lap Time Stuff
	lapTimeHead = 0;
	numLaps = 0;
	int i;
	for(i=0;i<20;i++){ 
		initDataItem(&(lapTimeBuffer[i]),0,0,1000,2,1);
		lapTimeBuffer[i].value = -1;
	}
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, 
				dataItem* data){
	item->x = x;
	item->y = y;
	item->size = size;
	item->currentValue = 1;
	item->data = data;
	item->refreshTime = millis;
}

// Helper function for drawing labels and such on a new screen
void initScreen(uint8_t num){
	if(num == RACE_SCREEN){
		textMode();
		textSetCursor(0, 0);
		textTransparent(FOREGROUND_COLOR);
		textEnlarge(0);
		textWrite("OIL TMP", 0);
		textSetCursor(0, 100);
		textWrite("OIL PRESS", 0);
		textSetCursor(300, 0);
		textWrite("WTR TMP", 0);
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
		if((currScreen->items[i]).currentValue != (currScreen->items[i]).data->value && 
						millis - (currScreen->items[i]).refreshTime >= 
						(currScreen->items[i]).data->refreshInterval){
			redrawItem(&(currScreen->items[i]));
		}
	}
}

// Redraw Helper Function
void redrawItem(screenItem * item){
	uint16_t fillColor;
	if(item->currentValue >= item->data->warnThreshold){
		if(item->currentValue >= item->data->errThreshold){
			fillColor = ERROR_COLOR;
		}else{
			fillColor = WARNING_COLOR;
		}
	}else{
		fillColor = BACKGROUND_COLOR;
	}
	uint16_t wholeNums, decNums;
	wholeNums = item->data->wholeDigits;
	decNums = item->data->decDigits;
	uint16_t fillWidth = (item->size)*(wholeNums + decNums) + (item->size)*0.2*(wholeNums + decNums - 1);
	if(decNums){
		fillWidth += (item->size)/5;
	}
	fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
	sevenSegmentDecimal(item->x, item->y, item->size, wholeNums + decNums, decNums, FOREGROUND_COLOR, item->data->value);
	item->refreshTime = millis;
	item->currentValue = item->data->value;
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