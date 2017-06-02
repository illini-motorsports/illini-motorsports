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
  int i;

  // Set default initialization for PDM data items
  for(i=0;i<PDM_DATAITEM_SIZE;i++){
    initDataItem(&pdmDataItems[i],0,0,MIN_REFRESH,2,1);
  }

  // Customized PDM dataitem initialization
  pdmDataItems[STR_DRAW_IDX].wholeDigits = 3;

  // Set default initialization for GCM data items
  for(i=0;i<GCM_DATAITEM_SIZE;i++){
    initDataItem(&gcmDataItems[i],0,0,MIN_REFRESH,1,0);
  }

  // Customized GCM dataitem initialization
  gcmDataItems[GEAR_VOLT_IDX].decDigits = 2;
  gcmDataItems[GEAR_VOLT_IDX].wholeDigits = 2;
  gcmDataItems[FORCE_IDX].decDigits = 2;
  gcmDataItems[FORCE_IDX].wholeDigits = 2;

  // Set default initializations for Motec data items
  for(i=0;i<MOTEC_DATAITEM_SIZE;i++) {
    initDataItem(&motecDataItems[i],0,0,MIN_REFRESH,2,1);
  }

  // Customized Motec dataitem initialization
  setDataItemDigits(&motecDataItems[ENG_RPM_IDX], 5, 0);
  setDataItemDigits(&motecDataItems[THROTTLE_POS_IDX], 0, 2);
  setDataItemDigits(&motecDataItems[OIL_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[OIL_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[LAMBDA_IDX], 3, 1);
  setDataItemDigits(&motecDataItems[MANIFOLD_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[MANIFOLD_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[ENG_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[VOLT_ECU_IDX], 2, 2);
  setDataItemDigits(&motecDataItems[AMBIENT_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[FUEL_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[FUEL_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[RUN_TIME_IDX], 4, 0);
  setDataItemDigits(&motecDataItems[FUEL_INJ_DUTY_IDX], 3, 1);
  setDataItemDigits(&motecDataItems[FUEL_TRIM_IDX], 3, 1);

  // Tire temps
  for(i=0;i<TIRETEMP_DATAITEM_SIZE;i++) {
    initDataItem(&tireTempDataItems[i],0,0,MIN_REFRESH,2,1);
  }

  // SPM
  for(i=0;i<SPM_DATAITEM_SIZE;i++) {
    initDataItem(&spmDataItems[i],0,0,MIN_REFRESH,2,1);
  }

  //Switches and Rotaries
  for(i=0;i<3;i++) {
    initDataItem(&wheelDataItems[i],0,0,MIN_REFRESH,2,1);
  }
  for(i=3;i<WHEEL_DATAITEM_SIZE;i++) {
    initDataItem(&wheelDataItems[i],0,0,MIN_REFRESH,1,0);
  }

  // Special array declerations
  fanSw[0] = &wheelDataItems[SWITCH_0_IDX];
  fanSw[1] = &pdmDataItems[FAN_ENABLITY_IDX];
  fuelSw[0] = &wheelDataItems[SWITCH_1_IDX];
  fuelSw[1] = &pdmDataItems[FUEL_ENABLITY_IDX];
  wtrSw[0] = &wheelDataItems[SWITCH_2_IDX];
  wtrSw[1] = &pdmDataItems[WTR_ENABLITY_IDX];
}

void initDataItem(volatile dataItem* data, double warn, double err, uint32_t refresh, uint8_t whole, uint8_t dec){
  data->value = 0;
  data->warnThreshold = warn;
  data->errThreshold = err;
  data->refreshInterval = refresh;
  data->wholeDigits = whole;
  data->decDigits = dec;
}

void setDataItemDigits(volatile dataItem* data, uint8_t whole, uint8_t dec) {
  data->wholeDigits = whole;
  data->decDigits = dec;
}

// Initializes all the screen and screenitem variables in all
// the screens that might be displayed
void initAllScreens(void){
  // Initialize colors
  backgroundColor = RA8875_WHITE;
  foregroundColor = RA8875_BLACK;
  //initNightMode(switches[3].value);
  initNightMode(1); //TODO

  // Race Screen Stuff
  allScreens[RACE_SCREEN] = &raceScreen;
  raceScreen.items = raceScreenItems;
  raceScreen.len = 10;
  initScreenItem(&raceScreenItems[0], 120, 20, 15, redrawFanSw, *fanSw);
  initScreenItem(&raceScreenItems[1], 240, 20, 15, redrawFUELPumpSw,*fuelSw);
  initScreenItem(&raceScreenItems[2], 360, 20, 15, redrawWTRPumpSw, *wtrSw);
  initScreenItem(&raceScreenItems[3], 20, 70, 30, redrawDigit, &motecDataItems[OIL_TEMP_IDX]);
  initScreenItem(&raceScreenItems[4], 330, 70, 30, redrawDigit, &motecDataItems[ENG_TEMP_IDX]);
  initScreenItem(&raceScreenItems[5], 20, 190, 30, redrawDigit, &motecDataItems[OIL_PRES_IDX]);
  initScreenItem(&raceScreenItems[6], 330, 180, 30, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX]);
  initScreenItem(&raceScreenItems[7], 170, 50, 100, redrawGearPos, &gcmDataItems[GEAR_IDX]);
  initScreenItem(&raceScreenItems[8], 20, 30, 15, redrawShiftLightsRPM, &motecDataItems[ENG_RPM_IDX]);
  initScreenItem(&raceScreenItems[9], 20, 30, 15, redrawKILLCluster, &pdmDataItems[KILL_SWITCH_IDX]);

  // PDM stuff
  allScreens[PDM_DRAW_SCREEN] = &pdmDrawScreen;
  pdmDrawScreen.items = pdmDrawItems;
  pdmDrawScreen.len = 20;
  initScreenItem(&pdmDrawItems[1], 10, 50, 15, redrawDigit, &pdmDataItems[IGN_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[2], 85, 50, 15, redrawDigit, &pdmDataItems[INJ_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[3], 160, 50, 15, redrawDigit, &pdmDataItems[FUEL_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[4], 235, 50, 15, redrawDigit, &pdmDataItems[ECU_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[5], 310, 50, 15, redrawDigit, &pdmDataItems[WTR_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[6], 385, 50, 15, redrawDigit, &pdmDataItems[FAN_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[7], 10, 100, 15, redrawDigit, &pdmDataItems[AUX_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[8], 85, 100, 15, redrawDigit, &pdmDataItems[PDLU_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[9], 160, 100, 15, redrawDigit, &pdmDataItems[PDLD_DRAW_IDX]);
  //changed from 5v5
  initScreenItem(&pdmDrawItems[10], 235, 100, 15, redrawDigit, &pdmDataItems[ABS_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[11], 310, 100, 15, redrawDigit, &pdmDataItems[BVBAT_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[16], 385, 100, 15, redrawDigit, &pdmDataItems[FUEL_CUT_P_IDX]);
  initScreenItem(&pdmDrawItems[17], 10, 150, 15, redrawDigit, &pdmDataItems[ECU_CUT_P_IDX]);
  initScreenItem(&pdmDrawItems[18], 85, 150, 15, redrawDigit, &pdmDataItems[WTR_CUT_P_IDX]);
  initScreenItem(&pdmDrawItems[19], 160, 150, 15, redrawDigit, &pdmDataItems[FAN_CUT_P_IDX]);
  initScreenItem(&pdmDrawItems[15], 280, 200, 15, redrawDigit, &pdmDataItems[STR_DRAW_IDX]);
  initScreenItem(&pdmDrawItems[0], 370, 200, 15, redrawDigit, &pdmDataItems[TOTAL_CURRENT_IDX]);


  allScreens[PDM_CUT_SCREEN] = &pdmCutScreen;
  pdmCutScreen.items = pdmCutItems;
  pdmCutScreen.len = 21;
  initScreenItem(&pdmCutItems[0], 10, 50, 15, redrawDigit, &pdmDataItems[PCB_TEMP_IDX]);
  initScreenItem(&pdmCutItems[1], 85, 50, 15, redrawDigit, &pdmDataItems[IC_TEMP_IDX]);
  initScreenItem(&pdmCutItems[2], 235, 200, 15, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX]);
  initScreenItem(&pdmCutItems[3], 330, 200, 15, redrawDigit, &pdmDataItems[V12_RAIL_IDX]);
  initScreenItem(&pdmCutItems[4], 310, 50, 15, redrawDigit, 0x0);
  initScreenItem(&pdmCutItems[5], 385, 50, 15, redrawDigit, 0x0);
  initScreenItem(&pdmCutItems[6], 10, 100, 15, redrawDigit, 0x0);
  initScreenItem(&pdmCutItems[7], 85, 100, 15, redrawDigit, &pdmDataItems[IGN_CUT_IDX]);
  initScreenItem(&pdmCutItems[8], 160, 100, 15, redrawDigit, &pdmDataItems[INJ_CUT_IDX]);
  initScreenItem(&pdmCutItems[9], 235, 100, 15, redrawDigit, &pdmDataItems[FUEL_CUT_IDX]);
  initScreenItem(&pdmCutItems[10], 160, 50, 15, redrawDigit,  &pdmDataItems[ECU_CUT_IDX]);
  initScreenItem(&pdmCutItems[12], 10, 150, 15, redrawDigit,  &pdmDataItems[FAN_CUT_IDX]);
  initScreenItem(&pdmCutItems[13], 85, 150, 15, redrawDigit,  &pdmDataItems[AUX_CUT_IDX]);
  initScreenItem(&pdmCutItems[14], 160, 150, 15, redrawDigit, &pdmDataItems[PDLU_CUT_IDX]);
  initScreenItem(&pdmCutItems[15], 235, 150, 15, redrawDigit, &pdmDataItems[PDLD_CUT_IDX]);
  initScreenItem(&pdmCutItems[16], 310, 150, 15, redrawDigit, &pdmDataItems[ABS_CUT_IDX]);
  initScreenItem(&pdmCutItems[17], 385, 150, 15, redrawDigit, &pdmDataItems[BVBAT_CUT_IDX]);
  initScreenItem(&pdmCutItems[18], 10, 200, 15, redrawDigit,  0x0);


  /*
  //brake screen
  allScreens[BRAKE_SCREEN] = &brakeScreen;
  brakeScreen.items = brakeItems;
  brakeScreen.len = 8;
  initScreenItem(&brakeItems[0], 20, 180, 30, redrawDigit, &brakePressFront);
  initScreenItem(&brakeItems[1], 360, 180, 30, redrawDigit, &brakePressRear);
  initScreenItem(&brakeItems[2], 80, 60, 20, redrawDigit, &brakeMinFront);
  initScreenItem(&brakeItems[3], 80, 100, 20, redrawDigit, &brakeMaxFront);
  initScreenItem(&brakeItems[4],360, 60, 20, redrawDigit, &brakeMinRear);
  initScreenItem(&brakeItems[5], 360, 100, 20, redrawDigit, &brakeMaxRear);
  initScreenItem(&brakeItems[6], 190, 40, 20, redrawBrakeBar, &brakePressFront);
  initScreenItem(&brakeItems[7], 250, 40, 20, redrawBrakeBar, &brakePressRear);
  */
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, volatile dataItem *, double), volatile dataItem* data){
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
  switch(num){
    case RACE_SCREEN:
      textMode();
      textEnlarge(1);
      textTransparent(foregroundColor);
      textSetCursor(0, 40);
      textWrite("OIL TEMP");
      textSetCursor(0, 140);
      textWrite("OIL PRESS");
      textSetCursor(330, 40);
      textWrite("WTR TMP");
      textSetCursor(330,140);
      textWrite("BAT V");
      textEnlarge(0);
      graphicsMode();
      break;

    case PDM_DRAW_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,30);
      textWrite("IGN DRW");
      textSetCursor(85, 30);
      textWrite("INJ DRW");
      textSetCursor(160, 30);
      textWrite("FUEL DRW");
      textSetCursor(235, 30);
      textWrite("ECU DRW");
      textSetCursor(310, 30);
      textWrite("WTR DRW");
      textSetCursor(385, 30);
      textWrite("FAN DRW");
      textSetCursor(10, 80);
      textWrite("AUX DRW");
      textSetCursor(85, 80);
      textWrite("PDLU DRW");
      textSetCursor(160,80);
      textWrite("PDLD DRW");
      textSetCursor(235, 80);
      textWrite("5V5 DRW");
      textSetCursor(310, 80);
      textWrite("BAT DRW");
      textSetCursor(385, 80);
      textWrite("FUEL P CUT");
      textSetCursor(10, 130);
      textWrite("ECU PCUT");
      textSetCursor(85, 130);
      textWrite("WTR PCUT");
      textSetCursor(160, 130);
      textWrite("FAN PCUT");
      textSetCursor(10, 180);
      textWrite("STR0 DRW");
      textSetCursor(100, 180);
      textWrite("STR1 DRW");
      textSetCursor(190, 180);
      textWrite("STR2 DRW");
      textSetCursor(280, 180);
      textWrite("STR DRW");
      textSetCursor(370, 180);
      textWrite("TOT DRW");
      graphicsMode();
      break;

    case PDM_CUT_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,30);
      textWrite("PDM TEMP");
      textSetCursor(85, 30);
      textWrite("IC TEMP");
      textSetCursor(160, 30);
      textWrite("BAT VOLT");
      textSetCursor(235, 30);
      textWrite("12V VOLT");
      textSetCursor(310, 30);
      textWrite("5V5 VOLT");
      textSetCursor(385, 30);
      textWrite("5V VOLT");
      textSetCursor(10, 80);
      textWrite("3V3 VOLT");
      textSetCursor(85, 80);
      textWrite("IGN CUT");
      textSetCursor(160,80);
      textWrite("INJ CUT");
      textSetCursor(235, 80);
      textWrite("FUEL NCUT");
      textSetCursor(310, 80);
      textWrite("ECU NCUT");
      textSetCursor(385, 80);
      textWrite("WTR NCUT");
      textSetCursor(10, 130);
      textWrite("FAN NCUT");
      textSetCursor(85, 130);
      textWrite("AUX CUT");
      textSetCursor(160, 130);
      textWrite("PDLU CUT");
      textSetCursor(235, 130);
      textWrite("PDLU CUT");
      textSetCursor(310,130);
      textWrite("5V5 CUT");
      textSetCursor(385, 130);
      textWrite("BAT CUT");
      textSetCursor(10, 180);
      textWrite("STR0 CUT");
      textSetCursor(85, 180);
      textWrite("BRK F");
      textSetCursor(160, 180);
      textWrite("BRK R");
      graphicsMode();
      break;

    case MOTEC_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;
    case END_RACE_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;
    case CHASSIS_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;

    case BRAKE_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textEnlarge(1);
      textSetCursor(20, 235);
      textWrite("BRK F");
      textSetCursor(360, 235);
      textWrite("BRK R");
      textSetCursor(20,60);
      textWrite("MIN F:");
      textSetCursor(20,100);
      textWrite("MAX F:");
      textSetCursor(300,60);
      textWrite("MIN R:");
      textSetCursor(300,100);
      textWrite("MAX R:");
      graphicsMode();
      textEnlarge(0);
      break;
  }
}

void changeAUXType(uint8_t num){
  if(num != auxNumber && screenNumber == RACE_SCREEN){
    fillRect(300, 140, 179, 135, backgroundColor);
    textMode();
    textTransparent(foregroundColor);
    textEnlarge(1);
    textSetCursor(330, 140);
    switch(num) {
      // Battery Voltage
      case 0:
        textWrite("BAT V");
        raceScreenItems[6].data = &pdmDataItems[VBAT_RAIL_IDX];
        raceScreenItems[6].info.x += 30;
        break;

        // Lambda
      case 1:
        textWrite("LAMBDA");
        raceScreenItems[6].data = &motecDataItems[LAMBDA_IDX];
        break;

        // RPM
      case 2:
        textWrite("RPM");
        raceScreenItems[6].data = &motecDataItems[ENG_RPM_IDX];
        raceScreenItems[6].info.x -= 30;
        break;
    }
    textEnlarge(0);
    graphicsMode();
    auxNumber = num;
  }
}

// User called function to change which screen is being displayed
void changeScreen(uint8_t num){
  if( num < 0 || num > NUM_SCREENS) {return;}
  screenNumber = num;
  clearScreen();
  initScreen(num);
  resetScreenItems();
  refreshScreenItems();
}

// Function to asynchronously update all the items currently being displayed
// A number will only be redrawn if its dataItem value has changed, and it
// has surpassed its refresh interval
void refreshScreenItems(void){
  // change night mode if the switch was toggled
  //nightMode(switches[3].value); //TODO

  screen* currScreen = allScreens[screenNumber];
  int i;
  for(i = 0;i<currScreen->len;i++){
    screenItem * currItem = &currScreen->items[i];
    if(currItem->data && millis - currItem->refreshTime >= currItem->data->refreshInterval){
      currItem->redrawItem(&currItem->info, currItem->data, currItem->currentValue);
      currItem->currentValue = currItem->data->value;
      currItem->refreshTime = millis;
    }
  }
}

// Redraw General Data
void redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  // Set Backround Color
  uint8_t warning = data->value >= data->warnThreshold;
  uint16_t fillColor;
  if(warning){
    if(data->value >= data->errThreshold){
      fillColor = errorColor;
    }else{
      fillColor = warningColor;
    }
  }else{
    fillColor = backgroundColor;
  }
  // Calculate Fill Rectangle width
  uint16_t wholeNums, decNums;
  wholeNums = data->wholeDigits;
  decNums = data->decDigits;
  uint16_t fillWidth = (item->size)*(wholeNums+decNums)+(item->size)*0.2*(wholeNums+decNums-1);
  if(decNums){
    fillWidth += (item->size)/5;
  }
  fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
  if(!warning || millis%1000 > 300) { // draw number if normal, or blink at 1hz
    sevenSegmentDecimal(item->x,item->y,item->size,wholeNums+decNums,decNums,foregroundColor,data->value);
  }
}

// For Single Digits with Error and Neutral Displays
void redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 1.75, backgroundColor);
  if(data->value == 0){
    sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_N);
  }
  else if(data->value == 7){
    sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_E);
  }
  else{
    sevenSegmentDigit(item->x, item->y, item->size, foregroundColor, data->value);
  }
}

// For Fan Override Indicator
void redrawFanSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // Fan On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// For Water Pump Override Indicator
void redrawWTRPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // WTR On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// For Launch Control Override Indicator
void redrawFUELPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // FUEL On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// Uses the 4 Tire Temp sensors to draw a color gradient tire
void redrawTireTemp(screenItemInfo * item, volatile dataItem * data, double currentValue){
  uint16_t fillColor = tempColor(data->value);
  uint16_t x = item->x;
  uint16_t y = item->y;
  uint16_t width = item->size / 4;
  uint16_t height = item->size * 2;
  fillCircleSquare(x,y,width*2,height,width,tempColor(data[0].value));
  fillCircleSquare(x+(2*width),y,width*2,height,width,tempColor(data[3].value));
  fillRect(x+width,y,width,height,tempColor(data[1].value));
  fillRect(x+(2*width),y,width,height,tempColor(data[2].value));
}

// Draws a bar with a height proportional to the suspension position
void redrawSPBar(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 5, backgroundColor);
  if(data->value > MIN_SUS_POS){
    uint16_t height = ((item->size*5)/(MAX_SUS_POS-MIN_SUS_POS))*(data->value-MIN_SUS_POS);
    if(height > MAX_SUS_POS){
      height = MAX_SUS_POS;
    }
    fillRect(item->x,item->y-(item->size*5)+height,item->size,height,RA8875_RED);
  }
}

//Draw a bar with height proportional to the brake pressure
void redrawBrakeBar(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 10, backgroundColor);
  if(data->value > MIN_BRAKE_PRESS){
    uint16_t height = ((item->size*10)/(MAX_BRAKE_PRESS-MIN_BRAKE_PRESS))*(data->value-MIN_BRAKE_PRESS);
    if(height > MAX_BRAKE_PRESS){
      height = MAX_BRAKE_PRESS;
    }
    fillRect(item->x,item->y-(item->size*10)+height,item->size,height,RA8875_RED);
  }
}

void redrawRotary(screenItemInfo * item, volatile dataItem * data, double currentValue){
  fillCircle(item->x, item->y, item->size, RA8875_RED);
  if(data->value == currentValue){
    return;
  }
  sevenSegmentDigit(item->x-(item->size/2.0),item->y-(item->size/2.0),item->size,RA8875_BLACK,data->value);
}

uint8_t _getShiftLightsRevRange(uint16_t rpm) {
  if (rpm > REV_RANGE_REDLINE) {
    return 10;
  } else if (rpm > REV_RANGE_9) {
    return 9;
  } else if (rpm > REV_RANGE_8) {
    return 8;
  } else if (rpm > REV_RANGE_7) {
    return 7;
  } else if (rpm > REV_RANGE_6) {
    return 6;
  } else if (rpm > REV_RANGE_5) {
    return 5;
  } else if (rpm > REV_RANGE_4) {
    return 4;
  } else if (rpm > REV_RANGE_3) {
    return 3;
  } else if (rpm > REV_RANGE_2) {
    return 2;
  } else if (rpm > REV_RANGE_1) {
    return 1;
  } else {
    return 0;
  }
}

void redrawShiftLightsRPM(screenItemInfo * item, volatile dataItem * data, double currentValue) {
  uint16_t rpm = (uint16_t) data->value;
  uint8_t num_leds = _getShiftLightsRevRange(rpm);
  if (num_leds == _getShiftLightsRevRange(((uint16_t) currentValue))) {
    return;
  }

  if (num_leds == 10) {
    tlc5955_set_main_blink(1, GRN, NO_OVR);
  } else {
    uint8_t i;
    uint64_t colorArray[9] = {0};
    for (i = 0; i < num_leds; i++) {
      colorArray[i] = BLU;
    }
    tlc5955_set_main_blink(0, 0x0, NO_OVR);
    tlc5955_write_main_colors(colorArray);
  }
}

void redrawKILLCluster(screenItemInfo * item, volatile dataItem * data, double currentValue) {
  uint8_t kill = data->value ? 1 : 0;
  if (kill && !tlc5955_get_cluster_warn(CLUSTER_RIGHT) ||
      !kill && tlc5955_get_cluster_warn(CLUSTER_RIGHT)) {
    if (!tlc5955_get_startup()) {
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, kill, RED, NO_OVR);
    }
  }
}

void clearScreen(void){
  fillScreen(backgroundColor);
}

// Resets the current value of all screen items on a screen
// This means the
void resetScreenItems(void){
  int i = 0;
  screen *currScreen = allScreens[screenNumber];
  for(;i<currScreen->len;i++){
    if(currScreen->items[i].data){
      currScreen->items[i].currentValue = !(currScreen->items[i].data->value);
    }
  }
}

// Immediantly returns if
uint8_t initNightMode(uint8_t on){
  if(on){
    if(backgroundColor == RA8875_BLACK){
      warningColor = errorColor = backgroundColor;
      return 0;
    }
    backgroundColor = RA8875_BLACK;
    foregroundColor = RA8875_WHITE;
  }
  else{
    if(backgroundColor == RA8875_WHITE){
      warningColor = errorColor = backgroundColor;
      return 0;
    }
    backgroundColor = RA8875_WHITE;
    foregroundColor = RA8875_BLACK;
  }
  warningColor = errorColor = backgroundColor;
  return 1;
}

void nightMode(uint8_t on){
  if(initNightMode(on)){
    changeScreen(screenNumber);
  }
}

uint16_t tempColor(uint8_t temp){
  return RA8875_RED;
}
