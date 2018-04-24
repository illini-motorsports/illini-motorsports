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
  warningCount = 0;

  uint16_t i;
  // Set default initialization for PDM data items
  for(i=0;i<PDM_DATAITEM_SIZE;i++){
    initDataItem(&pdmDataItems[i],200,200,2,1);
    pdmDataItems[i].thresholdDir = 1;
  }

  // Set default initialization for GCM data items
  for(i=0;i<GCM_DATAITEM_SIZE;i++){
    initDataItem(&gcmDataItems[i],0,0,1,0);
  }

  // Set default initializations for Motec data items
  for(i=0;i<MOTEC_DATAITEM_SIZE;i++) {
    initDataItem(&motecDataItems[i],0,0,2,1);
  }

  // Customized Motec dataitem initialization
  setDataItemDigits(&motecDataItems[ENG_RPM_IDX], 5, 0);
  setDataItemDigits(&motecDataItems[THROTTLE_POS_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[OIL_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[OIL_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[LAMBDA_IDX], 1, 3);
  setDataItemDigits(&motecDataItems[ENG_TEMP_IDX], 3, 0);
  motecDataItems[LAMBDA_IDX].warnThreshold = 1.1;
  motecDataItems[LAMBDA_IDX].errThreshold = 1.3;
  motecDataItems[LAMBDA_IDX].thresholdDir = 1;
  motecDataItems[OIL_PRES_IDX].warnThreshold = 1.2;
  motecDataItems[OIL_PRES_IDX].errThreshold = 1.2;
  motecDataItems[OIL_TEMP_IDX].thresholdDir = 1;
  motecDataItems[OIL_TEMP_IDX].warnThreshold = 120;
  motecDataItems[OIL_TEMP_IDX].errThreshold = 140;
  motecDataItems[ENG_TEMP_IDX].thresholdDir = 1;
  motecDataItems[ENG_TEMP_IDX].warnThreshold = 100;
  motecDataItems[ENG_TEMP_IDX].errThreshold = 110;
  motecDataItems[ENG_RPM_IDX].warnThreshold = 12000;
  motecDataItems[ENG_RPM_IDX].errThreshold = 13000;
  motecDataItems[ENG_RPM_IDX].thresholdDir = 1;
  motecDataItems[LAMBDA_IDX].warnThreshold = 1.1;
  motecDataItems[LAMBDA_IDX].errThreshold = 1.2;
  motecDataItems[LAMBDA_IDX].thresholdDir = 1;

  //Switches and Rotaries
  for(i=0;i<3;i++) {
    initDataItem(&wheelDataItems[i],0,0,2,1);
  }
  for(i=3;i<WHEEL_DATAITEM_SIZE;i++) {
    initDataItem(&wheelDataItems[i],0,0,1,0);
  }
}

void initDataItem(volatile dataItem* data, double warn, double err, uint8_t whole, uint8_t dec){
  data->value = 0;
  data->warnThreshold = warn;
  data->errThreshold = err;
  data->thresholdDir = 0;
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
  foregroundColor2 = RA8875_GREEN;

  // Shift lights screenItem
  initScreenItem(&shiftLightsItem, 0, 0, 0, redrawShiftLights, &pdmDataItems[KILL_SWITCH_IDX], MIN_REFRESH, "SHIFT LIGHTS", -1);
  shiftLightsItem.head.next = &shiftLightsGearPos;
  shiftLightsGearPos.next = &shiftLightsRPM;
  shiftLightsGearPos.data = &gcmDataItems[GEAR_IDX];
  shiftLightsGearPos.currentValue = 0;
  shiftLightsRPM.next = NULL;
  shiftLightsRPM.data = &motecDataItems[ENG_RPM_IDX];
  shiftLightsRPM.currentValue = 0;

  // Race Screen Stuff
  allScreens[RACE_SCREEN] = &raceScreen;
  raceScreen.items = raceScreenItems;
  raceScreen.len = 6;
  initScreenItem(&raceScreenItems[0], 20, 90, 30, redrawDigit, &motecDataItems[OIL_TEMP_IDX], MIN_REFRESH, "OIL TEMP", 1);
  initScreenItem(&raceScreenItems[1], 360, 90, 30, redrawDigit, &motecDataItems[ENG_TEMP_IDX], MIN_REFRESH, "ENG TEMP", 1);
  initScreenItem(&raceScreenItems[2], 20, 210, 30, redrawDigit, &motecDataItems[OIL_PRES_IDX], MIN_REFRESH, "OIL_PRESS", 1);
  initScreenItem(&raceScreenItems[3], 350, 210, 30, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX], MIN_REFRESH, "BAT V", 1);
  initScreenItem(&raceScreenItems[4], 200, 20, 80, redrawGearPos, &gcmDataItems[GEAR_IDX], MIN_REFRESH, "GEAR POS", -1);
  initScreenItem(&raceScreenItems[5], 180, 210, 30, redrawDigit, &motecDataItems[LAMBDA_IDX], MIN_REFRESH, "LAMBDA", 1);
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, screenItemNode *), volatile dataItem* data, uint32_t interval, char * label, int8_t labelSize){
  item->info.x = x;
  item->info.y = y;
  item->info.size = size;

  item->head.data = data;
  item->head.currentValue = 1;
  item->head.next = NULL; // default to only one screenItemNode

  item->refreshTime = millis;
  item->redrawItem = redrawItem;
  item->refreshInterval = interval;
  item->label = label;
  item->labelSize = labelSize;
}

// Helper function for drawing labels and such on a new screen
void initScreen(uint8_t num){
  uint16_t i;
  textMode();
  textTransparent(foregroundColor);
  screen * currScreen = allScreens[num];
  for(i = 0; i < currScreen->len; i++) {
    screenItem * item = &currScreen->items[i];
    if(item->labelSize >= 0) {
      textEnlarge(item->labelSize);
      // TODO: compare text length with number of digits to determine X offset
      uint16_t x = item->info.x;
      uint16_t y = item->info.y - item->info.size - ((item->labelSize + 1) * 8) - 5;
      textSetCursor(x, y);
      textWrite(item->label);
    }
  }
  graphicsMode();
}

// User called function to change which screen is being displayed
void changeScreen(uint8_t num){
  if( num < 0 || num > NUM_SCREENS) {
    return;
  }
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
  screen* currScreen = allScreens[screenNumber];
  int i;
  screenItem * currItem;
  for(i = 0;i<currScreen->len;i++){
    currItem = &currScreen->items[i];
    if(millis - currItem->refreshTime >= currItem->refreshInterval){
      currItem->redrawItem(&currItem->info, &currItem->head);
      currItem->refreshTime = millis;
    }
  }
  // Always redraw shift lights
  currItem = &shiftLightsItem;
  if(millis - currItem->refreshTime >= currItem->refreshInterval){
    currItem->redrawItem(&currItem->info, &currItem->head);
    currItem->refreshTime = millis;
  }
}

// Redraw General Data
void redrawDigit(screenItemInfo * item, screenItemNode * head){
  volatile dataItem * data = head->data;
  uint8_t warning = data->thresholdDir ? data->value >= data->warnThreshold: data->value <= data->warnThreshold;
  uint8_t error = data->thresholdDir ? data->value >= data->errThreshold: data->value <= data->errThreshold;

  if(!data->warningState && (error || warning)) { // Transition into warning state
    data->warningState = 1;
    warningCount++;
  }

  if(data->warningState && !error && !warning) { // Transition out of warning state
    data->warningState = 0;
    warningCount--;
  }

  uint8_t stale = millis - data->refreshTime > CAN_TIMEOUT;
  if(stale) {
    data->value = 0;
  }
  if(!error && !stale && !checkDataChange(head)) {
    return;
  }
  // Set Backround Color
  uint16_t fillColor;
  uint16_t numColor;
  if(warning){
    if(error){
      fillColor = errorColor;
      numColor = RA8875_WHITE;
    }else{
      fillColor = warningColor;
      numColor = RA8875_BLUE;
    }
  }
  else if(stale) {
    fillColor = errorColor;
  }
  else{
    fillColor = backgroundColor;
    numColor = foregroundColor;
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
  if(!stale && (!error || millis%500 > 200)) { // draw number if normal, or blink at 1hz
    sevenSegmentDecimal(item->x,item->y,item->size,wholeNums+decNums,decNums,numColor,data->value);
  }
  head->currentValue = data->value;
}

// For Single Digits with Error and Neutral Displays
void redrawGearPos(screenItemInfo * item, screenItemNode * head){
  volatile dataItem * data = head->data;

  uint8_t stale = millis - data->refreshTime > CAN_TIMEOUT;

  // Set value to 8 (impossible value) if stale
  // Will force refresh on error condition change
  if(stale) {
    data->value = 8;
  }

  if(data->value == head->currentValue){
    return;
  }
  uint16_t fillColor = stale ? errorColor : backgroundColor;
  fillRect(item->x, item->y, item->size, item->size * 1.75, fillColor);
  if(!stale) {
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
  head->currentValue = data->value;
}

/*
 * Main redraw function for all shift lights functionality
 * expects a screenItemNode with multiple data items, in the order of:
 * [0] - kill switch state
 * [1] - gear position
 * [2] - RPM
 */
//TODO: Add error cluster
void redrawShiftLights(screenItemInfo * item, screenItemNode * head) {
  volatile dataItem * killSw = head->data;
  volatile dataItem * gearPos = head->next->data;
  volatile dataItem * RPM = head->next->next->data;

  // Kill cluster redraw
  uint8_t kill = killSw->value ? 1 : 0;
  if (kill && !tlc5955_get_cluster_warn(CLUSTER_RIGHT) ||
      !kill && tlc5955_get_cluster_warn(CLUSTER_RIGHT)) {
    if (!tlc5955_get_startup()) {
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, kill, RED, NO_OVR);
    }
  }
  head->currentValue = killSw->value;

  // Set left warning cluster if there are any warnings on screen
  if(warningCount && !tlc5955_get_cluster_warn(CLUSTER_LEFT) ||
      !warningCount && tlc5955_get_cluster_warn(CLUSTER_LEFT)) {
    if(!tlc5955_get_startup()) {
      tlc5955_set_cluster_warn(CLUSTER_LEFT, warningCount, RED, NO_OVR);
    }
  }

  uint8_t num_leds = getShiftLightsRevRange(RPM->value, gearPos->value);
  if (num_leds == 10) {
    tlc5955_set_main_blink(1, GRN, NO_OVR);
  } else {
    uint8_t i;
    uint64_t colorArray[9] = {0};
    uint64_t drawColor = RED;
    for (i = 0; i < num_leds; i++) {
      colorArray[i] = drawColor;
    }
    tlc5955_set_main_blink(0, 0x0, NO_OVR);
    tlc5955_write_main_colors(colorArray);
  }
  head->next->currentValue = gearPos->value;
  head->next->next->currentValue = RPM->value;
}

uint8_t getShiftLightsRevRange(uint16_t rpm, uint8_t gear) {
  if(gear < 0 || gear > 6) {
    return 0;
  }

  uint16_t maxRPM = shiftRPM[gear];
  int i;
  for(i = 9; i < 0; i--) {
    if(rpm > maxRPM - shiftLightSub[i]) {
      return i + 1;
    }
  }
  return 0;
}

void clearScreen(void){
  fillScreen(backgroundColor);
}

// Resets the current value of all screen items on a screen
void resetScreenItems(void){
  int i = 0;
  warningCount = 0;
  screen *currScreen = allScreens[screenNumber];
  for(;i<currScreen->len;i++){
    screenItemNode * curr;
    for(curr = &(currScreen->items[i].head); curr; curr = curr->next) {
     curr->currentValue = !(curr->data->value);
     curr->data->warningState = 0;
    }
  }
}

uint8_t initNightMode(uint8_t on){
  if(on){
    if(backgroundColor == RA8875_BLACK){
      warningColor = errorColor = RA8875_RED;
      return 0;
    }
    backgroundColor = RA8875_BLACK;
    foregroundColor = RA8875_WHITE;
    foregroundColor2 = RA8875_CYAN;
    warningColor = RA8875_YELLOW;
    errorColor = RA8875_RED;
  }
  else{
    if(backgroundColor == RA8875_WHITE){
      warningColor = RA8875_YELLOW;
      errorColor = RA8875_RED;
      return 0;
    }
    backgroundColor = RA8875_WHITE;
    foregroundColor = RA8875_BLACK;
    foregroundColor2 = RA8875_GREEN;
  }
  warningColor = RA8875_YELLOW;
  errorColor = RA8875_RED;
  return 1;
}

void nightMode(uint8_t on){
  if(initNightMode(on)){
    changeScreen(screenNumber);
  }
}

uint8_t checkDataChange(screenItemNode * head) {
  screenItemNode * curr;
  for(curr = head; curr; curr = curr->next) {
    if(curr->currentValue != curr->data->value) {
      return 1;
    }
  }
  return 0;
}
