/*
  SteeringWheelSketch.pde
  v1.4
  
  Written by: Joe Sheehan
  University of Illinois Formula SAE Team 2011
  
  Target: Atmega328p
  
  This sketch was written for Revision 2 of the LEDControlBoard.
  This sketch was written to interact with Revision 2 of the DataAcqBoard.
  
  Description:
    This sketch monitors the hard switches on the steering wheel and receives
    data from the DataAcqBoard over the I2C bus as a slave, and shifts out
    control data to the display drivers on the LEDBoard. 

  TODO:
    -None
  
  Change Log:
  v1.4
    - Added Oil Temperature readings and display
  v1.3
    - Fixed bug in requestEvent() only sending 2 bytes instead of 4.
  v1.2
    - Modified RPM thresholds in updateShiftLights()
    - Adjusted LED bar scaling to correct range (100)
  v1.1 
    - Added I2C request handler and corresponding message constants.
  v1.0 
    - Initial upload
*/
#include <Wire.h>
#include <LedControl.h>

//Pin Declarations
const int leftDisplayPin = 10;
const int rightDisplayPin = 2;
const int brightnessDownPin = 9;
const int brightnessUpPin = 14;
const int dataPinAlpha = 17;
const int clockPinAlpha = 16;
const int latchPinAlpha = 15;
const int tracPin = 3;
const int fuelMapPin = 4;
const int toggle2Pin = 5;

//Display Constants
const int LEFT = 0;
const int RIGHT = 1;

//Brightness Adj. Constants
const int DOWN = 0;
const int UP = 1;

//Display State Constants
const int RPM_DISPLAY = 0;
const int SPEED_DISPLAY = 1;
const int OIL_P_DISPLAY = 2;
const int OIL_T_DISPLAY = 3;
const int WATER_C_DISPLAY = 4;
const int WATER_F_DISPLAY = 5;
const int LAP_BEST_DISPLAY = 6;
const int LAP_LAST_DISPLAY = 7;
//const int LAP_SPLIT_DISPLAY = 7;

const int TRAC_DISPLAY = 0;
const int ON_DISPLAY = 1;
const int OFF_DISPLAY = 2;
const int FUEL_DISPLAY = 3;
const int RICH_DISPLAY = 4;
const int LEAN_DISPLAY = 5;

//Message Constants
const byte GEAR_CHANGE = 0x01;
const byte RPM_UPDATE = 0x02;   
const byte SPEED_UPDATE = 0x03;
const byte OIL_PRESSURE_UPDATE = 0x04;
const byte OIL_TEMP_UPDATE = 0x05;
const byte WATER_TEMP_UPDATE = 0x06;
const byte LAP_TIME_UPDATE = 0x07;
const byte SPLIT_TIME_UPDATE = 0x08;
const byte TRAC_PIN_UPDATE = 0x09;
const byte FUEL_PIN_UPDATE = 0x0A;
const byte DEMO_MODE = 0xFF;

//These are arrays that define the led states for the 
//  Alphanumeric, shift lights, bar graphs, and seven segs
const byte gearArray[] = {
  0xB6,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x00};
const byte shiftLightArray[] = {
  0x00,0x80,0xC0,0xE0,0xF8,0xFF};
const byte barGraphArray[] = {
  0x00,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF};
const byte displayTexts[8][4] = {
  {0x70,0x77,0x4E,0x37}, //7ACH, TACH
  {0x5B,0x67,0x3D,0x00}, //SPD, SPEED
  {0x7E,0x30,0x0E,0x67}, //OILP, OIL PRESSURE
  {0x7E,0x30,0x0E,0x70}, //OIL7, OIL Temp Farenheit
  {0x37,0x6D,0x7E,0x4E}, //H2OC, Engine Temp Celcius
  {0x37,0x6D,0x7E,0x47}, //H20F, Engine Temp Farenheit
  {0x1F,0x4F,0x5B,0x70}, //BES7, BEST LAP
  {0x0E,0x77,0x5B,0x70}  //LAS7, LAST LAP
  //{0x5B,0x67,0x0E,0x70}  //SPL7, LAST SPLIT TIME
};
const byte otherDisplayTexts[6][4] = {
  {0x70,0x4E,0x5B,0x00}, //TC5, Traction Control System
  {0x7E,0x15,0x00,0x00}, //On
  {0x7E,0x47,0x47,0x00}, //OFF
  {0x47,0x3E,0x4F,0x0E}, //FUEL
  {0x05,0x30,0x4E,0x37}, //rICH, rich fuel map
  {0x0E,0x4F,0x77,0x15}  //LEAn, lean fuel map
};


//Variable Declarations
boolean gearChanged = false;
boolean demoMode = false;
boolean revLimit = false;
boolean displayHold[2] = {false,false};
boolean leftBarBlink = true;
boolean rightBarBlink = true;
boolean noBarBlink = false;

int leftDisplayButtonState = HIGH;
int rightDisplayButtonState = HIGH;
int brightnessDownButtonState = HIGH;
int brightnessUpButtonState = HIGH;
int tracState = HIGH;
int fuelMapState = HIGH;

unsigned long rpmBlinkTimer,textLeftHoldTimer,textRightHoldTimer,leftBlinkTimer,rightBlinkTimer,otherTextHoldTimer;
const long textHoldDelay = 1000; //1000ms or one second

//These don't need to be volatile because they are only assigned in an ISR
byte gear = 7;
int rpm = 0;
byte groundSpeed = 0;
byte oilPressure = 0;
byte oilTemp = 0;
byte waterTemp = 0;
byte shiftLightsState = 0;
byte oilPressureState = 0;
byte waterTempState = 0;
byte brightness = 15;
long lastLap = 0, bestLap = 999999999, splitTime = 0;

int displayStates[2] = {0,1};// [0] is LEFT and [1] is RIGHT. 

//These two unions are used for easy conversion of data received on I2C
union{
  unsigned int var;
  byte buf[2];
} toInt;

union{
  unsigned long var;
  byte buf[4];
} toLong;

//Create an object to control two cascaded MAX7219 chips
//  The library sets the pinModes for us.
LedControl displays = LedControl(11,8,12,2);

void setup(){

  pinMode(leftDisplayPin, INPUT);
  pinMode(rightDisplayPin, INPUT);
  pinMode(brightnessDownPin, INPUT);
  pinMode(brightnessUpPin, INPUT);
  pinMode(tracPin, INPUT);
  pinMode(fuelMapPin, INPUT);
  pinMode(toggle2Pin, INPUT);
  
  pinMode(dataPinAlpha, OUTPUT);
  pinMode(clockPinAlpha, OUTPUT);
  pinMode(latchPinAlpha, OUTPUT);

  //Start MAX7219 chips. Blank the displays.
  for(int index=0;index<displays.getDeviceCount();index++) {
    displays.shutdown(index,false);  
  }
  displays.setIntensity(0,brightness);
  displays.setIntensity(1,15);

  //Initialize the timing variables
  rpmBlinkTimer=textLeftHoldTimer=textRightHoldTimer=leftBlinkTimer=rightBlinkTimer=otherTextHoldTimer = millis();

  //Blink the lights to make it look like a "boot up" sequence
  startupSequence();
  
  //Start I2C hardware
  Wire.begin(2); // join i2c bus with address 2
  Wire.onReceive(receiveEvent); // register receive function
  Wire.onRequest(requestEvent); // register request function

  //Serial.begin(57600);
}

void loop(){
  //Refresh Button States
  int ldp = digitalRead(leftDisplayPin);
  int rdp = digitalRead(rightDisplayPin);
  int bdp = digitalRead(brightnessDownPin);
  int bup = digitalRead(brightnessUpPin);
  int tp = digitalRead(tracPin);
  int fmp = digitalRead(fuelMapPin);
  
  if(ldp != leftDisplayButtonState){
    if(ldp == LOW){
      if(++displayStates[LEFT] > 7)
        displayStates[LEFT] = 0;
      updateText(LEFT);
    }
  }
  if(rdp != rightDisplayButtonState){
    if(rdp == LOW){
      if(++displayStates[RIGHT] > 7)
        displayStates[RIGHT] = 0;
      updateText(RIGHT);
    }
  }
  if(bdp != brightnessDownButtonState){
    if(bdp == LOW)
      adjustBrightness(DOWN);
  }
  if(bup != brightnessUpButtonState){
    if(bup == LOW)
      adjustBrightness(UP); 
  }
  
  if(tp != tracState){
    if(tp == LOW){
      updateOtherText(TRAC_DISPLAY,ON_DISPLAY);
    }
    else
      updateOtherText(TRAC_DISPLAY,OFF_DISPLAY);
  }
  
  if(fmp != fuelMapState){
    if(fmp == LOW){
      updateOtherText(FUEL_DISPLAY,LEAN_DISPLAY);
    }
    else
      updateOtherText(FUEL_DISPLAY,RICH_DISPLAY);
  }
  
  //Update button states
  leftDisplayButtonState = ldp;
  rightDisplayButtonState = rdp;
  brightnessDownButtonState = bdp;
  brightnessUpButtonState = bup;
  tracState = tp;
  fuelMapState = fmp;

  //This loop determines which variables should be displayed on the left 
  // and right seven segment displays based on the driver's display 
  // selections and calls the appropriate function to convert that variable
  // into seven segment data and display it on either display. 
  for(int i=0; i < 2; i++){
    //If we are displaying a newly selected parameter, this
    //  will prevent it from being immediately overwritten.
    if(displayHold[i] == true)
      continue;
    switch(displayStates[i]){
      case RPM_DISPLAY:
        updateRPM(i);
        break;
      case SPEED_DISPLAY:
        updateSpeed(i);
        break;
      case OIL_P_DISPLAY:
        updateOilPDisplay(i);
        break;
      case OIL_T_DISPLAY:
        updateOilTDisplay(i);
        break;  
      case WATER_C_DISPLAY:
        updateTempDisplay(i,'C');
        break;
      case WATER_F_DISPLAY:
        updateTempDisplay(i,'F');
        break;
      case LAP_BEST_DISPLAY:
        updateTimeDisplay(i,'B');
        break;
      case LAP_LAST_DISPLAY:
        updateTimeDisplay(i,'L');
        break;
      //case LAP_SPLIT_DISPLAY:
      //  updateTimeDisplay(i,'S');
      //  break;
      default:
        break;
    }
  }

  //Only update the alphanumeric display when we receive a different gear
  // over the I2C bus. This saves needless data shifting.
  if(gearChanged){
    digitalWrite(latchPinAlpha, LOW);
    shiftOut(dataPinAlpha, clockPinAlpha, MSBFIRST, gearArray[gear]);
    digitalWrite(latchPinAlpha, HIGH);
    gearChanged = false;
  }
  
  //Self explanatory
  updateShiftLights();
  if(revLimit)
    blinkShiftLights();
  
  //This code overtakes the left led bar display when the driver has 
  // pushed the left display cycle button. While the text is being held
  // in the corresponding seven segment, the appropriate LED in the sequence
  // will blink at .1 second intervals. The "noBarBlink" parameter is for 
  // use with text displays that don't utilize a banked selection, i.e.
  // the TCS ON/OFF text.
  if((displayHold[LEFT] == true)&&(noBarBlink == false)){
    if(millis() - leftBlinkTimer > 100){
      leftBarBlink ^= 1;
      displays.setLed(0,2,displayStates[LEFT],leftBarBlink);
      leftBlinkTimer = millis();
    }
    //When our test hold timer has expired, clear the led bar.
    if(millis() - textLeftHoldTimer > textHoldDelay){
      displayHold[LEFT] = false;
      for(int i=0; i < 4; i++)
        displays.setRow(1,i,0x00);
    }
  }
  //Otherwise, the left led bar always displays oil pressure.
  else
    updateOilBar();
    
  //See above!
  if((displayHold[RIGHT] == true)&&(noBarBlink == false)){
    if(millis() - rightBlinkTimer > 100){
      rightBarBlink ^= 1;
      int reverseRight = 7-displayStates[RIGHT];
      displays.setLed(0,3,abs(reverseRight),rightBarBlink);
      rightBlinkTimer = millis();
    }
    if(millis() - textRightHoldTimer > textHoldDelay){
      displayHold[RIGHT] = false;
      for(int i=4; i < 8; i++)
        displays.setRow(1,i,0x00);
    }
  }
  else
    updateTempBar();
    
  //Takes care of holding other text on the displays for the specified hold time.
  if(noBarBlink == true){
    if(millis() - otherTextHoldTimer > textHoldDelay){
      displayHold[RIGHT] = displayHold[LEFT] = false;
      noBarBlink = false;
      displays.clearDisplay(1);
    }
  }
  
  //Something resembling a demo mode
  while(demoMode){
    startupSequence();
  } 
  //Serial.println("loop");
}

//ISR registered to an I2C data reception. See the native Wire library
void receiveEvent(int howMany){
  //Serial.println("Receive Event");
  //howMany should be a multiple of 2 except in the case of rpm and laptime
  byte buffer[howMany];
  //Buffer all receivable data first
  for(int i=0; i<howMany; i++){
    buffer[i] = Wire.receive();
  }
  //Then, iterate through the buffer and process it all. All messages are a pre-determined length.
  int index = 0;
  while(index < howMany){
    switch(buffer[index]){
    case GEAR_CHANGE:
      gearChanged = true;
      gear = buffer[index+1];
      break;
    case RPM_UPDATE:
      toInt.buf[0] = buffer[index+1];
      toInt.buf[1] = buffer[index+2];
      rpm = toInt.var;
      index++; // To cover the extra data byte
      break;
    case SPEED_UPDATE:
      groundSpeed = buffer[index+1];
      break;
    case OIL_PRESSURE_UPDATE:
      oilPressure = buffer[index+1];
      break;
    case OIL_TEMP_UPDATE:
      oilTemp = buffer[index+1];
      break;
    case WATER_TEMP_UPDATE:
      waterTemp = buffer[index+1];
      break;
    case LAP_TIME_UPDATE:
      toLong.buf[0] = buffer[index+1];
      toLong.buf[1] = buffer[index+2];
      toLong.buf[2] = buffer[index+3];
      toLong.buf[3] = buffer[index+4];      
      lastLap = toLong.var;
      if(lastLap < bestLap)
        bestLap = lastLap;
      index+=3; // To cover the extra 3 data bytes
      break;
    case SPLIT_TIME_UPDATE:
      toLong.buf[0] = buffer[index+1];
      toLong.buf[1] = buffer[index+2];
      toLong.buf[2] = buffer[index+3];
      toLong.buf[3] = buffer[index+4];      
      splitTime = toLong.var;
      index+=3; // To cover the extra 3 data bytes
      break;
    case DEMO_MODE:
      if(demoMode)
        demoMode = false;
      else
        demoMode = true;
      break;
    default:
      break;
    }
    //Jump to the next command in the buffer
    index+=2;
  }
}

void requestEvent(){
  //Serial.println("Request Event");
  //Wire.beginTransmission(1);
  Wire.send(TRAC_PIN_UPDATE);
  Wire.send((byte)tracState);
  Wire.send(FUEL_PIN_UPDATE);
  Wire.send((byte)fuelMapState);
  //Wire.endTransmission(); 
}

/* Function: adjustBrightness
 * 
 * Parameters: adjustment - either a 0 or 1 signaling increase or decrease
 *
 * Description: Adjusts brightness in one increment from 1 to 15. 
 */
void adjustBrightness(int adjustment){
  if(adjustment == UP){
    if(++brightness > 15)
      brightness = 15;
  }
  if(adjustment == DOWN){
    if(--brightness < 1)
      brightness = 1;
  }
  displays.setIntensity(0,brightness);
}

/* Function: updateText
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Writes text output to the seven segment displays, sets the 
 *              corresponding LED bar to all on, and begins the hold and 
 *              blink timers. 
 */
void updateText(int whichDisplay){
  if(whichDisplay == LEFT){
    displays.setRow(1,3,displayTexts[displayStates[whichDisplay]][0]);
    displays.setRow(1,2,displayTexts[displayStates[whichDisplay]][1]);
    displays.setRow(1,1,displayTexts[displayStates[whichDisplay]][2]);
    displays.setRow(1,0,displayTexts[displayStates[whichDisplay]][3]);
    displays.setRow(0,2,0xFF);
    textLeftHoldTimer = leftBlinkTimer = millis();
    displayHold[whichDisplay] = true;
  }
  else{
    displays.setRow(1,7,displayTexts[displayStates[whichDisplay]][0]);
    displays.setRow(1,6,displayTexts[displayStates[whichDisplay]][1]);
    displays.setRow(1,5,displayTexts[displayStates[whichDisplay]][2]);
    displays.setRow(1,4,displayTexts[displayStates[whichDisplay]][3]);
    displays.setRow(0,3,0xFF);
    textRightHoldTimer = rightBlinkTimer = millis();
    displayHold[whichDisplay] = true;
  } 
  
}

/* Function: updateOtherText
 * 
 * Parameters: leftDisplayIndex - index into otherDisplayTexts array
 *             rightDisplayIndex - "                               "
 *
 * Description: Writes the other text options to both of the seven 
 *              segments, as they arise.  Text is selected by the inputs. 
 */
void updateOtherText(int leftDisplayIndex, int rightDisplayIndex){
  //LEFT Display
  displays.setRow(1,3,otherDisplayTexts[leftDisplayIndex][0]);
  displays.setRow(1,2,otherDisplayTexts[leftDisplayIndex][1]);
  displays.setRow(1,1,otherDisplayTexts[leftDisplayIndex][2]);
  displays.setRow(1,0,otherDisplayTexts[leftDisplayIndex][3]);
  //RIGHT Display
  displays.setRow(1,7,otherDisplayTexts[rightDisplayIndex][0]);
  displays.setRow(1,6,otherDisplayTexts[rightDisplayIndex][1]);
  displays.setRow(1,5,otherDisplayTexts[rightDisplayIndex][2]);
  displays.setRow(1,4,otherDisplayTexts[rightDisplayIndex][3]);
  //Only one timer is needed since we are controlling both displays
  otherTextHoldTimer = millis();
  displayHold[LEFT] = displayHold[RIGHT] = true; 
  noBarBlink = true;
}

/* Function: updateRPM
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Converts the RPM variable received from the data board into
 *              a four digit display for use on a 7-seg and displays it. 
 */
void updateRPM(int whichDisplay){
  byte tenThousands, thousands, hundreds, tens;

  tenThousands = (byte)(rpm/10000);
  thousands = (byte)((rpm%10000)/1000);
  hundreds = (byte)((rpm%1000)/100);
  tens = (byte)((rpm%100)/10);

  if(whichDisplay == LEFT){
    displays.setDigit(1,3,tenThousands,false);
    displays.setDigit(1,2,thousands,true);
    displays.setDigit(1,1,hundreds,false);
    displays.setDigit(1,0,tens,false);
  }
  else{
    displays.setDigit(1,7,tenThousands,false);
    displays.setDigit(1,6,thousands,true);
    displays.setDigit(1,5,hundreds,false);
    displays.setDigit(1,4,tens,false);
  }
}

void updateShiftLights(){
  if(rpm <= 12500)
    revLimit = false;
  else if(revLimit == true)
    return;
  else;
  if(rpm < 8000)
    shiftLightsState = shiftLightArray[0];
  else{
    if(rpm >= 8000)
      shiftLightsState = shiftLightArray[1];
    if(rpm > 8500)
      shiftLightsState = shiftLightArray[2];
    if(rpm > 10000)
      shiftLightsState = shiftLightArray[3];
    if(rpm > 11000)
      shiftLightsState = shiftLightArray[4];
    if(rpm > 11500)
      shiftLightsState = shiftLightArray[5];
    if(rpm > 12000)
      revLimit = true;
  }

  displays.setRow(0,0,shiftLightsState);
  displays.setRow(0,1,shiftLightsState);
}

void blinkShiftLights(){
  if((millis()-rpmBlinkTimer) > 70){
    shiftLightsState ^= 0xFF;
    displays.setRow(0,0,shiftLightsState);
    displays.setRow(0,1,shiftLightsState);
    rpmBlinkTimer = millis();
  }
}

/* Function: updateSpeed
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Converts the groundSpeed variable received from the data board into
 *              a 2 digit display for use on a 7-seg and displays it. 
 */
void updateSpeed(int whichDisplay){
  byte tens, ones;
  
  tens = (byte)((groundSpeed%100)/10);
  ones = (byte)(groundSpeed%10);

  if(whichDisplay == LEFT){
    displays.setDigit(1,1,tens,false);
    displays.setDigit(1,0,ones,false);
  }
  else{
    displays.setDigit(1,5,tens,false);
    displays.setDigit(1,4,ones,false);
  }
}

/* Function: updateOilBar
 * 
 * Parameters: none
 *
 * Description: Converts the oilPressure variable into an 8 step scale and
 *              displays it on the left led bar display.
 */
void updateOilBar(){
  oilPressureState = map(oilPressure,0,100,1,8);
  displays.setRow(0,2,barGraphArray[oilPressureState]);
}

/* Function: updateOilPDisplay
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Converts the oilPressure variable received from the data board into
 *              a 2 digit display for use on a 7-seg and displays it. 
 */
void updateOilPDisplay(int whichDisplay){
  byte tens, ones;
  
  tens = (byte)((oilPressure%100)/10);
  ones = (byte)(oilPressure%10);
  
  if(whichDisplay == LEFT){
    displays.setDigit(1,1,tens,false);
    displays.setDigit(1,0,ones,false);    
  }
  else{
    displays.setDigit(1,5,tens,false);
    displays.setDigit(1,4,ones,false);
  }
}

/* Function: updateOilTDisplay
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Converts the oilTemp variable received from the data board into
 *              a 3 digit display for use on a 7-seg and displays it. 
 */
void updateOilTDisplay(int whichDisplay){
  byte hundreds, tens, ones, temp;
  
  hundreds = (byte)((oilTemp%1000)/100);
  tens = (byte)((oilTemp%100)/10);
  ones = (byte)(oilTemp%10);
  
  if(whichDisplay == LEFT){
    if(hundreds != 0)
      displays.setDigit(1,2,hundreds,false);
    else
      displays.setRow(1,2,0x00);
    displays.setDigit(1,1,tens,false);
    displays.setDigit(1,0,ones,false);    
  }
  else{
    if(hundreds != 0)
      displays.setDigit(1,6,hundreds,false);
    else
      displays.setRow(1,6,0x00);
    displays.setDigit(1,5,tens,false);
    displays.setDigit(1,4,ones,false);
  }
}

/* Function: updateTempBar
 * 
 * Parameters: none
 *
 * Description: Converts the waterTemp variable into an 8 step scale and
 *              displays it on the right led bar display.
 */
void updateTempBar(){
  waterTempState = map(waterTemp,0,100,1,8);
  displays.setRow(0,3,barGraphArray[waterTempState]);
}

/* Function: updateTempDisplay
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *             scale - either 'F' or 'C' for Fahrenheit or Celsius 
 *
 * Description: Converts the waterTemp variable received from the data board into
 *              a 3 digit display for use on a 7-seg and displays it. 
 */
void updateTempDisplay(int whichDisplay, char scale){
  byte hundreds, tens, ones, temp;
  
  if(scale == 'F')
    temp = (1.8*waterTemp)+32;
  else
    temp = waterTemp;
  
  hundreds = (byte)((temp%1000)/100);
  tens = (byte)((temp%100)/10);
  ones = (byte)(temp%10);
  
  if(whichDisplay == LEFT){
    if(hundreds != 0)
      displays.setDigit(1,2,hundreds,false);
    else
      displays.setRow(1,2,0x00);
    displays.setDigit(1,1,tens,false);
    displays.setDigit(1,0,ones,false);    
  }
  else{
    if(hundreds != 0)
      displays.setDigit(1,6,hundreds,false);
    else
      displays.setRow(1,6,0x00);
    displays.setDigit(1,5,tens,false);
    displays.setDigit(1,4,ones,false);
  }
}

/* Function: updateTimeDisplay
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *             selection - chooses which stored time to convert for display
 *
 * Description: Converts a time variable received from the data board into
 *              a four digit display for use on a 7-seg and displays it. 
 *
 *              Capable of displaying a time of less than 10 minutes in a 
 *              min.secs.tenths format, i.e. 1.20.5 would be 1min, 20.5secs
 */
void updateTimeDisplay(int whichDisplay, char selection){
  byte minutes, secondTens, secondOnes, tenths;
  long time;
  
  if(selection == 'L')
    time = lastLap;
  if(selection == 'B')
    time = bestLap;
  if(selection == 'S')
    time = splitTime;
  
  minutes = (byte)(((time/1000)/60)%10);
  secondTens = (byte)((time%60000)/10000);
  secondOnes = (byte)((time%10000)/1000);
  tenths = (byte)((time%1000)/100);
  
  if(whichDisplay == LEFT){
    displays.setDigit(1,3,minutes,true);
    displays.setDigit(1,2,secondTens,false);
    displays.setDigit(1,1,secondOnes,true);
    displays.setDigit(1,0,tenths,false);    
  }
  else{
    displays.setDigit(1,7,minutes,true);
    displays.setDigit(1,6,secondTens,false);
    displays.setDigit(1,5,secondOnes,true);
    displays.setDigit(1,4,tenths,false);    
  }
}


