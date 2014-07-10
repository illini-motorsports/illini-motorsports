/*
  DataAcquisitionSketch.pde
  v2.1
  
  Written by: Joe Sheehan
  University of Illinois Formula SAE Team 2011
  
  Target: Atmega2560
  
  This sketch was written for Revision 2 of the DataAcquisitionBoard.
  This sketch was written to interact with Revision 2 of the LEDControlBoard.
  
  Description:
    This sketch monitors the AUX ouputs from the Motec M400 ECU, as well
    as a few other inputs from sensors around the car, processes the 
    data, and sends it off to the wheel as well as a PC and the onboard 
    datalogger, if enabled.

  TODO:
    - Add lap beacon interrupt
    - Add a robust handshaking method between the car and GUI
  
  Change Log:
  v2.1 
    - Added Demo Mode functionality
  v2.0 
    - Switched from using PWM inputs to using a data stream from the 
      M400's RS232C connection. Provides faster and more reliable operation.
    - Added a manual timeout to the I2C Request function.
  v1.1
    - Finished comments and added computeGear()
  v1.0 
    - Initial upload
*/

#include <Wire.h>
#include <aJSON.h>

//#define DEBUG 1

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

//ECU Constants
const int NUM_CHANNELS = 11;
const int ECU_MSG_LENGTH = (NUM_CHANNELS*2) + 8; //8 = 4 header and 4 CRC bytes
const int rpm = 0;
const int waterTemperature = 1;
const int oilPressure = 2;
const int oilTemperature = 3;
const int throttlePosition = 4;
const int frWSS = 5;
const int flWSS = 6;
const int rrWSS = 7;
const int rlWSS = 8;
const int lambda = 9;
const int iap = 10;

int channelData[NUM_CHANNELS] = {0};

byte ECUBuffer[ECU_MSG_LENGTH] = {0};
char CRCBuffer[ECU_MSG_LENGTH-3] = {0};

long rpm_per_mph[] = {339.85, 247.39, 197.56, 170.45, 152.66, 140.81};
int gear_hist[25] = {0};

boolean messageReceived = false;

//Pin Declarations

//Add new pins to the arrays below for initialization.
  //Inputs
const int lapBeaconPin = 2;  
const int neutralPosPin = 19;
  //Outputs
const int resetPin = 12;
const int stopPin = 13;
const int powerLEDPin = 40;
const int readyLEDPin = 41;
const int status2LEDPin = 42;
const int status1LEDPin = 43;
const int rightfrontpot = A10;
const int leftfrontpot = A11;
const int backleftpot = A12;
const int rightbackpot = A13;

//Pin Arrays
const byte inputPins[] = {
  2,19};
const byte outputPins[] = {
  12,13,40,41,42,43};

//Communication Control Messages - (Temporary)
String startLogging = "BEGIN";
String stopLogging = "HALT";
String resetLogging = "RESET";
String aliveQuery = "HELLO?";
String aliveAcknowledge = "FOUND ME!";

//Variable Declarations
boolean dataEnabled = false;
boolean TCSSwitchChanged = false;
boolean fuelMapSwitchChanged = false;
boolean gearChanged = false;
boolean timesChanged = false;
boolean demoMode = false;

byte tractionState;
byte fuelMapState;
byte gear = -1;

int frontBrakePressure, rearBrakePressure;
int drivenSpeed, undrivenSpeed;

// LFD = left front disp.
// RFD = right front disp.
// LBD = left back disp.
// RBD = right back disp.

int RFD, LFD, LBD, RBD; 

long timerStart;
long lastLap, splitTime;

//Data structures for the benefit of I2C bus transmission and ECU message reception.
union{
  unsigned int var;
  byte buf[2];
} FromInt;

union{
  unsigned long var;
  byte buf[4];
} FromLong;

void setup(){
  initializePins();
  digitalWrite(resetPin, LOW);
  digitalWrite(stopPin, HIGH);
  digitalWrite(powerLEDPin, HIGH);
  digitalWrite(neutralPosPin, HIGH); //Enables internal pullup
  
  //attachInterrupt(0,lapTime,FALLING);
  //attachInterrupt(1,neutralPosition,CHANGE); 

  //XBee
  Serial3.begin(57600);
  //DataLogger
  Serial2.begin(57600);
  //For Debugging
  Serial.begin(115200);
  Wire.begin(); 
 
  timerStart = millis();
}

void loop(){
  if(Serial3.available() > 0){
    parseSerialInput();
  }
  
  toggleLED(readyLEDPin);
  
  takeReadings();
  //requestSwitchStates();
  computeCurrentGear();
  updateWheel();
  
  if(dataEnabled){
    sendReadings();
  }
  
  toggleLED(readyLEDPin);
  //delay(5);
}

/* Function: parseSerialInput
 * 
 * Parameters: none
 *
 * Description: Called when data appears from the XBee. Decodes the 
 *              incoming messages.
 */
void parseSerialInput(){
  
  int cmd = Serial3.read();
  switch(cmd){
  case '0':
    enableLogging(false);
    Serial3.println(stopLogging);
    break;
  case '1':
    enableLogging(true);
    Serial3.println(startLogging);
    break;
  case '2':
    resetLogger();
    Serial3.println(resetLogging);
    break;
  case 'A':
    Serial3.println(aliveAcknowledge);
    break;
  case 'D':
    demoMode = true;
    break;
  default:
    break;
  } 
}

/* Function: takeReadings
 * 
 * Parameters: none
 *
 * Description: Reads PWM inputs from the Motec M400 and converts 
 *              the data into the scaled readings for the wheel.
 */
void takeReadings(){
  toggleLED(status1LEDPin);
  //unsigned long checksum;
  
  Serial1.begin(38400);
  while(Serial1.available()){
    if(Serial1.peek() == 0x82){
      for(int i = 0; i < ECU_MSG_LENGTH; i++){
        while(!Serial1.available());
        ECUBuffer[i] = Serial1.read();
      }
      Serial1.end();
      Serial1.flush();
      
      if((ECUBuffer[0]!=0x82)||(ECUBuffer[1]!=0x81)||(ECUBuffer[2]!=0x80)||(ECUBuffer[3]!=NUM_CHANNELS))
        return;
      else      
        messageReceived = true;
      
      int index = 0;
      if(messageReceived){
        for(int i = 0; i < (NUM_CHANNELS*2); i+=2){
          FromInt.buf[1] = ECUBuffer[i+4];
          FromInt.buf[0] = ECUBuffer[i+5];
          if(index == 0)//RPM is the only parameter without 1/10 accuracy
            channelData[index++] = FromInt.var; 
          else
            channelData[index++] = FromInt.var/10; 
        }
      }
    }
    else
      Serial1.read(); //discard byte
  }
  
  frontBrakePressure = map(analogRead(0),0,1023,0,2600);
  rearBrakePressure = map(analogRead(1),0,1023,0,2600);
 
  RFD = (analogRead(rightfrontpot),0,1023);
  LFD = (analogRead(leftfrontpot),0,1023);
  LBD = (analogRead(leftbackpot),0,1023);
  RBD = (analogRead(rightbackpot),0,1023);
  
  drivenSpeed = (channelData[rrWSS]+channelData[rlWSS])/2;
  undrivenSpeed = (channelData[frWSS]+channelData[flWSS])/2;
  //Scale Oil Pressure and Temperature from kPa and C to PSI and F
  //channelData[oilPressure] *= 0.0145;
  //channelData[oilTemperature] = (1.8*channelData[oilTemperature])+32;
  
  #ifdef DEBUG
  if(messageReceived){
    /*
    for(int i = 0; i < ECU_MSG_LENGTH; i++){
      Serial.print(ECUBuffer[i], HEX);
      Serial.print("::");
    }
    Serial.println();
      */
    for(int i = 0; i < (NUM_CHANNELS); i++){
      Serial.print(channelData[i],DEC);
      Serial.print("::"); 
    }
    Serial.println();
    
  }
  #endif
  
  messageReceived = false;
  
  toggleLED(status1LEDPin);
}

/* Function: sendReadings
 * 
 * Parameters: none
 *
 * Description: Creates a JSON object string of all the stored variables
 *              and sends it off to both the XBee and the data logger.
 */
void sendReadings(){
  
  toggleLED(status2LEDPin);  
  
  aJsonObject* root = aJson.createObject();
    
  //aJson.addItemToObject(root,"groundSpeed", aJson.createItem(channelData[groundSpeed]));
  //aJson.addItemToObject(root,"drivenSpeed", aJson.createItem(channelData[drivenSpeed]));
  aJson.addItemToObject(root,"rpm", aJson.createItem(channelData[rpm]));
  aJson.addItemToObject(root,"tp", aJson.createItem(channelData[throttlePosition]));
  aJson.addItemToObject(root,"op", aJson.createItem(channelData[oilPressure]));
  aJson.addItemToObject(root,"ot", aJson.createItem(channelData[oilTemperature]));  
  aJson.addItemToObject(root,"et", aJson.createItem(channelData[waterTemperature]));
  //aJson.addItemToObject(root,"frontBrake", aJson.createItem(frontBrakePressure));
  //aJson.addItemToObject(root,"rearBrake", aJson.createItem(rearBrakePressure));
  aJson.addItemToObject(root,"fr", aJson.createItem(channelData[frWSS]));
  aJson.addItemToObject(root,"fl", aJson.createItem(channelData[flWSS]));
  aJson.addItemToObject(root,"rr", aJson.createItem(channelData[rrWSS]));
  aJson.addItemToObject(root,"rl", aJson.createItem(channelData[rlWSS]));
  aJson.addItemToObject(root,"O2", aJson.createItem(channelData[lambda]));
  aJson.addItemToObject(root,"map", aJson.createItem(channelData[iap]));
  aJson.addItemToObject(root,"time", aJson.createItem((long)(millis()-timerStart)));
 
  aJson.addItemToObject(root,"rightfrontpot", aJson.createItem(channelData[RFD]));
  aJson.addItemToObject(root,"leftfrontpot", aJson.createItem(channelData[LFD]));
  aJson.addItemToObject(root,"leftbackpot", aJson.createItem(channelData[LBD]));
  aJson.addItemToObject(root,"rightbackpot", aJson.createItem(channelData[RBD]));
     
  /*
  if(TCSSwitchChanged){
    aJson.addItemToObject(root,"TCS", aJson.createItem(tractionState));
    TCSSwitchChanged = false;
  }
  if(fuelMapSwitchChanged){
    aJson.addItemToObject(root,"fuelmap", aJson.createItem(fuelMapState)); 
    fuelMapSwitchChanged = false;
  }
  */
    
  char* string = aJson.print(root);
  if (string != NULL) {
    Serial2.println(string);
    Serial3.println(string);
  } 
  free(string);
  aJson.deleteItem(root);
  
  toggleLED(status2LEDPin);
}

/* Function: updateWheel
 * 
 * Parameters: none
 *
 * Description: Queues up all the data to send to the wheel and sends it. 
 */
void updateWheel(){                    
  
  //toggleLED(status2LEDPin);
  
  Wire.beginTransmission(2);
  FromInt.var = channelData[rpm]; 
  Wire.send(RPM_UPDATE);
  Wire.send((byte*)FromInt.buf,2);
  Wire.send(SPEED_UPDATE);
  Wire.send((byte)(channelData[flWSS]/2+channelData[frWSS]/2));
  Wire.send(OIL_PRESSURE_UPDATE);
  Wire.send((byte)channelData[oilPressure]);
  Wire.send(OIL_TEMP_UPDATE);
  Wire.send((byte)channelData[oilTemperature]);
  Wire.send(WATER_TEMP_UPDATE);
  Wire.send((byte)channelData[waterTemperature]);
  //if(gearChanged){
    Wire.send(GEAR_CHANGE);
    Wire.send(gear);
    gearChanged = false;
  //}
  if(timesChanged){
    FromLong.var = lastLap;
    Wire.send(LAP_TIME_UPDATE);
    Wire.send((byte*)FromLong.buf,4);
    FromLong.var = splitTime;
    Wire.send(SPLIT_TIME_UPDATE);
    Wire.send((byte*)FromLong.buf,4);
    timesChanged = false;
  }
  if(demoMode){
    Wire.send(DEMO_MODE);
    Wire.send(0xFF); //filler byte
    demoMode = false;
  }
  Wire.endTransmission();
  
  
  //toggleLED(status2LEDPin);
}

/* Function: enableLogging
 * 
 * Parameters: dataStreamEnabled - true to enable logging
 *
 * Description: Starts or stops data logger, resets. 
 */
void enableLogging(boolean dataStreamEnabled){
  if(dataStreamEnabled){
    dataEnabled = true;
    digitalWrite(stopPin, HIGH);
    digitalWrite(resetPin, HIGH);
    digitalWrite(readyLEDPin, HIGH);
    resetLogger();
  }
  else{
    dataEnabled = false;
    digitalWrite(resetPin, HIGH);
    digitalWrite(stopPin, LOW);
    digitalWrite(readyLEDPin, LOW);
    delay(100);
    digitalWrite(stopPin, HIGH);
  }
  timerStart = millis();
}

/* Function: resetLogger
 * 
 * Parameters: none
 *
 * Description: Umm....yeah. 
 */
void resetLogger(){
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  delay(1500);
}

/* Function: initializePins
 * 
 * Parameters: none
 *
 * Description: Iterates through the global input and output pin 
 *              arrays and sets their pinModes. 
 */
void initializePins(){
  int i;
  for(i = 0; i < sizeof(inputPins); i++){
    pinMode(inputPins[i],INPUT);
  }
  for(i = 0; i < sizeof(outputPins); i++){
    pinMode(outputPins[i],OUTPUT);
  }
}

/* Function: toggleLED
 * 
 * Parameters: pin - the output pin to toggle state on
 *
 * Description: Toggles state on the input parameter's output. 
 */
void toggleLED(int pin){
  int stat = digitalRead(pin);
  stat ^= 1;
  digitalWrite(pin, stat);
}

/* Function: requestSwitchStates
 * 
 * Parameters: none
 *
 * Description: Polls the LEDControlBoard attached for the states
 *              of the two toggle switches at the bottom of the wheel. 
 */
void requestSwitchStates(){
  byte buffer[4];
  
  Wire.requestFrom(2,4);
  
  Serial.println("Requesting");
  
  unsigned long timeout = millis();
  while(Wire.available() < 4){
    if(millis()-timeout > 20) //20ms timeout
      return;
  }
  Serial.println("GOT DATA!");
  for(int i = 0; i < 4; i++){
    buffer[i] = Wire.receive();
  }
  int index = 0;
  while(index < 4){
    switch(buffer[index]){
    case TRAC_PIN_UPDATE:
      tractionState = buffer[index+1];
      TCSSwitchChanged = true;
      break;
    case FUEL_PIN_UPDATE:
      fuelMapState = buffer[index+1];
      fuelMapSwitchChanged = true;
      break;
    default:
      break;
    }
    index+=2;
  }
}

/* Function: computeCurrentGear
 * 
 * Parameters: none
 *
 * Description: Calculates current gear based on rear wheel speed 
 *              and engine rpm. 
 */
void computeCurrentGear(){
  /*byte closest_gear=0;
  byte computedGear;
  if(digitalRead(neutralPosPin) == LOW) {
    computedGear = 0;
  }
  else if(drivenSpeed < 1){
    computedGear = 1;
  }
  else{
    long ratio = channelData[rpm]/drivenSpeed;
    long perc_diff[6];
    for(int i=0; i<6; i++){
      perc_diff[i] = 2*fabs(ratio-rpm_per_mph[i])/(ratio+rpm_per_mph[i]);
      if (perc_diff[i] < perc_diff[closest_gear])
        closest_gear = i;
    }

    int count[6];
    for(int i=0; i<24; i++) {
      gear_hist[i]=gear_hist[i+1];
      count[gear_hist[i]]++;
    }	

    gear_hist[24] = closest_gear;
    count[closest_gear]++;
    
    computedGear=gear;
    for(int i=0; i<6;i++){
      if(count[i]>count[computedGear])
      computedGear=i;
    }
  }
  
  //if(computedGear != gear){
    gearChanged = true;
    gear = computedGear;
  //}
  
  Serial.println((int)gear);
  //Serial.println(digitalRead(neutralPosPin));*/
  gearChanged = true;
  gear =0;
  
}


