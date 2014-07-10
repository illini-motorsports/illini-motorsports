void startupSequence(){
  //Flashing LEDs here!

  //GOING UP!
  //1
  displays.setRow(0,0,0x80);
  displays.setRow(0,1,0x80);
  displays.setRow(0,2,0x80);
  displays.setRow(0,3,0x80);
  displays.setChar(1,4,'H',false);
  changeGear();
  delay(100);
  //2
  displays.setRow(0,0,0xC0);
  displays.setRow(0,1,0xC0);
  displays.setRow(0,2,0xC0);
  displays.setRow(0,3,0xC0);
  changeGear();
  delay(100);
  //3
  displays.setRow(0,0,0xE0);
  displays.setRow(0,1,0xE0);
  displays.setRow(0,2,0xE0);
  displays.setRow(0,3,0xE0);
  displays.setChar(1,5,'H',false);
  displays.setChar(1,4,'E',false);
  changeGear();
  delay(100);
  //4
  displays.setRow(0,0,0xF0);
  displays.setRow(0,1,0xF0);
  displays.setRow(0,2,0xF0);
  displays.setRow(0,3,0xF0);
  changeGear();
  delay(100);
  //5
  displays.setRow(0,0,0xF8);
  displays.setRow(0,1,0xF8);
  displays.setRow(0,2,0xF8);
  displays.setRow(0,3,0xF8);
  displays.setChar(1,6,'H',false);
  displays.setChar(1,5,'E',false);
  displays.setChar(1,4,'L',false);
  changeGear();
  delay(100);
  //6
  displays.setRow(0,0,0xFC);
  displays.setRow(0,1,0xFC);
  displays.setRow(0,2,0xFC);
  displays.setRow(0,3,0xFC);
  changeGear();
  delay(100);
  //7
  displays.setRow(0,0,0xFE);
  displays.setRow(0,1,0xFE);
  displays.setRow(0,2,0xFE);
  displays.setRow(0,3,0xFE);
  displays.setChar(1,7,'H',false);
  displays.setChar(1,6,'E',false);
  displays.setChar(1,5,'L',false);
  displays.setChar(1,4,'L',false);
  changeGear();
  delay(100);
  //8
  displays.setRow(0,0,0xFF);
  displays.setRow(0,1,0xFF);
  displays.setRow(0,4,0xC0);
  displays.setRow(0,2,0xFF);
  displays.setRow(0,3,0xFF);
  changeGear();
  delay(100);
  //////////////////////////
  // GOING DOWN!          //
  //////////////////////////
  displays.setRow(0,0,0xFE);
  displays.setRow(0,1,0xFE);
  displays.setRow(0,4,0x00);
  displays.setRow(0,2,0xFE);
  displays.setRow(0,3,0xFE);
  displays.setChar(1,0,'H',false);
  displays.setChar(1,7,'E',false);
  displays.setChar(1,6,'L',false);
  displays.setChar(1,5,'L',false);
  displays.setChar(1,4,'0',false);
  changeGear();
  delay(100);
  displays.setRow(0,0,0xFC);
  displays.setRow(0,1,0xFC);
  displays.setRow(0,2,0xFC);
  displays.setRow(0,3,0xFC);
  changeGear();
  delay(100);
  displays.setRow(0,0,0xF8);
  displays.setRow(0,1,0xF8);
  displays.setRow(0,2,0xF8);
  displays.setRow(0,3,0xF8);
  displays.setChar(1,1,'H',false);
  displays.setChar(1,0,'E',false);
  displays.setChar(1,7,'L',false);
  displays.setChar(1,6,'L',false);
  displays.setChar(1,5,'0',false);
  displays.setChar(1,4,' ',false);
  changeGear();
  delay(100);
  displays.setRow(0,0,0xF0);
  displays.setRow(0,1,0xF0);
  displays.setRow(0,2,0xF0);
  displays.setRow(0,3,0xF0);
  changeGear();
  delay(100);
  displays.setRow(0,0,0xE0);
  displays.setRow(0,1,0xE0);
  displays.setRow(0,2,0xE0);
  displays.setRow(0,3,0xE0);
  displays.setChar(1,2,'H',false);
  displays.setChar(1,1,'E',false);
  displays.setChar(1,0,'L',false);
  displays.setChar(1,7,'L',false);
  displays.setChar(1,6,'0',false);
  displays.setChar(1,5,' ',false);
  changeGear();
  delay(100);
  displays.setRow(0,0,0xC0);
  displays.setRow(0,1,0xC0);
  displays.setRow(0,2,0xC0);
  displays.setRow(0,3,0xC0);
  changeGear();
  delay(100);
  displays.setRow(0,0,0x80);
  displays.setRow(0,1,0x80);
  displays.setRow(0,2,0x80);
  displays.setRow(0,3,0x80);
  displays.setChar(1,3,'H',false);
  displays.setChar(1,2,'E',false);
  displays.setChar(1,1,'L',false);
  displays.setChar(1,0,'L',false);
  displays.setChar(1,7,'0',false);
  displays.setChar(1,6,' ',false);
  changeGear();
  delay(100);
  displays.setRow(0,0,0x00);
  displays.setRow(0,1,0x00);
  displays.setRow(0,2,0x00);
  displays.setRow(0,3,0x00);
  changeGear();
  delay(100);
  displays.setChar(1,3,'E',false);
  displays.setChar(1,2,'L',false);
  displays.setChar(1,1,'L',false);
  displays.setChar(1,0,'0',false);
  displays.setChar(1,7,' ',false);
  clearGear();
  delay(200);
  displays.setChar(1,3,'L',false);
  displays.setChar(1,2,'L',false);
  displays.setChar(1,1,'0',false);
  displays.setChar(1,0,' ',false);
  delay(200);
  displays.setChar(1,3,'L',false);
  displays.setChar(1,2,'0',false);
  displays.setChar(1,1,' ',false);
  delay(200);
  displays.setChar(1,3,'0',false);
  displays.setChar(1,2,' ',false);
  delay(200);
  displays.setChar(1,3,' ',false);
  delay(100);

  // Flash 3 times!
  for(int i=0;i<4;i++){
    gear = -1;
    changeGear();
    displays.setRow(0,0,0xFF);
    displays.setRow(0,1,0xFF);
    displays.setRow(0,2,0xFF);
    displays.setRow(0,3,0xFF);
    displays.setRow(0,4,0xC0);
    displays.setDigit(1, 0, 0x08, true);
    displays.setDigit(1, 1, 0x08, true);
    displays.setDigit(1, 2, 0x08, true);
    displays.setDigit(1, 3, 0x08, true);
    displays.setDigit(1, 4, 0x08, true);
    displays.setDigit(1, 5, 0x08, true);
    displays.setDigit(1, 6, 0x08, true);
    displays.setDigit(1, 7, 0x08, true);
    delay(100);
    clearGear();
    displays.clearDisplay(0);
    displays.clearDisplay(1);
    delay(100);
  }
  
  gear = -1;
  changeGear();
  displays.setRow(0,0,0xFF);
  displays.setRow(0,1,0xFF);
  displays.setRow(0,2,0xFF);
  displays.setRow(0,3,0xFF);
  displays.setRow(0,4,0xC0);
  displays.setDigit(1, 0, 0x08, true);
  displays.setDigit(1, 1, 0x08, true);
  displays.setDigit(1, 2, 0x08, true);
  displays.setDigit(1, 3, 0x08, true);
  displays.setDigit(1, 4, 0x08, true);
  displays.setDigit(1, 5, 0x08, true);
  displays.setDigit(1, 6, 0x08, true);
  displays.setDigit(1, 7, 0x08, true);

  delay(2000);

  clearGear();
  displays.clearDisplay(0);
  displays.clearDisplay(1);  
}

void changeGear(){
  if(++gear > 6)
    gear = 0;
  digitalWrite(latchPinAlpha, LOW);
  shiftOut(dataPinAlpha, clockPinAlpha, MSBFIRST, gearArray[gear]);
  digitalWrite(latchPinAlpha, HIGH);
}

void clearGear(){
  digitalWrite(latchPinAlpha, LOW);
  shiftOut(dataPinAlpha, clockPinAlpha, MSBFIRST, 0x00);
  digitalWrite(latchPinAlpha, HIGH);
}
