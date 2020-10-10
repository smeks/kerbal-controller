#include "KerbalSimpit.h"
#include "KerbalSimpitMessageTypes.h"
#include <SoftwareSerial.h>


SoftwareSerial mySerial(14,15); //pin 14 connected to LCD, 15 unconnected

//analog pins
const int pTHROTTLE = A0; //slide pot
const int pTX = A1;       //translation x-axis
const int pTY = A2;       //translation y-axis
const int pTZ = A3;       //translation z-axis
const int pRX = A4;       //rotation x-axis
const int pRY = A5;       //rotation y-axis
const int pRZ = A6;       //rotation z-axis

//digital pins
const int pPOWER = 2;       //power switch
const int pTB = 3;          //translation joystick button
const int pRB = 4;          //rotation joystick button
const int latchPin = 8;     //ST_CP - green
const int dataPin = 11;     //DS - yellow
const int clockPin = 12;    //SH_CP - blue
const int pMODE = 22;       //mode switch (used for debug mode)
const int pLCDx = 27;       //toggle switch x (used for LCD display modes)
const int pLCDy = 24;       //toggle switch y (used for LCD display modes)
const int pLCDz = 29;       //toggle switch z (used for LCD display modes)
const int pSAS = 26;        //SAS switch
const int pRCS = 31;        //RCS switch
const int pABORT = 28;      //Abort switch (safety switch, active high)
const int pARM = 30;        //Arm switch (safety switch, active high)
const int pSTAGE = 32;      //Stage button
const int pSTAGELED = 33;   //Stage button LED
const int pLIGHTS = 34;     //Lights button
const int pLIGHTSLED = 35;  //Lights button LED
const int pLADDER = 36;     //Ladder button (action group 5)
const int pLADDERLED = 37;  //Ladder button LED
const int pSOLAR = 38;      //Solar button (action group 6)
const int pSOLARLED = 39;   //Solar button LED
const int pCHUTES = 40;     //Chutes button (action group 7)
const int pCHUTESLED = 41;  //Chutes button LED
const int pGEARS = 42;      //Gears button
const int pGEARSLED = 43;   //Gears button LED
const int pBRAKES = 44;     //Brakes button
const int pBRAKESLED = 45;  //Brakes button LED
const int pACTION1 = 46;    //Action Group 1 button
const int pACTION1LED = 47; //Action Group 1 button LED
const int pACTION2 = 48;    //Action Group 2 button
const int pACTION2LED = 49; //Action Group 2 button LED
const int pACTION3 = 50;    //Action Group 3 button
const int pACTION3LED = 51; //Action Group 3 button LED
const int pACTION4 = 52;    //Action Group 4 button
const int pACTION4LED = 53; //Action Group 4 button LED

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

void setup() {
  // Open the serial connection.
  Serial.begin(115200);
  
  mySerial.begin(9600); //LCD connection
  delay(500);           //wait for LCD boot
  clearLCD();
  writeLCD("KerbalController");
  jumpToLineTwo();
  writeLCD("booting...");

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
  }
  
  controlsInit();
  testLEDS(50);

  clearLCD();
  writeLCD("SIMPIT CONNECTED");
}


void debugAnalog(int pin)
{
  int data = analogRead(pin);     // read the input pin
  char achar[5];
  itoa(data, achar, 10);
  writeLCD(achar);
}

void loop() {
  clearLCD();
  // Read the state of the switch into a local variable.
  if(!digitalRead(pSTAGE))
  {
    writeLCD("S");
    mySimpit.activateAction(STAGE_ACTION);
  }
  else
  {
    writeLCD("s");
  }

  debugAnalog(pRY);
  debugAnalog(pRX);
  debugAnalog(pRZ);
  debugAnalog(pTHROTTLE);

  int rx = analogRead(pRX);     // read the input pin
  int ry = analogRead(pRY);     // read the input pin
  int roll = analogRead(pRZ);     // read the input pin
  int throttle = analogRead(pTHROTTLE);     // read the input pin

  if(roll <= 35)
  {
    roll = map(roll,0,35,0,512);
  }
  else if(roll >= 60)
  {
    roll = map(roll,60,500,512,1023);
  }
  else
  {
    roll = 512;
  }

  rotationMessage msg;

  msg.yaw = map(ry, 0, 1023, 32767, -32768);
  msg.pitch = map(rx, 0, 1023, -32768, 32767);
  msg.roll = map(roll, 0, 1023, -32768, 32767);

  int xflag = 1;
  int yflag = 2;
  int zflag = 4;
  
  msg.mask = xflag | yflag | zflag;

  mySimpit.send(ROTATION_MESSAGE, msg);

  int normalizedThrottle = map(throttle, 0, 1023, 0, 32767);

  mySimpit.send(THROTTLE_MESSAGE, (unsigned char*) &normalizedThrottle, 2);


  
  if(!digitalRead(pRCS))
  {
      byte sas = SAS_ACTION;
      mySimpit.send(SAS_MODE_MESSAGE, &sas, 1);
  }
  else
  {
      byte sas = 0;
      mySimpit.send(SAS_MODE_MESSAGE , &sas, 1);
  }
  
  delay(50); 
}
