#include "IOpins.h"
#include "Constants.h"
#include <Wire.h>

#include "Encoder.h"


void CalculateSpeed();

//-------------------------------------------------------------- define global variables --------------------------------------------

unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int Leftspeed=0;
int Rightspeed=0;
int Speed;
int Steer;
bool Stop;                                                    // true=Motors off, false=Motors on
byte Charged=1;                                               // 0=Flat battery  1=Charged battery
int Leftmode=1;                                               // 0=reverse, 1=brake, 2=forward
int Rightmode=1;                                              // 0=reverse, 1=brake, 2=forward
byte Leftmodechange=0;                                        // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange=0;                                       // Right input must be 1500 before brake or reverse can occur
int LeftPWM;                                                  // PWM value for left  motor speed / brake
int RightPWM;                                                 // PWM value for right motor speed / brake
int data;
int servo[7];

long last_update;
//int CurrentLeftSpeed;
//int CurrentRightSpeed;

void setup()
{
  // Serial Communication with the computer
  Serial.begin(Brate);                                      // enable serial communications if Cmode=1
  Serial.flush();                                           // flush buffer

  delay(500);
  Serial.println("Start ...");

  // I2C Communication
  // enable pull-ups
  digitalWrite(18,1);
  digitalWrite(19,1);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

    // Charging
  pinMode(Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite(Charger,1);                                   // disable current regulator to charge battery

  // Initialization
  last_update=millis();
  QuadratureEncoderInit();
  MotorBeep(3);
}


void loop()
{

  delay(200);
  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------
  Serial.print("Pos R: ");
  Serial.print(GetPosRight());
  Serial.print(", Pos L: ");
  Serial.println(GetPosLeft());
  

  Volts=analogRead(Battery);                                  // read the battery voltage
  LeftAmps=analogRead(LmotorC);                               // read left motor current draw
  RightAmps=analogRead(RmotorC);                              // read right motor current draw

  if (LeftAmps>Leftmaxamps)                                   // is motor current draw exceeding safe limit
  {
    analogWrite (LmotorA,0);                                  // turn off motors
    analogWrite (LmotorB,0);                                  // turn off motors
    leftoverload=millis();                                    // record time of overload
  }

  if (RightAmps>Rightmaxamps)                                 // is motor current draw exceeding safe limit
  {
    analogWrite (RmotorA,0);                                  // turn off motors
    analogWrite (RmotorB,0);                                  // turn off motors
    rightoverload=millis();                                   // record time of overload
  }

  //---------------------------------------------------------- FLAT BATTERY speed controller shuts down until battery is recharged ----
  //---------------------------------------------------------- This is a safety feature to prevent malfunction at low voltages!! ------
  if ((Volts<lowvolt) && (Charged==1))                        // check condition of the battery
  {                                                           // change battery status from charged to flat
    Charged=0;                                                // battery is flat
    highVolts=Volts;                                          // record the voltage
    startVolts=Volts;
    chargeTimer=millis();                                     // record the time

    digitalWrite (Charger,0);                                 // enable current regulator to charge battery
  }

  //------------------------------------------------------------ CHARGE BATTERY -------------------------------------------------------
  if ((Charged==0) && (Volts-startVolts>67))                  // if battery is flat and charger has been connected (voltage has increased by at least 1V)
  {
    if (Volts>highVolts)                                      // has battery voltage increased?
    {
      highVolts=Volts;                                        // record the highest voltage. Used to detect peak charging.
      chargeTimer=millis();                                   // when voltage increases record the time
    }

    if (Volts>batvolt)                                        // battery voltage must be higher than this before peak charging can occur.
    {
      if ((highVolts-Volts)>5 || (millis()-chargeTimer)>chargetimeout) // has voltage begun to drop or levelled out?
      {
        Charged=1;                                            // battery voltage has peaked
        digitalWrite (Charger,1);                             // turn off current regulator
      }
    } 
  }
  //----------------------------------------------------------- GOOD BATTERY speed controller opperates normally ----------------------
  else 
  {
    /*if (millis()-last_update>1000) {
     Speed=0;
     Steer=0;
     }*/

    CalculateSpeed();

    // --------------------------------------------------------- Code to drive dual "H" bridges --------------------------------------
    if (Charged==1 && !Stop)                                           // Only power motors if battery voltage is good
    {
      if ((millis()-leftoverload)>overloadtime)             
      {
        switch (Leftmode)                                     // if left motor has not overloaded recently
        {
        case 2:                                               // left motor forward
          analogWrite(LmotorA,0);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 1:                                               // left motor brake
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,LeftPWM);
          break;

        case 0:                                               // left motor reverse
          analogWrite(LmotorA,LeftPWM);
          analogWrite(LmotorB,0);
          break;
        }
      }
      if ((millis()-rightoverload)>overloadtime)
      {
        switch (Rightmode)                                    // if right motor has not overloaded recently
        {
        case 2:                                               // right motor forward
          analogWrite(RmotorA,0);
          analogWrite(RmotorB,RightPWM);
          break;

        case 1:                                               // right motor brake
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,RightPWM);
          break;

        case 0:                                               // right motor reverse
          analogWrite(RmotorA,RightPWM);
          analogWrite(RmotorB,0);
          break;
        }
      } 
    }
    else                                                      // Battery is flat
    {
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
    }
  }
}


void CalculateSpeed()
{
  Leftspeed=Speed+Steer/2;
  Rightspeed=Speed-Steer/2;

  //  CurrentLeftSpeed=GetSpeedLeft;
  //  CurrentRightSpeed=GetSpeedRight;
  //  Leftspeed=Kp*(Leftspeed-CurrentLeftSpeed);
  //  Rightspeed=Kp*(Rightspeed-CurrentRightSpeed);

  if (Leftspeed>0) Leftmode=2;
  else if (Leftspeed<0) Leftmode=0;
  else Leftmode=1;
  if (Rightspeed>0) Rightmode=2;
  else if (Rightspeed<0) Rightmode=0;
  else Rightmode=1;

  LeftPWM = abs(Leftspeed);
  LeftPWM = min(LeftPWM,255);                                   // set maximum limit 255
  RightPWM = abs(Rightspeed);
  RightPWM = min(RightPWM,255);                                 // set maximum limit 255
}



void receiveEvent(int bytesReceived)
{
  bytesReceived = Wire.available();
  if(bytesReceived == 3 || bytesReceived == 1)
  {
    char input = Wire.read(); // receive byte as a character
    switch(input) {
    case 'v': //set speed
      Speed=(int)(((unsigned int)Wire.read())<<8 | (unsigned int)Wire.read());
      //Speed = Wire.read()*256 + Wire.read();
      last_update=millis();
      break;

    case 'w': //set angle to turn
      Steer=(int)(((unsigned int)Wire.read())<<8 | (unsigned int)Wire.read());
      //Steer = Wire.read()*256 + Wire.read();
      Stop = false;
      last_update=millis();
      Serial.println("Starting motors...");
      Serial.print("v");             // print the character
      Serial.print(" ");
      Serial.println(Speed);         // print the speed
      Serial.print("w");             // print the character
      Serial.print(" ");
      Serial.println(Steer);         // print the angle

      break;

    case 's': //set angle to turn
      Stop = true;
      Speed = 0;
      Steer = 0;
      last_update=millis();
      Serial.println("Stopping motors.");
      break;

    default: //something went wrong
      Serial.println("ERROR: Received faulty speed settings over I2C!");
      Stop = true;
      Speed = 0;
      Steer = 0;
      Serial.println("Stopping motors.");
    }
  } 
  else {
    while(Wire.available()) {
      char c = Wire.read();
      Serial.print(c); 
    }
    Serial.println("");
  }

  // empty buffer
  Wire.flush();
}

void requestEvent() {
  Wire.write("hello ");
}


void MotorBeep(int beeps)                              
{
  digitalWrite(lmbrkpin,0);                             // ensure breaks are off
  digitalWrite(rmbrkpin,0);

  for(int b=0;b<beeps;b++)                              // loop to generate multiple beeps
  {
    for(int duration=0;duration<400;duration++)         // generate 2kHz tone for 200mS
    {
      digitalWrite(lmdirpin,1);                         // drive left  motor forward
      digitalWrite(rmdirpin,1);                         // drive right motor forward
      digitalWrite(lmpwmpin,1);                         // left  motor at 100%
      digitalWrite(rmpwmpin,1);                         // right motor at 100%
      delayMicroseconds(50);                            // limit full power to 50uS
      digitalWrite(lmpwmpin,0);                         // shutdown left  motor
      digitalWrite(rmpwmpin,0);                         // shutdown right motor
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone

      digitalWrite(lmdirpin,0);                         // drive left  motor backward
      digitalWrite(rmdirpin,0);                         // drive right motor backward
      digitalWrite(lmpwmpin,1);                         // left  motor at 100%
      digitalWrite(rmpwmpin,1);                         // right motor at 100%
      delayMicroseconds(50);                            // limit full power to 50uS
      digitalWrite(lmpwmpin,0);                         // shutdown left  motor
      digitalWrite(rmpwmpin,0);                         // shutdown right motor
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone
    }
    delay(200);                                         // pause for 200mS (1/5th of a second) between beeps
  }
}

