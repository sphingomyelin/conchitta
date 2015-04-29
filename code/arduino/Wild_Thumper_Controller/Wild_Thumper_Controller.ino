#include <Servo.h>
#include "IOpins.h"
#include "Constants.h"
#include <Wire.h>

//#include <Motors.ino>
//#include <Encoder.cpp>


void Set_Speed();

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
bool Stop;
int Angle;
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
int SpeedcmdL=0;
int SpeedcmdR=0;
int CurrentLeftSpeed;
int CurrentRightSpeed;



void setup()
{
  
  Serial.begin(Brate);                                      // enable serial communications if Cmode=1
  Serial.flush();                                           // flush buffer
  
  delay(1000);
  Serial.println("Start ...");
 
  //i2c communication
  // enable pull-ups
  digitalWrite(18,1);
  digitalWrite(19,1);
  Wire.begin(1);                // join i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event

  last_update=millis();
  MotorBeep(3);
}


void loop()
{
  delay(100);
  Serial.println("Loooop....");
  receiveEvent(0);
  
  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------

  Volts=analogRead(Battery);                                  // read the battery voltage
  LeftAmps=analogRead(LmotorC);                               // read left motor current draw
  RightAmps=analogRead(RmotorC);                              // read right motor current draw

  //Serial.print(LeftAmps);
  //Serial.print("    ");
  //Serial.println(RightAmps);

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

  if ((Volts<lowvolt) && (Charged==1))                        // check condition of the battery
  {                                                           // change battery status from charged to flat

    //---------------------------------------------------------- FLAT BATTERY speed controller shuts down until battery is recharged ----
    //---------------------------------------------------------- This is a safety feature to prevent malfunction at low voltages!! ------

    Charged=0;                                                // battery is flat
    highVolts=Volts;                                          // record the voltage
    startVolts=Volts;
    chargeTimer=millis();                                     // record the time

    digitalWrite (Charger,0);                                 // enable current regulator to charge battery
  }


  {//----------------------------------------------------------- GOOD BATTERY speed controller opperates normally ----------------------


  if (millis()-last_update>1000) {
    Speed=0;
    Steer=0;
  }
  
  Set_Speed();
//
//  SpeedcmdL=Speed+Steer/2;
//  SpeedcmdR=Speed-Steer/2;
//  
////  CurrentLeftSpeed=GetSpeedLeft;
////  CurrentRightSpeed=GetSpeedRight;
////  SpeedcmdL=Kp*(SpeedcmdL-CurrentLeftSpeed);
////  SpeedcmdR=Kp*(SpeedcmdR-CurrentRightSpeed);
//  
//  if (SpeedcmdL>0) Leftmode=2;
//  else Leftmode=0;
//  if (SpeedcmdR>0) Rightmode=2;
//  else Rightmode=0;
//
//  LeftPWM = abs(SpeedcmdL)/145*255;
//  LeftPWM = min(LeftPWM,255);                                   // set maximum limit 255
//  RightPWM = abs(SpeedcmdR)/145*255;
//  RightPWM = min(RightPWM,255);                                 // set maximum limit 255
//

    // --------------------------------------------------------- Code to drive dual "H" bridges --------------------------------------

    if (Charged==1)                                           // Only power motors if battery voltage is good
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


void Set_Speed()
{
  SpeedcmdL=Speed+Steer/2;
  SpeedcmdR=Speed-Steer/2;
  
//  CurrentLeftSpeed=GetSpeedLeft;
//  CurrentRightSpeed=GetSpeedRight;
//  SpeedcmdL=Kp*(SpeedcmdL-CurrentLeftSpeed);
//  SpeedcmdR=Kp*(SpeedcmdR-CurrentRightSpeed);
  
  if (SpeedcmdL>0) Leftmode=2;
  else Leftmode=0;
  if (SpeedcmdR>0) Rightmode=2;
  else Rightmode=0;

  LeftPWM = abs(SpeedcmdL)/145*255;
  LeftPWM = min(LeftPWM,255);                                   // set maximum limit 255
  RightPWM = abs(SpeedcmdR)/145*255;
  RightPWM = min(RightPWM,255);                                 // set maximum limit 255
}
  

//
//void I2Cmode()                    // I2C mode via A4(SDA) and A5(SCL)
//{//----------------------------------------------------------- Your code goes here ------------------------------------------------------------
//
//}

void receiveEvent(int howMany)
{
  char v,w;
  
    Serial.println("Receiving over I2C...");
  
    while(1 < Wire.available()) // loop through all but the last
    {
      char input = Wire.read(); // receive byte as a character
      switch(input) {
      case 'v': //set speed
        Speed = Wire.parseInt();
        Serial.print(v);             // print the character
        Serial.print(" ");
        Serial.println(Speed);         // print the speed
        Stop = false;
        last_update=millis();
        break;
        
      case 'w': //set angle to turn
        Steer = Wire.parseInt();
        Serial.print(w);             // print the character
        Serial.print(" ");
        Serial.println(Steer);         // print the angle
        Stop = false;
        last_update=millis();
        break;
        
      case 's': //set angle to turn
        Stop = true;
        break;
        
     }
   }
   
   Serial.println("Finished receiving.");
   
   // empty buffer
   Wire.flush();
}
  
  
  // function that executes whenever data is requested by master
  // this function is registered as an event, see setup()
//void requestEvent()
//  {
//    Wire.write("hello "); // respond with message of 6 bytes
//                         // as expected by master
//  }

  

