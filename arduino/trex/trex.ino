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
int speed_R = 0;
int speed_L = 0;
int Speed = 0;  //entre 0 et 136 (full speed a 14V) 
int Steer = 0;  //entre 0 et 136
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
int MeasuredSpeed_R;
int MeasuredSpeed_L;

float integrator_R=0, integrator_L=0;

float const Kpi = Kp+Ki;                  // sth beween 0.001 and 1 (fix)(kp 0.12 and ki 0.001 pour tp köchli)
float const inv_Kpi = 1.0/Kpi;            // sth beween 1 and 1000 

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

  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------

  Volts=analogRead(Battery);                                  // read the battery voltage
  LeftAmps=analogRead(LmotorC);                               // read left motor current draw
  RightAmps=analogRead(RmotorC);                              // read right motor current draw
  
  Serial.print("Volts: ");
  Serial.println(Volts);

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
    if (millis()-last_update>1000) {                        // if communication brakes, stops motor after 3 seconds
      Speed=0;
      Steer=0;
      Stop=true;
    }
    
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
  //Serial.println(millis());
  int speed_difference_R, speed_difference_L;
  float speed_p_R, speed_p_L;
  float speed_p_R_arw, speed_p_L_arw;
  float elim_R, elim_L;

  //calculate and limits speed value for each side
  if (abs(Speed)+abs(Steer)<140) {
    speed_L=Speed+Steer;
    speed_R=Speed-Steer;
        /*
        Serial.print("Speed");           // print the character
        Serial.print(" ");
        Serial.println(Speed);           // print the speed
        Serial.print("Steer");           // print the character
        Serial.print(" ");
        Serial.println(Steer);           // print the angle
        */
  }
  else {
    speed_L=(int)((Speed+Steer)/((abs(Speed)+abs(Steer))/140.0));
    speed_R=(int)((Speed-Steer)/((abs(Speed)+abs(Steer))/140.0));
  }
        /*Serial.print("speed_R");       // print the character
        Serial.print(" ");
        Serial.println(speed_R);         // print the angle
        */
 // target value in encoder speed
  MeasuredSpeed_L=GetSpeedLeft();        // Speed updating at 100 Hz. v_rpm = n_enc * 100Hz/64 * 60sec/70gearratio; 
  MeasuredSpeed_R=GetSpeedRight();       // v_rpm = n_enc * 1.339285714

  //PI controller-Right side
  speed_difference_R = speed_R - MeasuredSpeed_R;
  speed_p_R = integrator_R + (Kpi * speed_difference_R);
  // arw
  if(speed_p_R > 255) speed_p_R_arw = 255;
  else if(speed_p_R < -255) speed_p_R_arw = -255;
  else speed_p_R_arw = speed_p_R;
  elim_R = speed_difference_R - ((speed_p_R - speed_p_R_arw)*inv_Kpi);
  integrator_R = integrator_R + (Ki*elim_R);

        /*Serial.print("speed_p_R");             // print the character
        Serial.print(" ");
        Serial.println(speed_p_R);         // print the angle
        Serial.print("speed_pR_arw");             // print the character
        Serial.print(" ");
        Serial.println(speed_p_R_arw);         // print the speed
        Serial.print("measured");             // print the character
        Serial.print(" ");
        Serial.println(MeasuredSpeed_L);         // print the angle
        Serial.print(" ");             // print the character
        Serial.print("integrator_r");             // print the character
        Serial.print(" ");
        Serial.println(integrator_R);         // print the angle
        Serial.println(" ");*/
        

  //PI controller-Left side
  speed_difference_L = speed_L - MeasuredSpeed_L;
  speed_p_L = integrator_L + (Kpi * speed_difference_L);
  // arw
  if(speed_p_L > 255) speed_p_L_arw = 255;
  else if(speed_p_L < -255) speed_p_L_arw = -255;
  else speed_p_L_arw = speed_p_L;
  elim_L = speed_difference_L - ((speed_p_L - speed_p_L_arw)*inv_Kpi);
  integrator_L = integrator_L + (Ki*elim_L);

  
  if (speed_p_L>0) Leftmode=2;
  else if (speed_p_L<0) Leftmode=0;
  else Leftmode=1;
  if (speed_p_R>0) Rightmode=2;
  else if (speed_p_R<0) Rightmode=0;
  else Rightmode=1;

  LeftPWM = abs(speed_p_L_arw);                            
  RightPWM = abs(speed_p_R_arw);     

  if (MeasuredSpeed_R ==0 && RightPWM<50) RightPWM=0;  
  if (MeasuredSpeed_L ==0 && LeftPWM<50) LeftPWM=0;

  //Serial.println(millis());
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
        Stop = false;
        last_update=millis();
        break;

      case 'w': //set angle to turn
        Steer=(int)(((unsigned int)Wire.read())<<8 | (unsigned int)Wire.read());
        //Steer = Wire.read()*256 + Wire.read();
        Stop = false;
        last_update=millis();
        /*Serial.println("Starting motors...");
        Serial.print("v");             // print the character
        Serial.print(" ");
        Serial.println(Speed);         // print the speed
        Serial.print("w");             // print the character
        Serial.print(" ");
        Serial.println(Steer);         // print the angle*/
        break;

      case 's': //set angle to turn
        Stop = true;
        Speed = 0;
        Steer = 0;
        last_update=millis();
        //Serial.println("Stopping motors.");
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
  int pwmon = 250;

  analogWrite (LmotorA,0);                                // turn off motors
  analogWrite (LmotorB,0);                                // turn off motors
  analogWrite (RmotorA,0);                                // turn off motors
  analogWrite (RmotorB,0);                                // turn off motors


  for(int b=0;b<beeps;b++)                              // loop to generate multiple beeps
  {
    for(int duration=0;duration<400;duration++)         // generate 2kHz tone for 200mS
    {
      analogWrite(RmotorA,0);                          // drive right motor forward
      analogWrite(RmotorB,pwmon);
      analogWrite(LmotorA,0);                          // drive left  motor forward
      analogWrite(LmotorB,pwmon);
      delayMicroseconds(50);                            // limit full power to 50uS
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone

      analogWrite(RmotorA,pwmon);                          // drive right motor forward
      analogWrite(RmotorB,0);
      analogWrite(LmotorA,pwmon);                          // drive left  motor forward
      analogWrite(LmotorB,0);
      delayMicroseconds(50);                              // limit full power to 50uS
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone

    }
    delay(200);                                         // pause for 200mS (1/5th of a second) between beeps
  }
}

