#include <Arduino.h>
#include "Brain.h"

/**
 * @file Brain.cpp
 * @author Jan Hermann
 * @version 1.0
 */


Brain::Brain() {
	// initialize variables
  //initDynamixel();
}

void Brain::blink() const {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
}

void Brain::setState(STATE state) {
  _state = state;
}

void Brain::test() const {  
  blink();
  turnDynamixelForward();
}


// State functions
void Brain::start() {}

void Brain::getBottle() {}

void Brain::goHome() {}

void Brain::releaseBottles() {}


// functions used by state functions
void Brain::approachNearestBottle() {}

int Brain::getBottleCount() {
  return 0;
}

// Communication with RPi
void Brain::getPosNearestBottle() {}

// Communication with WildThumper
void Brain::setSpeed(int speed, int steer) {
  Wire.beginTransmission(1);
  Wire.write("speed");            // set speed
  Wire.write((speed>>8)&0xFF);    // sends the most significant byte  
  Wire.write((speed)&0xFF);       // sends the least significant byte
  Wire.endTransmission();


  Wire.beginTransmission(1);
  Wire.write("steer");            // set steer (tangential)
  Wire.write((steer>>8)&0xFF);    // sends the most significant byte
  Wire.write((steer)&0xFF);       // sends the least significant byte  
  Wire.endTransmission();
}

// Dynamixel motors

void Brain::turnDynamixelForward() const {
  Dynamixel.turn(DYMX_ID_R, false, 1023);
  Dynamixel.turn(DYMX_ID_L, true, 1023);
}

void Brain::turnDynamixelBackward() const {
  Dynamixel.turn(DYMX_ID_R, true, 1023);
  Dynamixel.turn(DYMX_ID_L, false, 1023);
}

void Brain::stopDynamixel() const {
  Dynamixel.turn(DYMX_ID_R, true, 0);
  Dynamixel.turn(DYMX_ID_L, true, 0);
}