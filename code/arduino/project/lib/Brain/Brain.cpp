#include <Arduino.h>
#include "Brain.h"

/**
 * @file Brain.cpp
 * @author Jan Hermann
 * @version 1.0
 */


Brain::Brain() {
	// initialize variables
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


// State functions

void Brain::getBottle() {}

void Brain::goHome() {}

void Brain::releaseBottles() {}


// functions used by state functions
void Brain::approachNearestBottle() {}

int Brain::getBottleCount() {}

// Communication with RPi
void Brain::getPosNearestBottle() {}

// Communication with WildThumper
void Brain::setSpeed(int speed, int steer) {
  Wire.beginTransmission(1);
  Wire.write("v");            // set speed
  Wire.write((v>>8)&0xFF);    // sends MSByte  
  Wire.write((v)&0xFF);       // sends LSByte
  Wire.endTransmission();


  Wire.beginTransmission(1);
  Wire.write("t");            // set steer (tangential)
  Wire.write((t>>8)&0xFF);    // sends MSByte
  Wire.write((t)&0xFF);       // sends LSByte  
  Wire.endTransmission();
}