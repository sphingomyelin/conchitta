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
  if(millis() - last_blink > 200) {
    if(last_state_led == true) digitalWrite(LED, LOW);
    else digitalWrite(LED, HIGH);
  }
}

void Brain::setState(STATE state) {
  _state = state;
}

void Brain::run() {
  
  
  
}

void Brain::test() {  
  blink();
  
  // ------- Testing Belt -------
  // turnBeltForward();

  // ------- Testing Trap -------
  //Serial.println("Move! Go");
  openTrap();
  delay(500);
  closeTrap();
  delay(500);
  //delay(1000);
  //delay (20); 

}

void Brain::RCmode() {
  Serial.println(millis());
  Bluetooth.process();
  if(Bluetooth.buttonIsOn(3)) {
    stopMotors();
  } else {
    setSpeed(Bluetooth.getSpeed(), Bluetooth.getSteer());
  }
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
void Brain::setSpeed(int speed, int steer) const {
  Serial.print("Sending speed to WildThumper: ");
  Serial.println(speed);
  Wire.beginTransmission(4);
  Wire.write("v");                // set speed
  Wire.write((speed>>8)&0xFF);    // sends the most significant byte  
  Wire.write((speed)&0xFF);       // sends the least significant byte
  Wire.endTransmission();


  Serial.print("Sending steer to WildThumper: ");
  Serial.println(steer);
  Wire.beginTransmission(4);
  Wire.write("w");                // set steer (tangential)
  Wire.write((steer>>8)&0xFF);    // sends the most significant byte
  Wire.write((steer)&0xFF);       // sends the least significant byte
  Wire.endTransmission();
}

void Brain::stopMotors() const {
  Serial.println("Stopping Motors.");
  Wire.beginTransmission(4);
  Wire.write("s");
  Wire.endTransmission();
}

// Dynamixel motors

void Brain::turnBeltForward() const {
  Dynamixel.turn(DYMX_ID_R, false, 1023);
  Dynamixel.turn(DYMX_ID_L, true, 1023);
}

void Brain::turnBeltBackward() const {
  Dynamixel.turn(DYMX_ID_R, true, 1023);
  Dynamixel.turn(DYMX_ID_L, false, 1023);
}

void Brain::stopBelt() const {
  Dynamixel.turn(DYMX_ID_R, true, 0);
  Dynamixel.turn(DYMX_ID_L, true, 0);
}

void Brain::openTrap() const {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 160, 1000);
}

void Brain::closeTrap() const {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 490, 1000);
  //Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 100);
}