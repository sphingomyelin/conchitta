#ifndef _CONCHITTA_BRAIN_
#define _CONCHITTA_BRAIN_

/**
 * @file Brain.h
 * @author Jan Hermann
 * @version 1.0
 */

#include <Arduino.h>
#include <Wire.h>
#include "DynamixelSerial.h"
#include "constants_mega.h"

class Brain {

  public:

    Brain();

    // General functions
    void blink() const;
    void setState(STATE State);
    void test() const;

  private:
    // State functions
    void start();
    void getBottle();
    void goHome();
    void releaseBottles();

    // functions used by state functions
    void approachNearestBottle();
    int getBottleCount();

    // Communication with RPi
    void getPosNearestBottle();

    // Communication with WildThumper
    void setSpeed(int speed, int steer);

    // Dynamixel motors
    void turnDynamixelForward() const;
    void turnDynamixelBackward() const;
    void stopDynamixel() const;

    //	VARIABLES
    STATE _state;

    //	CONSTANTS

    //	OBJECTS
};

#endif
