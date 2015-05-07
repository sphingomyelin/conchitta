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
#include "BluetoothController.h"
#include "constants_mega.h"

#define SCAN_MAX_ANGLE 900

class Brain {

  public:

    Brain();

    int scan_angle;
    int scan_direction;

    // General functions
    void blink() const;
    void setState(STATE State);
    void run();
    void test();
    void RCmode();

  private:
    // Variables
    long last_blink;
    bool last_state_led;

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
    void setSpeed(int speed, int steer) const;
    void stopMotors() const;

    // Dynamixel motors
    void turnBeltForward() const;
    void turnBeltBackward() const;
    void stopBelt() const;

    void openTrap() const;
    void closeTrap() const;


    //	VARIABLES
    STATE _state;

    //	CONSTANTS

    //	OBJECTS
};

#endif
