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
#include "digitalWriteFast.h"
#include "constants_mega.h"

#define SCAN_MAX_ANGLE 900

class Brain {

  public:

    Brain();

    /*int scan_angle;
    int scan_direction;*/

    // General functions
    void blink() const;
    void setState(STATE state);
    void test();
    void run();

  private:
    // Variables
    long last_blink;
    bool last_state_led;
    unsigned int _current_state;
    long _startTime;
    bool _trapIsOpen;

    int _xBottle, _yBottle; 

    // State functions
    void execute_fsm();

    void stateStart();
    void stateGetBottle();
    void stateGoHome();
    void stateReleaseBottles();
    void stateAvoidObstacle();
    void stateAvoidObstacleHome();

    // functions used by state functions
    void approachNearestBottle();
    int getBottleCount();
    int getTimeMillis();
    bool obstacleInTheWay();

    // Communication with RPi
    bool getPosNearestBottle();

    // Communication with WildThumper
    void setSpeed(int speed, int steer) const;
    void stopMotors() const;

    // Dynamixel motors
    void turnBeltForward() const;
    void turnBeltBackward() const;
    void stopBelt() const;

    void openTrap();
    void closeTrap();


    //	VARIABLES
    STATE _state;

    //	CONSTANTS

    //	OBJECTS
};

#endif
