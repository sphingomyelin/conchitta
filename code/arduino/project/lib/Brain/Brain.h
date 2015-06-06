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
// #include "digitalWriteFast.h"
#include "constants_mega.h"

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
    STATE _state; //unsigned int _current_state;
    long _startTime;
    bool _trapIsOpen;

    int _xBottle, _yBottle;
    char _colorLed;
    int _xLed;
    int _speed, _steer;

    // State functions
    void execute_fsm();

    void stateStart();
    void stateGetBottles();
    void stateGetBottlesTransition();
    void stateGoHome();
    void stateReleaseBottles();
    void stateAvoidObstacle();
    void stateAvoidObstacleHome();
    void stateStuckBottle();

    // functions used by state functions
    void approachNearestBottle();
    int getBottleCount();
    long getTimeMillis();
    bool obstacleInTheWay();
    bool isHome();
    bool hasStuckBottle();

    // Communication with RPi
    bool getPosNearestBottle();
    bool getLed();
    void setStateRPi(STATE_RPI stateRpi);
    // int parseNextInt();

    // Communication with WildThumper
    void setSpeed(int speed, int steer) const;
    void stopMotors() const;

    // Dynamixel motors
    void turnBeltForward() const;
    void turnBeltBackward() const;
    void stopBelt() const;

    void openTrap();
    void closeTrap();


    //	VARIABLES OF STATES
    unsigned long _getbottles_last_forward_command;
    unsigned long _getbottles_time_turning;
    int _getbottles_direction;
    int _getbottles_speed;
    int _getbottles_steer;
    unsigned long _stuckbottle_last_free;

    // Test variable for now
    // unsigned long _last_state_change_rpi;
    // int _state_rpi;

    //	CONSTANTS

    //	OBJECTS
};

#endif
