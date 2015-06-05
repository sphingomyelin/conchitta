#include "Brain.h"

/**
 * @file Brain.cpp
 * @author Jan Hermann
 * @version 1.0
 */


Brain::Brain() {
	// initialize variables
  //initDynamixel();
  _trapIsOpen = false;
  _current_state = START;
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

void Brain::execute_fsm() {
  switch(_current_state)
  {
    case START:
      stateStart();
      break;
    case GET_BOTTLES:
      stateGetBottle();
      break;
    case GO_HOME:
      stateGoHome();
      break;
    case RELEASE_BOTTLES:
      stateReleaseBottles();
      break;
    case AVOID_OBSTACLE:
      stateAvoidObstacle();
      break;
    case AVOID_OBSTACLE_HOME:
      stateAvoidObstacleHome();
      break;
  }
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

void Brain::run() {
 // Serial.println(millis());
  Bluetooth.process();
  if(Bluetooth.buttonIsOn(2)) {
    execute_fsm();
  } else {
    if (Bluetooth.buttonIsOn(3)) {
      stopMotors();
    } else {
      setSpeed(Bluetooth.getSpeed(), Bluetooth.getSteer());
    }
    if(Bluetooth.buttonIsOn(5)) {
      if(Bluetooth.buttonIsOn(6)) {
        turnBeltBackward();
      } else {
        turnBeltForward();
      }
    } else {
      stopBelt();
    }
    if(Bluetooth.buttonIsOn(4)) {
      openTrap();
    } else {
      closeTrap();
    }
  }
}


// State functions
void Brain::stateStart() {
  SEND("STATE: START");
  _startTime = millis();
  setState(GET_BOTTLES);
}

void Brain::stateGetBottle() {
  SEND("STATE: GET_BOTTLES");
  int last_forward_command = millis();
  int time_turning = 0;
  setSpeed(MAX_SPEED, 0);
  while(1) {
    if(getBottleCount() > MAX_BOTTLES) {
      setState(GO_HOME);
    } else if(obstacleInTheWay()) {
      setState(AVOID_OBSTACLE);
    } else {
      if(millis() - last_forward_command < TIME_GOING_STRAIGHT) {
        setSpeed(MAX_SPEED, 0);
      } else if(millis() - last_forward_command < (unsigned int)(TIME_GOING_STRAIGHT + time_turning)) {
        int direction = random(0, 1)*2-1;
        setSpeed(direction*MAX_SPEED, -direction*MAX_SPEED);
      } else {
        last_forward_command = millis();
        time_turning = random(-TIME_TURNING, TIME_TURNING);
      }
    }
  }
}

void Brain::stateGoHome() {
  SEND("STATE: GO_HOME");
  // TODO
}

void Brain::stateReleaseBottles() {
  SEND("STATE: RELEASE_BOTTLES");
  openTrap();
  delay(5000);
  setState(GET_BOTTLES);
}

void Brain::stateAvoidObstacle() {
  SEND("STATE: AVOID_OBSTACLE");
  // TODO
}

void Brain::stateAvoidObstacleHome() {
  SEND("STATE: AVOID_OBSTACLE_HOME");
  // TODO
}


// functions used by state functions
void Brain::approachNearestBottle() {
  // TODO
}

int Brain::getBottleCount() {
  // TODO
  return 0;
}

int Brain::getTimeMillis() {
  return millis() - _startTime; 
}

bool Brain::obstacleInTheWay() {
  // TODO
  if((analogRead(IR_FRONT) < IR_FRONT_TH) && (analogRead(IR_TOP) < IR_TOP_TH) && (analogRead(IR_CONTAINER) < IR_CONTAINER_TH)) {
    return true;
  } else {
    return false;    
  }
}


// Communication with RPi
void Brain::getPosNearestBottle() {
  // TODO
}

// Communication with WildThumper
void Brain::setSpeed(int speed, int steer) const {
 // Serial.print("Sending speed to WildThumper: ");
 // Serial.println(speed);
  Wire.beginTransmission(4);
  Wire.write("v");                // set speed
  Wire.write((speed>>8)&0xFF);    // sends the most significant byte  
  Wire.write((speed)&0xFF);       // sends the least significant byte
  Wire.endTransmission();


 // Serial.print("Sending steer to WildThumper: ");
 // Serial.println(steer);
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

void Brain::openTrap() {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 160, 1000);
  _trapIsOpen = true;
}

void Brain::closeTrap() {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 1000);
  _trapIsOpen = false;
  //Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 100);
}
