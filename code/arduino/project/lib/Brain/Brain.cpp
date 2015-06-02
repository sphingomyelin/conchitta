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
  Serial.println(millis());
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
  int time_turning = random(TIME_TURNING_MIN, TIME_TURNING_MAX);
  int speed = 0, steer = 0;
  setSpeed(MAX_SPEED, 0);
  while(1) {
    if(getBottleCount() > MAX_BOTTLES) {
      setState(GO_HOME);
    } else if(obstacleInTheWay()) {
      setState(AVOID_OBSTACLE);
    } else if(getTimeMillis() > TIME_END_GO_HOME) {
      setState(GO_HOME);
    }else {
      if(getPosNearestBottle()) {
        approachNearestBottle();
      } else {
        // Random
        if(millis() - last_forward_command < TIME_GOING_STRAIGHT) {
          // Go straight, but avoid obstacles if possible
          float speedf = 0.0;
          float steerf = 0.0;

          speedf=-pow(1.0116,analogRead(A0))+speed;
          speed = (int)speedf;
          
          if (analogRead(A0)>100) {
            steerf=0.3*analogRead(A0)-30+steer;
          }
          steer = (int)steerf;

          setSpeed(speed, steer);
        } else if(millis() - last_forward_command < (unsigned int)(TIME_GOING_STRAIGHT + time_turning)) {
          int direction = (int)(random(0, 1))*2-1;
          setSpeed(0, direction*MAX_SPEED);
        } else {
          last_forward_command = millis();
          time_turning = random(TIME_TURNING_MIN, TIME_TURNING_MAX);
        }
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
  // TODO: TURN 180 DEGREES properly
  int started_turning = millis();
  while(started_turning - millis() < TIME_TURNING_RELEASE_BOTTLES) {
    setSpeed(0, MAX_SPEED/2);  
  }
  setSpeed(0, 0);
  openTrap();
  delay(5000);
  closeTrap();
  if(getTimeMillis() > TIME_END_GO_HOME) {
    while(1) {
      // TODO: SOMETHING FUNNY
      delay(1000);
    }
  }
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
  // TODO: PID on the x and y of the nearest bottle
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
  if((analogRead(IR_OBST_FL) < IR_OBST_FL_TH) || \
     (analogRead(IR_OBST_FR) < IR_OBST_FR_TH) || \
     (analogRead(IR_OBST_SL) < IR_OBST_SL_TH) || \
     (analogRead(IR_OBST_SR) < IR_OBST_SR_TH)) {
    return true;
  } else {
    return false;    
  }
}


// Communication with RPi
bool Brain::getPosNearestBottle() {
  if (Serial.available() > 0) {
    delay(5);
    char xRPIstr[4], yRPIstr[4];
    char xRPIchar=-1, yRPIchar=-1;
    char charRPI = Serial.read();
    byte index = 0;
    if (charRPI=='x') {
      while ((xRPIchar=Serial.read()) > 47) {
        xRPIchar = xRPIchar - '0';
        xRPIstr[index] = xRPIchar;
        index++;
      }
      if (index==1)
        _xBottle = xRPIstr[0];
      else if (index==2)
        _xBottle = xRPIstr[0]*10+xRPIstr[1];
      else if (index==3)
        _xBottle = xRPIstr[0]*100+xRPIstr[1]*10+xRPIstr[2];
      //index=0;
      //Serial.print("x =");
      //Serial.println(xRPI);
    } else if (charRPI=='y') {
      while ((yRPIchar=Serial.read()) > 47) {
        yRPIchar = yRPIchar -'0';
        yRPIstr[index] = yRPIchar;
        index++;
      }
      if (index==1)
        _yBottle = yRPIstr[0];
      else if (index==2)
        _yBottle = yRPIstr[0]*10+yRPIstr[1];
      else if (index==3)
        _yBottle = yRPIstr[0]*100+yRPIstr[1]*10+yRPIstr[2];
      //index=0;
      //Serial.print("y =");
      //Serial.println(yRPI);
    } else{
      //Serial.println("wrong character");
    }
    //charRPI=-1;
    Serial.flush();

    Bluetooth.send((int)xRPI);
    Bluetooth.send((float)yRPI);
    Bluetooth.send("received pos");
    return true;
  } else {
    return false;
  }
}

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

void Brain::openTrap() {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 160, 1000);
  _trapIsOpen = true;
}

void Brain::closeTrap() {
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 1000);
  _trapIsOpen = false;
  //Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 100);
}
