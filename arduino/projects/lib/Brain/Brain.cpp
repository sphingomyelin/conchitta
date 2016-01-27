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
  _state = START;

  _speed = 0;
  _steer = 0;

  _stuckbottle_last_free = millis();
  _bottle_check_index = 0;
  _bottle_here = true;
  _bottle_count = 0;


  _findbottle_expiration = 0;
  _findled_expiration = 0;
  _flag_go_again = false;
  // // Testing
  // _last_state_change_rpi = 0;
  // _state_rpi = 1;

  setStateRPi(RPI_GET_BOTTLES);
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
  switch(_state)
  {
    case START:
      stateStart();
      break;
    case GET_BOTTLES:
      stateGetBottlesTransition();
    case GET_BOTTLES_STATE:
      stateGetBottles();
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
    case STUCK_BOTTLE:
      stateStuckBottle();
      break;
  }
}

void Brain::run() {
  //Serial.println(millis());
  Bluetooth.process();
  // getPosFromLinCam(); // --------------- TESTING THIS -------------------
  if(Bluetooth.buttonIsOn(2)) {
    execute_fsm();
  } else {

  // --------------------------------------------- TESTING ----------------------------------------------------- TODO

    // Bluetooth.send((float)((int)(_xPosLinCam*10.0) + ((int)(_yPosLinCam*10.0))/100.0));
    // Bluetooth.send((float)analogRead(IR_FRONT));
    Bluetooth.send("RC Mode");
    countBottles();
    Bluetooth.send((int)getBottleCount());
    Bluetooth.send((float)analogRead(IR_FRONT));
    if (Bluetooth.buttonIsOn(3)) {
      stopMotors();
    } else {
      // ----------------------------- TESTING --------------------------------------------
      setSpeed((int)Bluetooth.getSpeed()*1.4, (int)Bluetooth.getSteer()*1.4);
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
  SEND("START");
  setStateRPi(RPI_GET_BOTTLES);
  // Bluetooth.send("Start");
  _startTime = millis();
  setState(GET_BOTTLES);
}

void Brain::stateGetBottlesTransition() {
  SEND("GET_BOTTLES");
  turnBeltForward();
  setStateRPi(RPI_GET_BOTTLES);
  _getbottles_last_forward_command = millis();
  _getbottles_time_turning = random(TIME_TURNING_MIN, TIME_TURNING_MAX);
  _getbottles_speed = 0; 
  _getbottles_steer = 0;
  _stuckbottle_last_free = millis();
  setSpeed(MAX_SPEED, 0);
  setState(GET_BOTTLES_STATE);
}

void Brain::stateGetBottles() {
  // SEND("GET_BOTTLES_STATE");
  if(_rpi_count > 10) {
    setStateRPi(RPI_GET_BOTTLES);
    _rpi_count = 0;
  }
  _rpi_count++;
  checkForNoStuckBottle();
  countBottles();
  //Bluetooth.send((int)(_getbottles_time_turning+TIME_GOING_STRAIGHT));
  if(getTimeMillis() > TIME_END) {
    setState(STOP);
    return;
  } else if(getTimeMillis() > TIME_END_GO_HOME && !_flag_go_again) {
    setState(GO_HOME);
    return;
  } else if(getBottleCount() > MAX_BOTTLES) {
    setState(GO_HOME);
    return;
  } /*else if(obstacleInTheWay()) {
    setState(AVOID_OBSTACLE);
  }*/ else if(hasStuckBottle()) {
    setState(STUCK_BOTTLE);
    return;
  } else {
    if(getPosNearestBottle()) {
      SEND("BOTTLE O.O");
      _getbottles_last_forward_command = millis();
      approachNearestBottle();
    } else {
      randomWalkAvoidingObstacles();
    }
  }
}

void Brain::stateGoHome() {
  SEND("GO_HOME");
  // turnBeltForward();
  if(_rpi_count == 20) {
    _rpi_count = 0;
    setStateRPi(RPI_GO_HOME);
  }
  _rpi_count++;

  if(isHome()) {
    setState(RELEASE_BOTTLES);
    return;
  }
  if(getLed()) {
    if(_colorLed == 'j') {
      setSpeedAvoidingObstacles(MAX_SPEED, ((_xLed - 160.0) / 160.0) * SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
      _going_home_last_forward_command = millis();
      return;
    }
  } 

  // setSpeed(_speed*(IR_OBST_SMOOTHING_RATIO), _steer*(IR_OBST_SMOOTHING_RATIO) + (1.0-IR_OBST_SMOOTHING_RATIO)*SLOW_STEER);
  
  unsigned long time_since_last_forward = millis() - _going_home_last_forward_command;
  if(time_since_last_forward < TIME_GOING_STRAIGHT_GOING_HOME) {
    // Go straight, but avoid obstacles if possible
    setSpeedAvoidingObstacles(MAX_SPEED, 0, IR_OBST_SMOOTHING_RATIO);
  } else if(time_since_last_forward < (TIME_GOING_STRAIGHT_GOING_HOME + _going_home_time_turning)) {
    setSpeedAvoidingObstacles(0, _going_home_direction*SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
  } else {
    _going_home_last_forward_command = millis();
    _going_home_time_turning = random(TIME_TURNING_MIN_GOING_HOME, TIME_TURNING_MAX_GOING_HOME);
    _going_home_direction = (int)(random(0, 2))*2-1;
  }
}

void Brain::stateReleaseBottles() {
  SEND("RELEASE_BOTTLES");
  // TODO: TURN 180 DEGREES properly
  // int started_turning = millis();
  // while(started_turning - millis() < TIME_TURNING_RELEASE_BOTTLES) {
  //   setSpeedAvoidingObstacles(0, MAX_STEER, IR_OBST_SMOOTHING_RATIO);
  // }

  while(1) {
    if(getEndLinCam()) {
      if(abs(_b_index) < 30) {
        setSpeed(0, 0);
        break;
      }
    }
    setSpeed(0, SLOW_STEER);
  }
  setSpeed(SLOW_SPEED, 0);
  delay(2000);
  setSpeed(0, 0);
  openTrap();
  setSpeed(MAX_SPEED, 0);
  delay(500);
  setSpeed(-MAX_SPEED, 0);
  delay(500);
  setSpeed(MAX_SPEED, 0);
  setSpeed(0,0);
  turnBeltBackward();  //jai rajouté ca sur idée de charlotte ;-)
  delay(10000);
  stopBelt();
  setSpeed(SLOW_SPEED, 0);
  closeTrap();
  if(getTimeMillis() > TIME_END_GO_HOME) {
    _flag_go_again = true;
  }
  _bottle_count = 0;
  setState(GET_BOTTLES);
}

void Brain::stateAvoidObstacle() {
  SEND("AVOID_OBSTACLE");
  Bluetooth.send("Avoid obstacle");
  // TODO
  int dir = random(0,2)*2-1;
  setSpeedAvoidingObstacles(0, dir*SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
  delay(TIME_TURNING_RELEASE_BOTTLES);
  setSpeedAvoidingObstacles(SLOW_SPEED, 0, IR_OBST_SMOOTHING_RATIO);
  setState(GET_BOTTLES);
}

void Brain::stateAvoidObstacleHome() {
  SEND("AVOID_OBSTACLE_HOME");
 
  // TODO
  int dir = random(0,2)*2-1;
  setSpeedAvoidingObstacles(0, dir*SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
  delay(TIME_TURNING_RELEASE_BOTTLES/2);
  setSpeedAvoidingObstacles(SLOW_SPEED, 0, IR_OBST_SMOOTHING_RATIO);
  delay(TIME_TURNING_RELEASE_BOTTLES);
  setSpeedAvoidingObstacles(0, -dir*SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
  delay(TIME_TURNING_RELEASE_BOTTLES/2);
  setState(GO_HOME);
}

void Brain::stateStuckBottle() {
  SEND("STUCK_BOTTLE");
  setSpeed(-100, 0);
  delay(200);
  setSpeed(0, 0);
  turnBeltBackward();
  delay(2000);
  int dir = 0;
  if(_bottle_stuck[_median_index_stuck_bottle] > STUCK_BOTTLE_IN_THE_MIDDLE_IR_VALUE) {
    dir = 1;
  } else {
    dir = -1;
  }
  unsigned long turning_time = millis();
  while(millis() - turning_time < 2000) {
    setSpeedAvoidingObstacles(SLOW_SPEED, dir*SLOW_STEER, IR_OBST_SMOOTHING_RATIO);
  }
  setSpeed(0, 0);
  turnBeltForward();
  _stuckbottle_last_free = millis();
  setState(GET_BOTTLES);
}

void Brain::stateStop() {
  SEND("STOP");
  setSpeed(0, 0);
  stopBelt();
  stopMotors();
  setStateRPi(RPI_SHUTDOWN);
}


// functions used by state functions
void Brain::approachNearestBottle() {
  // TODO: PID on the x and y of the nearest bottle
  int speed, steer;
  speed = (((240.0 - _yBottle) / 240.0) * 0.5 + 0.5) * MAX_SPEED; 
  steer = ((_xBottle - 160.0) / 160.0) * MAX_STEER;

  setSpeedAvoidingObstacles(speed, steer, BOTTLE_SMOOTHING_RATIO);
  //setSpeed(_speed, _steer);
}

void Brain::randomWalkAvoidingObstacles() {
  // Random
  SEND("RANDOM");
  unsigned long time_since_last_forward = millis() - _getbottles_last_forward_command;
  if(time_since_last_forward < TIME_GOING_STRAIGHT) {
    // Go straight, but avoid obstacles if possible, but if not... tant pis
    setSpeedAvoidingObstacles(MAX_SPEED, 0, IR_OBST_SMOOTHING_RATIO);
  } else if(time_since_last_forward < (TIME_GOING_STRAIGHT + _getbottles_time_turning)) {
    setSpeedAvoidingObstacles(0, _getbottles_direction*MAX_STEER, IR_OBST_SMOOTHING_RATIO);
  } else {
    _getbottles_last_forward_command = millis();
    _getbottles_time_turning = random(TIME_TURNING_MIN, TIME_TURNING_MAX);
    _getbottles_direction = (int)(random(0, 2))*2-1;
  }
}

int Brain::getBottleCount() {
  return _bottle_count;
}

unsigned long Brain::getTimeMillis() {
  return millis() - _startTime; 
}

bool Brain::obstacleInTheWay() {
  // TODO
  /*if((analogRead(IR_OBST_FL) > IR_OBST_FL_TH) || \
     (analogRead(IR_OBST_FR) > IR_OBST_FR_TH) || \
     (analogRead(IR_OBST_SL) > IR_OBST_SL_TH) || \
     (analogRead(IR_OBST_SR) > IR_OBST_SR_TH)) {
    return true;
  } else {*/
    return false;    
  // }
}

bool Brain::isHome() {
  // TODO
  // if(getTimeMillis() > TIME_END - 20000) {
  //   return true;
  // } else {
  //   return false;
  // }

  // if(_found_intensity) {
  //   _led_home[_led_home_index] = _colorIntensity;
  //   _led_home_index = (_led_home_index + 1) % LED_INTENSITY_MEDIAN_SIZE;
  //   // Calculate the pseudo-median
  //   int avg = 0;
  //   for(int i = 0; i < LED_INTENSITY_MEDIAN_SIZE; i++) {
  //     avg += _led_home[i];
  //   }
  //   avg /= LED_INTENSITY_MEDIAN_SIZE;
  //   int median_index = 0, diff = 20000;
  //   for(int i = 0; i < LED_INTENSITY_MEDIAN_SIZE; i++) {
  //     if(abs(avg - _led_home[i]) < diff) {
  //       diff = abs(avg - _led_home[i]);
  //       median_index = i;
  //     }
  //   }

  //   if(_led_home[median_index] > HOME_INTENSITY_THRESHOLD) {
  //     return true;
  //   }
  // }

  if(getEndLinCam()) {
    if(_f_nb >= LED_PEAK_ON_MAX_DURING_GO_HOME) {
      return true;
    }
  }

  return false;
}

bool Brain::hasStuckBottle() {
  if(millis() - _stuckbottle_last_free > STUCK_BOTTLE_THRESHOLD_TIME) {
    return true;
  } else {
    return false;
  }  
}

void Brain::checkForNoStuckBottle() {
  // if(analogRead(IR_FRONT) > IR_FRONT_TH) {
  //   _stuckbottle_last_free = millis();
  // }

  _bottle_stuck[_bottle_stuck_check_index] = analogRead(IR_FRONT);
  _bottle_stuck_check_index = (_bottle_stuck_check_index + 1) % BOTTLE_STUCK_MEDIAN_SIZE;
  // Calculate the pseudo-median
  int avg = 0;
  for(int i = 0; i < BOTTLE_STUCK_MEDIAN_SIZE; i++) {
    avg += _bottle_stuck[i];
  }
  avg /= BOTTLE_STUCK_MEDIAN_SIZE;
  int diff = 20000;
  _median_index_stuck_bottle = 0;
  for(int i = 0; i < BOTTLE_STUCK_MEDIAN_SIZE; i++) {
    if(abs(avg - _bottle_stuck[i]) < diff) {
      diff = abs(avg - _bottle_stuck[i]);
      _median_index_stuck_bottle = i;
    }
  }

  if(_bottle_stuck[_median_index_stuck_bottle] < IR_FRONT_TH) {
    _stuckbottle_last_free = millis();
  }
}

void Brain::countBottles() {
  _bottle_at_ramp[_bottle_check_index] = analogRead(IR_TOP);
  _bottle_check_index = (_bottle_check_index + 1) % BOTTLE_COUNT_MEDIAN_SIZE;
  // Calculate the pseudo-median
  int avg = 0;
  for(int i = 0; i < BOTTLE_COUNT_MEDIAN_SIZE; i++) {
    avg += _bottle_at_ramp[i];
  }
  avg /= BOTTLE_COUNT_MEDIAN_SIZE;
  int median_index = 0, diff = 20000;
  for(int i = 0; i < BOTTLE_COUNT_MEDIAN_SIZE; i++) {
    if(abs(avg - _bottle_at_ramp[i]) < diff) {
      diff = abs(avg - _bottle_at_ramp[i]);
      median_index = i;
    }
  }

  if((_bottle_at_ramp[median_index] > IR_TOP_TH)) {
    if(millis() - _bottle_detect_time > BOTTLE_COUNT_DEAD_WINDOW) {
      if(!_bottle_here) {
        _bottle_count++;
        _bottle_detect_time = millis();
        _bottle_here = true;
      }
    }
  } else {
    _bottle_here = false;
  }
}

// Communication with RPi
bool Brain::getPosNearestBottle() {
  bool found_x = false, found_y = false;
  int xBottle = 0, yBottle = 0;
  // delay(5); // ----------------------------TESTING REMOVED!!!!!! ---------------------------------------------------
  // while(Serial.available() > 0) {
  while((!found_x || !found_y) && Serial.available() > 0) {
    char charRPI = Serial.read();
    if (charRPI=='x') {
      found_x = true;
      xBottle = Serial.parseInt();
    } else if(charRPI=='y') {
      found_y = true;
      yBottle = Serial.parseInt();
    }
  }
  // Discard everything in buffer
  while(Serial.read() != -1);

  if(found_x && found_y) {
    _xBottle = xBottle;
    _yBottle = yBottle;
    // Bluetooth.send((int)_xBottle);
    // Bluetooth.send((float)_yBottle);
    _findbottle_expiration = millis();
    // Bluetooth.send("REC X/Y");
    return true;
  } else if(millis() - _findbottle_expiration < BOTTLE_DETECTION_EXPIRATION_TIME) {
    return true;
  } else {
    // Bluetooth.send("NOT REC BOT");
    return false;
  }
}

bool Brain::getLed() {
  // int waiting = Serial.available();
  // Bluetooth.send(waiting);
  // if(waiting > 0) {
  //   Bluetooth.send(Serial.readString());
  // }
  
  // delay(5); // ----------------------------TESTING REMOVED!!!!!! ---------------------------------------------------
  bool found_led = false;
  _found_intensity = false;
  while((!found_led || !_found_intensity) && Serial.available() > 0) {
    char charRPI = Serial.read();
    if (charRPI=='g' || charRPI=='r' || charRPI=='b' || charRPI=='j') {
      found_led = true;
      _xLed = Serial.parseInt();
      _colorLed = charRPI;
    } else if(charRPI=='h') {
      _found_intensity = true;
      _colorIntensity = Serial.parseInt();
    }
  }
  // Discard everything in buffer
  while(Serial.read() != -1);

  if(found_led) {
    // Bluetooth.send(_xLed);
    // Bluetooth.send((float)(_colorIntensity));
    _findled_expiration = millis();
    // Bluetooth.send("REC LED");
    return true;
  } else if(millis() - _findled_expiration < LED_DETECTION_EXPIRATION_TIME) {
    _found_intensity = false;
    return true;
  } else {
    // Bluetooth.send("NOT REC LED");
    _found_intensity = false;
    return false;
  }
}

void Brain::setStateRPi(STATE_RPI stateRpi) {
  switch(stateRpi) {
    case RPI_GET_BOTTLES:
      Serial.print("a");
      break;
    case RPI_GO_HOME:
      Serial.print("b");
      break;
    case RPI_SHUTDOWN:
      Serial.print("c");
      break;
    }
}

// Communication with Arduino Micro
bool Brain::getPosFromLinCam() {
  bool found_x = false, found_y = false, found_theta = false;
  int x = 0, y = 0, theta = 0;
  // delay(5);
  // while(Serial.available() > 0) {
  while((!found_x || !found_y || !found_theta) && Serial3.available() > 0) {
    char charRPI = Serial3.read();
    if (charRPI=='x') {
      found_x = true;
      x = Serial3.parseInt();
    } else if(charRPI=='y') {
      found_y = true;
      y = Serial3.parseInt();
    } else if(charRPI=='t') {
      found_theta = true;
      theta = Serial3.parseInt();
    }
  }
  // Discard everything in buffer
  while(Serial3.read() != -1);

 // ------------------------ INSERT THAT ONE VALUE COUNTS FOR SOME SECONDS -------------------------
  if(found_x && found_y && found_theta) {
    _xPosLinCam = x;
    _yPosLinCam = y;
    _thetaLinCam = theta;
    // Bluetooth.send("REC X/Y");
    return true;
  } else {
    // Bluetooth.send("NOT REC BOT");
    return false;
  }
}

bool Brain::getEndLinCam() {
  bool found_f = false, found_b = false;
  int f_nb = 0, b_index = 0;
  // delay(5);
  // while(Serial.available() > 0) {
  while((!found_f || !found_b) && Serial3.available() > 0) {
    char charRPI = Serial3.read();
    if (charRPI=='f') {
      found_f = true;
      f_nb = Serial3.parseInt();
    } else if(charRPI=='b') {
      found_b = true;
      b_index = Serial3.parseInt();
    }
  }
  // Discard everything in buffer
  while(Serial3.read() != -1);

 // ------------------------ INSERT THAT ONE VALUE COUNTS FOR SOME SECONDS -------------------------
  if(found_f && found_b) {
    _f_nb = f_nb;
    _b_index = b_index;
    Bluetooth.send((int)_f_nb);
    Bluetooth.send((float)_b_index);
    return true;
  } else {
    // Bluetooth.send("NOT REC BOT");
    return false;
  }
}

// Communication with WildThumper
void Brain::setSpeed(int speed, int steer) const {
  //Serial.print("Sending speed to WildThumper: ");
  //Serial.println(speed);
  Wire.beginTransmission(4);
  Wire.write("v");                // set speed
  Wire.write((speed>>8)&0xFF);    // sends the most significant byte  
  Wire.write((speed)&0xFF);       // sends the least significant byte
  Wire.endTransmission();


  // Serial.print("Sending steer to WildThumper: ");
  //Serial.println(steer);
  Wire.beginTransmission(4);
  Wire.write("w");                // set steer (tangential)
  Wire.write((steer>>8)&0xFF);    // sends the most significant byte
  Wire.write((steer)&0xFF);       // sends the least significant byte
  Wire.endTransmission();
}

void Brain::stopMotors() const {
  // Serial.println("Stopping Motors.");
  Wire.beginTransmission(4);
  Wire.write("s");
  Wire.endTransmission();
}

void Brain::setSpeedAvoidingObstacles(int speed, int steer, float smoothing) {
  // TODO
  int ir_obst_sl_valid = 0, ir_obst_sr_valid = 0, ir_obst_fl_valid = 0, ir_obst_fr_valid = 0;
  int ir_obst_sl_meas = analogRead(IR_OBST_SL);
  int ir_obst_sr_meas = analogRead(IR_OBST_SR);
  int ir_obst_fl_meas = analogRead(IR_OBST_FL);
  int ir_obst_fr_meas = analogRead(IR_OBST_FR);

  if(ir_obst_sl_meas > IR_OBST_LOWER_THRESHOLD) ir_obst_sl_valid = 1;
  if(ir_obst_sr_meas > IR_OBST_LOWER_THRESHOLD) ir_obst_sr_valid = 1;
  if(ir_obst_fl_meas > IR_OBST_LOWER_THRESHOLD_SPEED) ir_obst_fl_valid = 1;
  if(ir_obst_fr_meas > IR_OBST_LOWER_THRESHOLD_SPEED) ir_obst_fr_valid = 1;

  _speed = (int) (_speed*(smoothing) + (1.0 - smoothing) *
           (speed - (LINEARITY_SPEED*ir_obst_fl_meas-LINEARITY_SPEED*IR_OBST_LOWER_THRESHOLD_SPEED)*ir_obst_fl_valid
                  - (LINEARITY_SPEED*ir_obst_fr_meas-LINEARITY_SPEED*IR_OBST_LOWER_THRESHOLD_SPEED)*ir_obst_fr_valid));

  _steer = (int) (_steer*(smoothing) + (1.0 - smoothing) *
           (steer +(LINEARITY*ir_obst_sl_meas-LINEARITY*IR_OBST_LOWER_THRESHOLD)*ir_obst_sl_valid 
                  -(LINEARITY*ir_obst_sr_meas-LINEARITY*IR_OBST_LOWER_THRESHOLD)*ir_obst_sr_valid
                  +(LINEARITY*ir_obst_fl_meas-LINEARITY*IR_OBST_LOWER_THRESHOLD_SPEED)*ir_obst_fl_valid
                  -(LINEARITY*ir_obst_fr_meas-LINEARITY*IR_OBST_LOWER_THRESHOLD_SPEED)*ir_obst_fr_valid));

  if(_steer > MAX_STEER) _steer = MAX_STEER;
  else if(_steer < -MAX_STEER) _steer = -MAX_STEER;

  if(_speed > MAX_SPEED) {
    _steer = _steer * MAX_SPEED / abs(_speed);
    _speed = MAX_SPEED;
  } else if(_speed < -MAX_SPEED) {
    _steer = _steer * MAX_SPEED / abs(_speed);
    _speed = -MAX_SPEED;
  }

  if(abs(_speed) < LOW_SPEED_STEER_TURNING_THRESHOLD && abs(_steer) < LOW_SPEED_STEER_TURNING_THRESHOLD) {
    if(!_stuck_in_corner) {
      _dir_stuck_in_corner = random(0,2)*2-1;
      _stuck_in_corner = true;
    }
    _speed = -SLOW_SPEED_BACK_WHEN_STUCK_IN_CORNER;
    _steer = _dir_stuck_in_corner * SLOW_STEER;
  } else {
    _stuck_in_corner = false;
  }

  
  setSpeed(_speed, _steer);

  //~ Serial.print("IR SR :");
  //~ Serial.println(ir_obst_sr_meas);
  //~ Serial.print("IR FR :");
  //~ Serial.println(ir_obst_fr_meas);
  //~ Serial.print("IR FL :");
  //~ Serial.println(ir_obst_fl_meas);
  //~ Serial.print("IR SL :");
  //~ Serial.println(ir_obst_sl_meas);
  //~ Serial.print("speed: ");
  //~ Serial.println(_speed);
  //~ Serial.print("steer: ");
  //~ Serial.println(_steer);
  //~ Serial.println("");
  //~ delay(600);
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
  Dynamixel.moveSpeed(DYMX_ID_TRAP, 508, 1000);
  _trapIsOpen = false;
  //Dynamixel.moveSpeed(DYMX_ID_TRAP, 500, 100);
}


int Brain::getXPosLinCam() {return _xPosLinCam;}
int Brain::getYPosLinCam() {return _yPosLinCam;}
int Brain::getThetaLinCam() {return _thetaLinCam;}

/*int Brain::parseNextInt() {
  int parsedInt;
  char input_str[4];
  char input_char = -1;
  byte index = 0;
  while ((input_char = Serial.read()) > 47) {
    input_char = input_char - '0';
    input_str[index] = input_char;
    index++;
  }
  if (index==1)
    parsedInt = input_str[0];
  else if (index==2)
    parsedInt = input_str[0]*10+input_str[1];
  else if (index==3)
    parsedInt = input_str[0]*100+input_str[1]*10+input_str[2];
  return parsedInt;
}*/


void Brain::test() {  
  blink();

  // ------- Testing Belt -------
  // turnBeltForward();

  // ------- Testing Trap -------
  //Serial.println("Move! Go");
  // openTrap();
  // delay(500);
  // closeTrap();
  // delay(500);
  //delay(1000);
  //delay (20); 

  // ------- Testing Comm Rpi - Arduino --------
  // if(millis() - _last_state_change_rpi > 10000 && _state_rpi == 1) {
  //   setStateRPi(RPI_GET_BOTTLES);
  //   _state_rpi = 0;
  //   _last_state_change_rpi = millis();
  // }
  // if(millis() - _last_state_change_rpi > 20000 && _state_rpi == 0) {
  //   _last_state_change_rpi = millis();
  //   _state_rpi = 1;
  //   setStateRPi(RPI_GO_HOME);
  // }
  
  // if(_state_rpi == 0) {
  //   getPosNearestBottle();
  // } else {
  //   getLed();
  // }
  
  //delay(1000);

}