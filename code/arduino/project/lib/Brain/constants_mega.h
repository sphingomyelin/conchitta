#ifndef _CONCHITTA_CONSTANTS_MEGA_
#define _CONCHITTA_CONSTANTS_MEGA_

enum STATE
{
  START,
  GET_BOTTLES,
  GO_HOME,
  RELEASE_BOTTLES,
  AVOID_OBSTACLE,
  AVOID_OBSTACLE_HOME,
};

#define SEND(x) Serial.println(x);

#define LED 13

#define MAX_SPEED 120
#define MAX_BOTTLES 6
#define TIME_GOING_STRAIGHT 10000
#define TIME_TURNING_MAX 3000
#define TIME_TURNING_MIN 500
#define RUNTIME 3000

#define TIME_END_GO_HOME 8 * 60000
#define TIME_TURNING_RELEASE_BOTTLES 1500

#define DYMX_ID_R 4
#define DYMX_ID_L 6
#define DYMX_ID_TRAP 1

// TODO: define IR
#define IR_FRONT 0
#define IR_TOP 0
#define IR_CONTAINER 0

#define IR_FRONT_TH 280
#define IR_TOP_TH 280
#define IR_CONTAINER_TH 280

#define IR_OBST_FL 0
#define IR_OBST_FR 0
#define IR_OBST_SL 0
#define IR_OBST_SR 0

#define IR_OBST_FL_TH 0
#define IR_OBST_FR_TH 0
#define IR_OBST_SL_TH 0
#define IR_OBST_SR_TH 0

#endif

