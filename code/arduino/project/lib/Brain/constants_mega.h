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
#define TIME_GOING_STRAIGHT 3000
#define TIME_TURNING 3000

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

#endif

