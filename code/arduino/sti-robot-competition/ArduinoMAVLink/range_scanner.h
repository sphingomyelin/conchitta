#ifndef range_scanner.h
#define range_scanner.h
#include "DynamixelSerial.h"


#define DYMX_ID 3
#define SCAN_MAX_ANGLE 900
#define SCAN_SIZE  15

#define TURN_INTERVAL 200
#define STEER_GAIN 10
#define MAX_FWD_SPEED 100
#define MAX_REV_SPEED 90
#define TURN_LPF 8

#define US_PIN 7


void initRangeScanner();
void rangeScanUpdate();

void Calculate_Speeds();

#endif
