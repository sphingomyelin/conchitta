#ifndef _ENCODER_
#define _ENCODER_

#include "Arduino.h"
#include "digitalWriteFast.h"
#include "IOpins.h"
//#include "Constant.h"

#define EncoderTimerFrequency 20000
#define SpeedTimerFrequency 100
#define SpeedFilterCoeff 0

int QuadratureEncoderReadLt(void);
int QuadratureEncoderReadRt(void);
void QuadratureEncoderInit(void);	
long GetSpeedRight(void);
long GetSpeedLeft(void);
long GetPosLeft(void);
long GetPosRight(void);

#endif
