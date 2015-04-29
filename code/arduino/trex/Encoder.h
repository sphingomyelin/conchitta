#ifndef _ENCODER_
#define _ENCODER_

#include "Arduino.h"
//#include "Constant.h"

#define EncoderTimerFrequency 64
#define SpeedTimerFrequency 64
#define SpeedFilterCoeff 1

#define ENC_RT_A 10
#define ENC_RT_B 12
#define ENC_LT_A 2
#define ENC_LT_B 4

int QuadratureEncoderReadLt(void);
int QuadratureEncoderReadRt(void);
void QuadratureEncoderInit(void);	
int GetSpeedRight(void);
int GetSpeedLeft(void);
int GetPosLeft(void);
int GetPosRight(void);

#endif
