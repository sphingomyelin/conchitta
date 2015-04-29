#ifndef _ENCODER_
#define _ENCODER_

#include "Arduino.h"
//#include "Constant.h"

#define EncoderTimerFrequency 4096
#define SpeedTimerFrequency 4096
#define SpeedFilterCoeff 1

#define ENC_RT_A 10
#define ENC_RT_B 12
#define ENC_LT_A 2
#define ENC_LT_B 4

int QuadratureEncoderReadLt(void);
int QuadratureEncoderReadRt(void);
void QuadratureEncoderInit(void);	
long GetSpeedRight(void);
long GetSpeedLeft(void);
long GetPosLeft(void);
long GetPosRight(void);

#endif
