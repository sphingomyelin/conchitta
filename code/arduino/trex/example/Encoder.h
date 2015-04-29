#include "Arduino.h"
//#include "Constant.h"

#define ENC_RT_A 12
#define ENC_RT_B 10
#define ENC_LT_A 4
#define ENC_LT_B 2

int QuadratureEncoderReadLt(void);
int QuadratureEncoderReadRt(void);
void QuadratureEncoderInit(void);	
int GetSpeedRight(void);
int GetSpeedLeft(void);
int GetPosLeft(void);
int GetPosRight(void);
