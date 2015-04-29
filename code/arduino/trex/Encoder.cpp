/* 
 * Arduino 101: timer and interrupts
 * 3: Timer2 compare interrupt example. Quadrature Encoder
 * more infos: http://www.letmakerobots.com/node/28278
 * created by RobotFreak 
 *
 * Credits:
 * based on code from Peter Dannegger
 * http://www.mikrocontroller.net/articles/Drehgeber
 */

#include "Encoder.h"

static volatile int timer_count;

static volatile int encDeltaLt, encDeltaRt;
static int lastLt, lastRt;
//static int SpeedL, SpeedR;

volatile int CurrentLeftSpeed=0;
volatile int CurrentRightSpeed=0;
volatile int DiffRight=0;
volatile int DiffLeft=0;
static volatile int encLt, encRt;
static volatile int temp=0;
static volatile int OldL=0;
static volatile int OldR=0;

ISR( TIMER1_COMPA_vect ) {
  //Serial.print("interrupt"); 
  //Serial.println("                  in");
  timer_count++;

  int val, diff;
  val = 0;
  if(digitalRead(ENC_LT_A))
    val = 3;
  if(digitalRead(ENC_LT_B))
    val ^= 1;						// convert gray to binary
  diff = lastLt - val;				// difference last - new
  if( diff & 1 ){					// bit 0 = value (1)
    lastLt = val;					// store new as next last
    encDeltaLt += (diff & 2) - 1;	// bit 1 = direction (+/-)
  }

  val = 0;
  if( digitalRead(ENC_RT_A) )
    val = 3;
  if( digitalRead(ENC_RT_B))
    val ^= 1;						// convert gray to binary
  diff = lastRt - val;				// difference last - new
  if( diff & 1 ){					// bit 0 = value (1)
    lastRt = val;					// store new as next last
    encDeltaRt += (diff & 2) - 1;	// bit 1 = direction (+/-)
  }
  encLt +=QuadratureEncoderReadLt(); 
  encRt +=QuadratureEncoderReadRt();
  if(timer_count==(EncoderTimerFrequency/SpeedTimerFrequency))
  {
    CurrentLeftSpeed=SpeedFilterCoeff*CurrentLeftSpeed+(1-SpeedFilterCoeff)*(encLt-OldL);
    OldL=encLt;
    CurrentRightSpeed=SpeedFilterCoeff*CurrentRightSpeed+(1-SpeedFilterCoeff)*(encRt-OldR);
    OldR=encRt;
    timer_count=0;
  }
}

void QuadratureEncoderInit(void)
{
  int val;

  cli();//stop interrupts

  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  // set timer count for 1khz increments
  OCR1A = 16000000/256/EncoderTimerFrequency;// = (16*10^6) / (1000*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS12);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  pinMode(ENC_LT_A, INPUT);
  pinMode(ENC_RT_A, INPUT);
  pinMode(ENC_LT_B, INPUT);
  pinMode(ENC_RT_B, INPUT);

  val=0;
  if (digitalRead(ENC_LT_A))
    val = 3;
  if (digitalRead(ENC_LT_B))
    val ^= 1;
  lastLt = val;
  encDeltaLt = 0;

  val=0;
  if (digitalRead(ENC_RT_A))
    val = 3;
  if (digitalRead(ENC_RT_B))
    val ^= 1;
  lastRt = val;
  encDeltaRt = 0;
}

int QuadratureEncoderReadLt( void ) // read single step encoders
{
  int val;
  cli();
  val = encDeltaLt;
  encDeltaLt = 0;
  sei();
  return -val; //- because of inversed wheel, counts since last call
}

int QuadratureEncoderReadRt( void )	// read single step encoders
{
  int val;

  cli();
  val = encDeltaRt;
  encDeltaRt = 0;
  sei();
  return val; // counts since last call
}

int GetSpeedRight(void)
{return CurrentRightSpeed;}
int GetSpeedLeft(void)
{return CurrentLeftSpeed;}
int GetPosLeft(void)
{return encLt;}
int GetPosRight(void)
{return encRt;}
