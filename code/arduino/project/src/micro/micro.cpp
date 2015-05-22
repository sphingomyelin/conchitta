
// #define TAB Serial.print('\t');
// #define LN Serial.print('\n');
// #define ICI Serial.println("ICI");
// #define OULA Serial.println("OU LA");

#include <Arduino.h>
#include "MultiLinearCamera.h"
#include "calibration.h"

//Classes definitions

void processBuffer();
void sendPicture(int);


struct peak_request
{
  int done;

  int peak[5][2];
  int filtered_data[NCAMS*NPIXELS];
  //int peak_index;
  long mod(long a, long b)
  { return (a%b+b)%b; }
  int process(unsigned char *data);

};


//Global variables

int reset_request = 0;

struct peak_request pr;
//struct multiple_peak_request mpr;

unsigned char *data;

//int t_int = 10;
//int led = 13;




/*struct multiple_peak_request
{
  int INF_CAMs[NBEACONS];
  int sups[NBEACONS];
  int done;

  unsigned char peak[NBEACONS];
  int peak_index[NBEACONS];

  int process(unsigned char *data);

  void send_peak(void);

};*/

//Methods

int peak_request::process(unsigned char *data)
{
  // int p_INF_CAM = angle2px[INF_CAM];
  // int p_sup = angle2px[sup];
  for(int i = 0; i < 5; i++) {
    peak[i][0] = 0;
    peak[i][1] = 0;
  }

  // some filtering first (averaging with window width WINDOW_WIDTH)
  int running_sum = 0;
  for(int i = 0; i < WINDOW_WIDTH; i++) {
    running_sum += data[mod((i-WINDOW_WIDTH/2) , (NCAMS*NPIXELS))];
  }
  for(int i = 0; i < NCAMS*NPIXELS; i++)
  {
    filtered_data[i] = running_sum/WINDOW_WIDTH;
    running_sum += -data[mod((i-WINDOW_WIDTH/2) , (NCAMS*NPIXELS))] + data[mod((i+1+WINDOW_WIDTH/2) , (NCAMS*NPIXELS))];
  }

  Serial.print(0);
  Serial.print(": ");
  for(int i=0;i<NCAMS*NPIXELS;i++)
  {
    Serial.print(filtered_data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println("");

  // extract peaks 
  // and save only the highest 4 peaks
  int min = INF_CAM;
  float max = -INF_CAM;
  int max_count = 0;
  int maxpos = 0;
  //int minpos;
  bool lookformax = true;
  int current = 0;
  for(int i = 0; i < NCAMS*NPIXELS; i++) {
    current = filtered_data[i];
    if(current > max) {
      max = current;
      maxpos = i;
    } else if(current == max) {
      max_count++;
    }
    if(current < min) {
      min = current;
      //minpos = i;
    }

    if(lookformax) {
      if(current < max-PEAK_DELTA) {
        // There's a maximum at position maxpos
        int minpos_peak = 0;
        int min_peak = INF_CAM;
        for(int j = 0; j < 5; j++) {
          if(peak[j][0] < min_peak) {
            min_peak = peak[j][0];
            minpos_peak = j;
          }
        }
        if(max > min_peak) {
          peak[minpos_peak][0] = max;
          peak[minpos_peak][1] = maxpos+(max_count)/2;
        }

        Serial.print("Found peak: ");
        Serial.print(max);
        Serial.print(" at ");
        Serial.println(maxpos+(max_count)/2);

        min = current;
        //minpos = i;
        max_count = 0;
        lookformax = false;
      }
    } else {
      if(current > min+PEAK_DELTA) {
        // there's a minimum at position minpos
        //mintab = [mintab; mnpos mn];
        max = current;
        maxpos = i;
        lookformax = true;
      }
    }
  }


  return 1; 
}


/*int multiple_peak_request::process(unsigned char *data)
{

  for(int j = 0; j>NBEACONS; j++)
  {

    peak[j] = 0;
    peak_index[j] = 0;
    int p_INF_CAM = angle2px[INF_CAMs[j]];
    int p_sup = angle2px[sups[j]];

    for(int i = p_INF_CAM;i!=p_sup+1;i++)
    {
      if(i==NCAMS*NPIXELS)
        i=0;

      if(data[i]>peak[j])
      {
        peak[j] = data[i];
        peak_index[j] = i;
      }
    }
    return 1;
  }
}*/

/*void multiple_peak_request::send_peak(void)
{
  for(int i=0;i<NBEACONS;i++)
  {
    Serial.println(peak_index[i]);
  }
}*/


void setup()
{
  Serial.begin(115200);
  //Serial1.begin(9600);

  delay(5000);
  Serial.println("Start...");

  //pinMode(led, OUTPUT); 

  lcam_setup();
  //mpr.done=1;
  pr.done=1;

  // Retreive images from buffer
  data = lcam_getdata();

}

void loop()
{
  //Wait for requests and take pictures in the meanwhile
  Serial.println(millis());
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);               // wait for a second


  //int micros_0, micros_1, micros_2;
  do
  {
    //lcam_integrate(50); // takes about 940 us at 50 us
    lcam_integrate(2000);
    //delay(50);
    lcam_reset(); // takes about 3540 us
    delay(2);
  }
  while(!Serial.available());

  // load pixel values to lcam_buffer byte array
  lcam_read();

  //Requests available: process them
  processBuffer();

  if(!(pr.done))
  {
    pr.done = 1;
    pr.process(data);
    //sendPicture(1);
    for(int i = 0; i < 5; i++) {
      // For debugging
      // End debugging
      Serial.print(pr.peak[i][0]);
      Serial.print(" at index ");
      Serial.println(pr.peak[i][1]);
    }
  }

  /*if(!(mpr.done))
  {
    mpr.done = 1;
    mpr.process(data);
    mpr.send_peak();
  }*/


  //delay(50);
}

//current function reads the communication buffer and extracts the messages
void processBuffer()
{
  while(Serial.available())
  {
    char input = Serial.read();

    int cam;
    //int i;

    switch (input) {

      case 'P': //Find a peak in a given range.
        pr.done = 0;

        //pr.INF_CAM = Serial.parseInt();
        //pr.sup = Serial.parseInt();
        break;

      /*case 'M': //Find multiple peaks in the given ranges
        mpr.done = 0;

        for(i=0;i<NBEACONS;i++)
        {
          mpr.INF_CAMs[i] = Serial.parseInt();
          mpr.sups[i] = Serial.parseInt();
        }
        break;*/

      case 'C':
        cam = Serial.parseInt();
        sendPicture(cam);
        break;

      case 'R': //Reset
        lcam_setup();
        reset_request = 0;
        break;

      default: 
        break;
    }
  }
}


void sendPicture(int cam)
{
  int j,i;
  //cam = 6;
  if(cam==0)
  {
    for(j=0;j<6;j++)
    {
      Serial.print(j);
      Serial.print(": ");
      for(i=0;i<NPIXELS;i++)
      {
        Serial.print(data[j*NPIXELS+i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  else if(cam>0 && cam<=6)
  {
    for(i=0;i<NPIXELS;i++)
    {
      Serial.print(data[(cam-1)*NPIXELS+i]);
      Serial.print(" ");

    }
    Serial.println();
  }
}




/*
   int window_peak(unsigned char CurrentPosition, unsigned char data)
   {

   int i;
   unsigned char Maxi=0;
   unsigned char IndexMaxi=CurrentPosition;
// if(CurrentPosition>WINDOW_WIDTH && CurrentPosition<(101-WINDOW_WIDTH))

for(i=CurrentPosition-WINDOW_WIDTH;i<=CurrentPosition+WINDOW_WIDTH;i++)
{
if(i<0)
i=0;
else if(i>101)
break;

if(data[i]>Maxi)
{
Maxi=data[i];
IndexMaxi=i;
}
}
int Valide=0;
long Sum1=0;
long Sum2=0;
Serial.print(IndexMaxi);
Serial.print("\t");
Serial.print("\t");
for(i=IndexMaxi-WINDOW_WIDTH;i<=IndexMaxi+WINDOW_WIDTH;i++)
{  


if(i<0)
i=0;
else if(i>101)
{
i=101;
Sum1+=i*data[i];
Sum2+=data[i];

break;
}
Sum1+=i*data[i];
Sum2+=data[i];
// Serial.print(i);
// Serial.print("\t");
}
Serial.print("Sum");
Serial.print(Sum1);
Serial.print("\t");
Serial.print("Valide");
Serial.print(Valide);
Serial.print("\t");
Serial.print(Sum1/Sum2);
Serial.print("\t");
return(Sum1/Sum2);
}

*/
