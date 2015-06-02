#include <Arduino.h>
#include "MultiLinearCamera.h"
#include "calibration.h"

// Definitions
void processBuffer();

// Global variables
int reset_request = 0;
unsigned char *data;

void sendPicture();

void setup()
{
  Serial.begin(9600); //Serial1.begin(9600);

  delay(2000);

  lcam_setup();

  // Retreive images from buffer
  data = lcam_getdata();

}

void loop()
{
  //Wait for requests and take pictures in the meanwhile

  lcam_reset(); // takes about 3540 us
  do {
    //lcam_integrate(50); // takes about 940 us at 50 us
    lcam_integrate(INTEGRATION_TIME);
    //delay(50);

    // load pixel values to lcam_buffer byte array
    lcam_read();

    // get the peaks from the pixel data
  }
  while(!Serial.available());

  // load pixel values to lcam_buffer byte array
  // lcam_read();

  //Requests available: process them
  processBuffer();
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
      case 'C':
        sendPicture();
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

void sendPicture() {
  int i;
  for(i=0;i<NCAMS*NPIXELS;i++)
  {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}