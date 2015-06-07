#include <Arduino.h>
#include "MultiLinearCamera.h"
#include "Localization.h"

// Definitions
void processBuffer();

// Global variables
int reset_request = 0;
Localization localization;


void setup()
{
  Serial.begin(115200); //Serial1.begin(9600);
  Serial1.begin(115200);

  delay(5000);
  Serial.println("Start...");

  lcam_setup();

  // Retreive images from buffer
  localization.data = lcam_getdata();

}

void loop()
{
  //Wait for requests and take pictures in the meanwhile
  lcam_reset();
  do {
    Serial.println("----------------------------------------------");
    Serial.println(millis());
    //lcam_integrate(50); // takes about 940 us at 50 us
    lcam_integrate(INTEGRATION_TIME);
    //delay(50);
    //lcam_reset(); // takes about 3540 us

    // load pixel values to lcam_buffer byte array
    lcam_read();

    // get the peaks from the pixel data
    localization.calculatePeaks();
    // Display the peaks
    for(int i = 0; i < 4; i++) {
      Serial.print("Peak height ");
      Serial.print(localization.getPeakHeight(i));
      Serial.print(" at index/angle ");
      Serial.print(localization.getPeakIndex(i));
      Serial.print("/");
      Serial.println(localization.getPeakAngle(i));
    }

    // Reinitializing pose estimate to START values for debugging
    localization.setX(X_START);
    localization.setY(Y_START);
    localization.setTheta(THETA_START);
    
    // Update the pose
    localization.calculatePose();

    Serial.print("Position: (");
    Serial.print(localization.getX());
    Serial.print(", ");
    Serial.print(localization.getY());
    Serial.print("), orientation: ");
    Serial.println(localization.getTheta());

    Serial1.print("x");
    Serial1.println(localization.getX());
    Serial1.print("y");
    Serial1.println(localization.getY());
    Serial1.print("t");
    Serial1.println(localization.getTheta());
    
  }
  while(!Serial.available());

  // load pixel values to lcam_buffer byte array
  // lcam_read();

  //Requests available: process them
  // processBuffer();

  /*if(!(localization.done))
  {
    localization.done = 1;
    //localization.calculatePeaks();
    for(int i = 0; i < 5; i++) {
      // For debugging
      // End debugging
      Serial.print(localization.getPeakHeight(i));
      Serial.print(" at index ");
      Serial.println(localization.getPeakIndex(i));
    }
    Serial.print("(");
    Serial.print(localization.getX());
    Serial.print(", ");
    Serial.println(localization.getY());
    Serial.print(")");
  }*/
}


//current function reads the communication buffer and extracts the messages
// void processBuffer()
// {
//   while(Serial.available())
//   {
//     char input = Serial.read();

//     int cam;
//     //int i;

//     switch (input) {

//       // case 'P': //Find a peak in a given range.
//       //   localization.done = 0;

//       //   //localization.INF_CAM = Serial.parseInt();
//       //   //localization.sup = Serial.parseInt();
//       //   break;

//       // case 'M': //Find multiple peaks in the given ranges
//       //   mlocalization.done = 0;

//       //   for(i=0;i<NBEACONS;i++)
//       //   {
//       //     mlocalization.INF_CAMs[i] = Serial.parseInt();
//       //     mlocalization.sups[i] = Serial.parseInt();
//       //   }
//       //   break;

//       case 'C':
//         cam = Serial.parseInt();
//         localization.sendPicture(cam);
//         break;

//       case 'R': //Reset
//         lcam_setup();
//         reset_request = 0;
//         break;

//       default: 
//         break;
//     }
//   }
// }

