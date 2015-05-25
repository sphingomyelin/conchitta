#ifndef _LOCALIZATION_
#define _LOCALIZATION_

#include <Arduino.h>
#include "calibration.h"

class Localization
{
  public:
  int done;
  unsigned char *data;

  Localization();
  float getPeakAngle(int index);
  float getPeakHeight(int index);
  int getPeakIndex(int index);
  int calculatePeaks();
  bool calculatePose();
  void sendPicture(int cam);
  float getX();
  float getY();

  private:
  int peak[5][2];
  int filtered_data[NCAMS*NPIXELS];

  float beacons[4][2]; // Initialize somewhere else
  float x;
  float y;
  float theta;

  float getAngleFromIndex(int index);
  int mod(int a, int b);
  bool triangulation(float angle1, float angle2, float angle3, float x1, float y1, float x2, float y2, float x3, float y3, float &x_R, float &y_R, float &vec_R_x, float &vec_R_y);
  bool triangulation_4Point(float angles[4], int beacon, float &x_R, float &y_R, float &theta_R);
  int findBeacon(float angle, float rejection_range, float x_R, float y_R, float theta_R);

};


#endif