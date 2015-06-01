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
  float getTheta();

  void setX(float new_x);
  void setY(float new_y);
  void setTheta(float new_theta);

  private:
  int peak[5][2];
  int filtered_data[NCAMS*NPIXELS];
  const int mask_highpass[13] = {-1, -1, -1, -1, -1, -1, 12, -1, -1, -1, -1, -1, -1};
  const int mask_conv[9] = {1, 5, 16, 23, 28, 23, 16, 5, 1};

  float beacons[4][2]; // Initialize somewhere else
  float x;
  float y;
  float theta;

  float getAngleFromIndex(int index);
  int mod(int a, int b);
  float modf(float a, float b);
  bool triangulation(float angle1, float angle2, float angle3, float x1, float y1, float x2, float y2, float x3, float y3, float &x_R, float &y_R, float &vec_R_x, float &vec_R_y);
  bool triangulation_4Point(float angles[4], int beacon, float &x_R, float &y_R, float &theta_R);
  void computeBeaconVectors(float x_R, float y_R, float theta_R, float vec_theory[4][2]);
  int findBeacon(int pixel_index, float rejection_range, float vec_theory[4][2]);
  int matchingValidPoints(int matchedBeacon[5]);
  void swap(int &a, int &b);

};


#endif