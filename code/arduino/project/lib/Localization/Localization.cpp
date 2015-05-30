#include "Localization.h"

Localization::Localization() {
  beacons[0][0] = 8;
  beacons[0][1] = 8;
  beacons[1][0] = 0;
  beacons[1][1] = 8;
  beacons[2][0] = 0;
  beacons[2][1] = 0;
  beacons[3][0] = 8;
  beacons[3][1] = 0;

  x = 0.5;
  y = 0.5;
  theta = 0;

  done = 1;
}

float Localization::getPeakAngle(int index) {
  return getAngleFromIndex(peak[index][1]);
}

float Localization::getPeakHeight(int index) {
  return peak[index][0];
}

int Localization::getPeakIndex(int index) {
	return peak[index][1];
}

float Localization::getX() {
  return x;
}

float Localization::getY() {
  return y;
}

bool Localization::calculatePose() {
  // Look at the peaks and decide on quality. Do 4-Point triangulation, 3-Point triangulation accordingly, or discard if unusable.
  int matchedBeacon[] = {-1, -1, -1, -1, -1};
  float beaconVector[4][2];
  computeBeaconVectors(x, y, theta, beaconVector);

  // Find the 4 beacon matches for the 5 measured peaks
  for(int i = 0; i < 5; i++) {
    matchedBeacon[i] = findBeacon(peak[i][1], REJECTION_ANGLE, beaconVector);
  }
  // Count the number of matches
  int match = matchingValidPoints(matchedBeacon);
  // Check if 
  if(match < 3 || match > 4) {
  	return false;
  } else if(match == 4) {
  	float angles[4];
  	int count = 0;
    int correctMatchedBeacon[4]; 

    // Scan through the angles and only retain the ones that matched
  	for(int i = 0; i < 5; i++) {
  		if(matchedBeacon[i] != -1){
  			angles[count] = getPeakAngle(i);
  			correctMatchedBeacon[count] = matchedBeacon[i]; // Only save the indexes of the matched beacons (i.e discard the -1 values)
  			count++;
  		}
  	}
  	triangulation_4Point(angles, correctMatchedBeacon[0], x, y, theta);
  } else if(match == 3) {
    float angles[3];
    int count = 0;
    int correctMatchedBeacon[3];

    // Scan through the angles and only retain the ones that matched
    for(int i = 0; i < 5; i++) {
      if(matchedBeacon[i] != -1){
        angles[count] = getPeakAngle(i);
        correctMatchedBeacon[count] = matchedBeacon[i]; // Only save the indexes of the matched beacons (i.e discard the -1 values)
        count++;
      }
    }
    float vec_x = 0, vec_y = 0;
    triangulation(angles[0], angles[1], angles[2], \
      beacons[correctMatchedBeacon[0]][0], beacons[correctMatchedBeacon[0]][1], \
      beacons[correctMatchedBeacon[1]][0], beacons[correctMatchedBeacon[1]][1], \
      beacons[correctMatchedBeacon[2]][0], beacons[correctMatchedBeacon[2]][1], \
      x, y, vec_x, vec_y);
    theta = atan2(vec_y, vec_x);
  }

  return true;
}

int Localization::mod(int a, int b) {
  return (a % b + b) % b;
}

int Localization::calculatePeaks()
{
  // int p_INF_CAM = angle2px[INF_CAM];
  // int p_sup = angle2px[sup];
  for(int i = 0; i < 5; i++) {
    peak[i][0] = -1;
    peak[i][1] = INF;
  }

  Serial.println("Pixels: ");
  for(int i=0;i<NCAMS*NPIXELS;i++)
  {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println("");

  // ---------- Removed the filtering because it removed information instead of making it clearer. ---------

  // some filtering first (averaging with window width WINDOW_WIDTH)
  /*int running_sum = 0;
  for(int i = 0; i < WINDOW_WIDTH; i++) {
    running_sum += data[mod((i-WINDOW_WIDTH/2) , (NCAMS*NPIXELS))];
  }
  for(int i = 0; i < NCAMS*NPIXELS; i++)
  {
    filtered_data[i] = running_sum/WINDOW_WIDTH;
    running_sum += -data[mod((i-WINDOW_WIDTH/2) , (NCAMS*NPIXELS))] + data[mod((i+1+WINDOW_WIDTH/2) , (NCAMS*NPIXELS))];
  }

  Serial.println("Filtered pixels: ");
  for(int i=0;i<NCAMS*NPIXELS;i++)
  {
    Serial.print(filtered_data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println("");*/


  // extract peaks 
  // and save only the highest 5 peaks
  int min = INF_CAM;
  float max = -INF_CAM;
  int max_count = 0;
  int maxpos = 0;
  //int minpos;
  bool lookformax = true;
  int current = 0;
  for(int i = 0; i < NCAMS*NPIXELS; i++) {
    current = data[i]; // inserted raw data here instead of filtered data.
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

  // Sort array with Selection Sort
  for (int i = 0; i < 5; i++) {
    // nSmallestIndex is the index of the smallest element
    // we've encountered so far.
    int nSmallestIndex = i;

    // Search through every element starting at nStartIndex+1
    for (int j = i + 1; j < 5; j++)
    {
      // If the current element is smaller than our previously found smallest
      if (peak[j][1] < peak[nSmallestIndex][1])
          // Store the index in nSmallestIndex
          nSmallestIndex = j;
    }

    // Swap our start element with our smallest element
    swap(peak[i][0], peak[nSmallestIndex][0]);
    swap(peak[i][1], peak[nSmallestIndex][1]);
  }

  // Put the INF position of the pixels to -1
  for (int i = 0; i < 5; i++) {
    if(peak[i][1] == INF) {
      peak[i][1] = -1;
    }
  }

  return 1; 
}



bool Localization::triangulation(float angle1, float angle2, float angle3, float x1, float y1, float x2, float y2, float x3, float y3, float &x_R, float &y_R, float &vec_R_x, float &vec_R_y){
  // This function triangulates from 3 angles associated with 3 beacons. Returns a position and an orientation.

  //Compute modified beacon coordinates (x'_i, y'_i)
  float x1p = x1 - x2;
  float y1p = y1 - y2;
  float x3p = x3 - x2;
  float y3p = y3 - y2;

  //Compute cotangents of angle differences T_ij = cot(a_ij)
  float T_12 = 1.0/tan(angle2 - angle1);
  float T_23 = 1.0/tan(angle3 - angle2);
  float T_31 = (1.0 - T_12*T_23)/(T_12 + T_23);

  //Compute modified circle centers coordinates {x'_ij, y'_ij}
  float xp_12 = x1p + T_12*y3p;
  float yp_12 = y1p - T_12*x1p;
  float xp_23 = x3p - T_23*y3p;
  float yp_23 = y3p + T_23*x3p;
  float xp_31 = (x3p + x1p) + T_31*(y3p - y1p);
  float yp_31 = (y3p + y1p) - T_31*(x3p - x1p);

  //Compute k'_31
  float kp_31 = x1p*x3p + y1p*y3p + T_31*(x1p*y3p - x3p*y1p);

  //Compute denominator D
  float D = (xp_12 - xp_23)*(yp_23 - yp_31) - (yp_12 - yp_23)*(xp_23 - xp_31);
  if (D == 0)
    return false;

  //Compute robot position {x_R, y_R}

  x_R = x2 + kp_31*(yp_12 - yp_23)/D;
  y_R = y2 + kp_31*(xp_23 - xp_12)/D;

  //Compute angle (made changes to the substraction of the measured angle, it should have been a rotation)
  float vec_R1_x = x1 - x_R;
  float vec_R1_y = y1 - y_R;
  float sqrtR1 = sqrt(vec_R1_x*vec_R1_x+vec_R1_y*vec_R1_y);
  vec_R1_x /= sqrtR1;
  vec_R1_y /= sqrtR1;
  // Rotate vector by - angle1 so as to only be left with heading angle
  float temp_vec_R[2];
  temp_vec_R[0] = cos(angle1)*vec_R1_x + sin(angle1)*vec_R1_y;
  temp_vec_R[1] = -sin(angle1)*vec_R1_x + cos(angle1)*vec_R1_y;
  vec_R1_x = temp_vec_R[0];
  vec_R1_y = temp_vec_R[1];

  float vec_R2_x = x2 - x_R;
  float vec_R2_y = y2 - y_R;
  float sqrtR2 = sqrt(vec_R2_x*vec_R2_x+vec_R2_y*vec_R2_y);
  vec_R2_x /= sqrtR2;
  vec_R2_y /= sqrtR2;
  // Rotate vector by - angle2 so as to only be left with heading angle
  temp_vec_R[0] = cos(angle2)*vec_R2_x + sin(angle2)*vec_R2_y;
  temp_vec_R[1] = -sin(angle2)*vec_R2_x + cos(angle2)*vec_R2_y;
  vec_R2_x = temp_vec_R[0];
  vec_R2_y = temp_vec_R[1];

  float vec_R3_x = x3 - x_R;
  float vec_R3_y = y3 - y_R;
  float sqrtR3 = sqrt(vec_R3_x*vec_R3_x+vec_R3_y*vec_R3_y);
  vec_R3_x /= sqrtR3;
  vec_R3_y /= sqrtR3;
  // Rotate vector by - angle3 so as to only be left with heading angle
  temp_vec_R[0] = cos(angle3)*vec_R3_x + sin(angle3)*vec_R3_y;
  temp_vec_R[1] = -sin(angle3)*vec_R3_x + cos(angle3)*vec_R3_y;
  vec_R3_x = temp_vec_R[0];
  vec_R3_y = temp_vec_R[1];

  // Calculate average vector
  vec_R_x = (vec_R1_x + vec_R2_x + vec_R3_x)/3.0;
  vec_R_y = (vec_R1_y + vec_R2_y + vec_R3_y)/3.0;

  return true;
}

bool Localization::computeBeaconVectors(float x_R, float y_R, float theta_R, float vec_theory[4][2]) {
  // Compute angle between theoretical beacons and heading
    // Get vectors from previous pose and theoretical beacons

  for (int i = 0; i < 4; ++i)
  {
    // Calculate vector between robot and beacon
    vec_theory[i][0] = beacons[i][0] - x_R;
    vec_theory[i][1] = beacons[i][1] - y_R;
    float mag = sqrt(vec_theory[i][0]*vec_theory[i][0]+vec_theory[i][1]*vec_theory[i][1]);
    vec_theory[i][0] /= mag;
    vec_theory[i][1] /= mag;

    // Rotate vector to be in the referential with heading along x-axis (i.e same as the peak measurements)
    float temp_vec_theory[2];
    temp_vec_theory[0] = cos(theta_R)*vec_theory[i][0] + sin(theta_R)*vec_theory[i][1];
    temp_vec_theory[1] = -sin(theta_R)*vec_theory[i][0] + cos(theta_R)*vec_theory[i][1];

    vec_theory[i][0] = temp_vec_theory[0];
    vec_theory[i][1] = temp_vec_theory[1];
 
    // vec_theory[i][0] -= cos(theta_R); // These substractions made no sense :)
    // vec_theory[i][1] -= sin(theta_R);
  }

  return true;

}

int Localization::findBeacon(float angle, float rejection_range, float vec_theory[4][2]) {
  // Finds the index of the beacon corresponding to angle index
  int mse = -INF;
  int beacon = -1;

  float vec_meas[2];
  

  vec_meas[0] = cos(angle);
  vec_meas[1] = sin(angle);

  // Compute angle between theoretical beacon and heading (to see which beacon corresponds to alpha1)
    // Get vectors from previous pose and theoretical beacons
  // I thought that it would be better to have this as a separate function, since it otherwise would be calculated everytime we ran this function which would be useless


    // Calculate metric (dot product) to determine best fit, get minimum //This made no sense originally, since the dot product is maximum when vectors are colinear :)
  for (int i = 0; i < 4; ++i)
  { 
    float temp_x = (vec_theory[i][0] * vec_meas[0]);
    float temp_y = (vec_theory[i][1] * vec_meas[1]);
    float temp = (temp_x) + (temp_y);
    if (temp > mse)
    {
      mse = temp;
      beacon = i;
    }
  }

  if(abs(acos(mse)) < rejection_range) {
    return beacon;
  } else {
    return -1;
  }

}

int Localization::matchingValidPoints(int matchedBeacon[5]) {
  int nb_matched = 0;

  for(int i=0; i<5; i++) {
    if(matchedBeacon[i] != -1) {
      // count the valid beacons
      nb_matched++;
      // check for duplicates
      for(int j=i+1; j<5; j++) {
        if(matchedBeacon[i] != -1) {
          if(matchedBeacon[i] == matchedBeacon[j])
            return -1;
        }
      }
    }
  }
  
  return nb_matched;
}


bool Localization::triangulation_4Point(float angles[4], int beacon, float &x_R, float &y_R, float &theta_R){
  // Does triangulation with 4 measurements of beacons  

  float temp_pos[4][2];
  float temp_orient[4][2];
  int count_valid = 0;
  bool valid[4];
  for (int i = 0; i < 4; ++i)
  {
    int isSuccessful = triangulation(angles[i], angles[(i+1)%4], angles[(i+2)%4], \
      beacons[(beacon+i)%4][0], beacons[(beacon+i)%4][1], \
      beacons[(beacon+i+1)%4][0], beacons[(beacon+i+1)%4][1], \
      beacons[(beacon+i+2)%4][0], beacons[(beacon+i+2)%4][1], \
      temp_pos[i][0], temp_pos[i][1], temp_orient[i][0], temp_orient[i][1]);
    if(isSuccessful) {
      valid[i] = true;
      count_valid++;
    } else {
      valid[i] = false;
    }
  }

  if(count_valid == 0) {
    return false;
  }

  // Average the results of the averaged results (12 times averaging of averaging of averaging...)
  x_R = 0; y_R = 0; theta_R = 0;
  float vec_x = 0, vec_y = 0;
  for (int i = 0; i < 4; ++i)
  { 
    if (valid[i])
    {
      // Average positions
      x_R += temp_pos[i][0];
      y_R += temp_pos[i][1];
      // Average angles
      vec_x += temp_orient[i][0]; //Why were you subtracting here originally????????????????
      vec_y += temp_orient[i][1];
    }
  }
  x_R /= count_valid;
  y_R /= count_valid;
  vec_x /= count_valid;
  vec_y /= count_valid;
  theta_R = atan2(vec_y, vec_x);

  return true;
}

void Localization::sendPicture(int cam)
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



float Localization::getAngleFromIndex(int index) {
  return mod((index - 61)* -2*PI/612.0, 2*PI);
}



void Localization::swap(int &a, int &b) {
  int temp = a;
  a = b;
  b = temp;
}