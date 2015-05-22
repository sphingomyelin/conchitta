#include <math.h>

#define INF 30000

float beacons[4][2]; // Initialize somewhere else

void init_beacons() {
	beacons[0][0] = 8;
	beacons[0][1] = 8;
	beacons[1][0] = 0;
	beacons[1][1] = 8;
	beacons[2][0] = 0;
	beacons[2][1] = 0;
	beacons[3][0] = 8;
	beacons[3][1] = 0;
}

int mod(int a, int b) {
	return (a % b + b) % b;
}


bool triangulation(float angle1, float angle2, float angle3, float x1, float y1, float x2, float y2, float x3, float y3, float &x_R, float &y_R, float &vec_R_x, float &vec_R_y){
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

	//Compute angle
	float vec_R1_x = x1 - x_R;
	float vec_R1_y = y1 - y_R;
	float sqrtR1 = sqrt(vec_R1_x*vec_R1_x+vec_R1_y*vec_R1_y);
	vec_R1_x /= sqrtR1;
	vec_R1_y /= sqrtR1;
	vec_R1_x -= cos(angle1);
	vec_R1_y -= sin(angle1);

	float vec_R2_x = x2 - x_R;
	float vec_R2_y = y2 - y_R;
	float sqrtR2 = sqrt(vec_R2_x*vec_R2_x+vec_R2_y*vec_R2_y);
	vec_R2_x /= sqrtR2;
	vec_R2_y /= sqrtR2;
	vec_R2_x -= cos(angle2);
	vec_R2_y -= sin(angle2);

	float vec_R3_x = x3 - x_R;
	float vec_R3_y = y3 - y_R;
	float sqrtR3 = sqrt(vec_R3_x*vec_R3_x+vec_R3_y*vec_R3_y);
	vec_R3_x /= sqrtR3;
	vec_R3_y /= sqrtR3;
	vec_R3_x -= cos(angle3);
	vec_R3_y -= sin(angle3);

	vec_R_x = (vec_R1_x + vec_R2_x + vec_R3_x)/3.0;
	vec_R_y = (vec_R1_y + vec_R2_y + vec_R3_y)/3.0;

	return true;
}


int findBeacon(int index, float angles[4], float &x_R, float &y_R, float &theta_R) {
	// Finds the index of the beacon corresponding to angle index
	int mse = INF;
	int beacon = -1;

	float vec_meas[2];
	float vec_theory[4][2];

	vec_meas[0] = cos(angles[index]);
	vec_meas[1] = sin(angles[index]);

	//Compute angle between theoretical beacon and heading (to see which beacon corresponds to alpha1)
	for (int i = 0; i < 4; ++i)
	{
		vec_theory[i][0] = beacons[i][0] - x_R;
		vec_theory[i][1] = beacons[i][1] - y_R;
		float mag = sqrt(vec_R1_x*vec_R1_x+vec_R1_y*vec_R1_y);
		vec_theory[i][0] /= mag;
		vec_theory[i][1] /= mag;
		vec_theory[i][0] -= cos(theta_R);
		vec_theory[i][1] -= sin(theta_R);
	}

	for (int i = 0; i < 4; ++i)
	{	
		float temp_x = (vec_theory[i][0] * vec_meas[0]);
		float temp_y = (vec_theory[i][1] * vec_meas[1]);
		float temp = (temp_x) + (temp_y);
		if (temp < mse)
		{
			mse = temp;
			beacon = i;
		}
	}
	return beacon;
}

bool triangulation_4Point(float angles[4], float &x_R, float &y_R, float &theta_R){
	// Does triangulation with 4 measurements of beacons	
	int beacon = findFirstBeacon(0, angles, x_R, y_R, theta_R);

	float temp_pos[4][2];
	float temp_orient[4][2]
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

	// Average the results of the averaged results (12 times averaging of averaging of averaging...)
	x_R	= 0; y_R = 0; theta_R = 0;
	float vec_x = 0, vec_y = 0;
	for (int i = 0; i < 4; ++i)
	{	
		if (valid[i])
		{
			// Average positions
			x_R += temp_pos[i][0];
			y_R += temp_pos[i][1];
			// Average angles
			vec_x -= temp_pos[i][0];
			vec_y -= temp_pos[i][1];
		}
	}
	x_R /= count_valid;
	y_R /= count_valid;
	vec_x /= count_valid;
	vec_y /= count_valid;
	theta_R = atan2(vec_y, vec_x);
}