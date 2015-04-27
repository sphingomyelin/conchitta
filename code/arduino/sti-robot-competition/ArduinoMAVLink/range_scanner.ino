#include "range_scanner.h"

#include "ultrasonic.h"

#include "MAVlink_bridge.h"
#include <mavlink.h>

//#include <Servo.h>

void Set_Speed(int Speed, int Steer) {
    //forward_output.writeMicroseconds(1500+Speed);
    //turn_output.writeMicroseconds(1600+Steer);
    Wire.beginTransmission(1); // transmit to device #4
    Wire.write("s");        // sends five bytes
    Wire.write((Speed>>8)&0xFF);              // sends one byte  
    Wire.write((Speed)&0xFF);              // sends one byte  
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(1); // transmit to device #4
    Wire.write("t");        // sends five bytes
    Wire.write((Steer>>8)&0xFF);              // sends one byte  
    Wire.write((Steer)&0xFF);              // sends one byte  
    Wire.endTransmission();    // stop transmitting
}

int scan_angle;
int scan_direction;
int range_scan[SCAN_SIZE];
int scan_index;

int i;
int Leftspeed=0;
int Rightspeed=0;
int Speed;
int Steer;
int distance;
long last_scan;

//Servo forward_output, turn_output;


void initRangeScanner() {
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  Dynamixel.setEndless(DYMX_ID, OFF);
  scan_angle=0;
  scan_direction=(SCAN_MAX_ANGLE/SCAN_SIZE);
  Speed=0;
  Steer=0;
  last_scan=millis();

  Wire.begin(); // join i2c bus (address optional for master)
//  forward_output.attach(3);  // attaches the servo on pin 2 to the servo object 
//  turn_output.attach(4);  // attaches the servo on pin 3 to the servo object 

}
int scan_angle_to_index(int angle) {
  int index;
  index=(angle+SCAN_MAX_ANGLE)*SCAN_SIZE/(2*SCAN_MAX_ANGLE);
  if (index<0) index=0;
  if (index>=SCAN_SIZE) index=SCAN_SIZE-1;
  return index;
}


void print_range_scan() {
  for (int i=0; i<SCAN_SIZE; i++) {
    Serial.print(range_scan[i]); Serial.print("\t");      
  }
  Serial.print(Speed);Serial.print("\t");      
  Serial.print(Steer);Serial.print("\t");      
  Serial.print("\n");
}



void mavlink_send_range_scan() {
  for (int i=0; i<SCAN_SIZE; i++) {
    mavlink_msg_named_value_int_send(MAVLINK_COMM_0, (millis()+(millis()-last_scan)*i/(SCAN_SIZE+3))/1000.0, "RangeScan", (int)range_scan[i] );
  }


  last_scan=millis();
}


void rangeScanUpdate() {

  int i;
  distance=(int)readUltrasonic(US_PIN);
// keep scanning
  scan_index=scan_angle_to_index(scan_angle);
  range_scan[scan_index]=distance;
  Dynamixel.moveSpeed(DYMX_ID,(500+(6*scan_angle/18)),1000);                          // set servo to default position

  if (scan_angle> SCAN_MAX_ANGLE) {
    scan_direction=-(SCAN_MAX_ANGLE/SCAN_SIZE);
    mavlink_send_range_scan();
  } else {
    if (scan_angle<-SCAN_MAX_ANGLE) {
      scan_direction= (SCAN_MAX_ANGLE/SCAN_SIZE);
      mavlink_send_range_scan();
    }

  }
  scan_angle+=scan_direction;
  
}



void Calculate_Speeds() {
  int left_mean, right_mean;
  int max_range_index;

    max_range_index=0;
  // find best direction
  for (i=0; i<SCAN_SIZE; i++) {
    if (range_scan[i]>range_scan[max_range_index]) {
      max_range_index=i;
    }
  }

  left_mean=0;
  right_mean=0;
  for (i=0; i<SCAN_SIZE; i++) {
    if (i<SCAN_SIZE/2) {
      left_mean+=50/(abs(range_scan[i])+1);
    }
    if (i>SCAN_SIZE/2) {
      right_mean+=50/(abs(range_scan[i])+1);
    }
  }

  
  Speed =max(min(4*( min((range_scan[SCAN_SIZE/2-1] + range_scan[SCAN_SIZE/2] + range_scan[SCAN_SIZE/2+1])/3 , distance)-40), MAX_FWD_SPEED),-MAX_REV_SPEED);
  Steer=max(min(((min(left_mean, 40)-min(right_mean,40))) + (STEER_GAIN*(max_range_index-(SCAN_SIZE/2))), 70),-70) ;
  //Steer=100;
  //Speed=100;
  mavlink_msg_named_value_int_send(MAVLINK_COMM_0, (millis()/1000.0), "Steer", Steer );
  mavlink_msg_named_value_int_send(MAVLINK_COMM_0, (millis()/1000.0), "Speed", Speed );
  mavlink_msg_named_value_int_send(MAVLINK_COMM_0, (millis()/1000.0), "Distance", distance );


  Set_Speed(Speed, Steer);
//  Set_Speed(0, 0);
}
