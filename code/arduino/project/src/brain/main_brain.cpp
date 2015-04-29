#include <Arduino.h>
#include "Brain.h"
#include "constants_mega.h"
#include "scheduler.h"


Brain brain;
void initDynamixel();
bool bottle;
unsigned long bottleStartTime;

void setup() {

  // Serial USB connection to computer
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start!");
  Serial.println("");

  // Set the LED as output
  pinMode(LED, OUTPUT);

  // Set the A7 as input from the IR sensor
  pinMode(A7, INPUT);

  // Dynamixel
  initDynamixel();

  // Set up the Wire for I2C
  Wire.begin();

  // Check for bottles (test)
  bottle = false;
  bottleStartTime = -1;

  // Scheduler (NOT USED)
  //init_scheduler();
  //register_task(0, &task_blink, 20);

}

void loop() {

  //run_scheduler_update();
  //delay(1000);
  //brain.test();
  // brain.test();
  delay(10000);
  // Serial.println("move yo 2!");


// ---------- TEST DYNAMIXEL -------------
  // for(int i = 0; i < 255; i++) {
  //   Serial.print("id: ");
  //   Serial.print(i);
  //   Serial.print(" ");
  //   Serial.println(Dynamixel.ping(i));
  // }


// ---------- TEST IR ------------
  // Read analog signal from IR sensor, determine bottle and send result to
  // serial. It waits for 4 seconds after having detected a bottle to avoid
  // counting a bottle twice.
  
  // if(bottleStartTime == -1) {
  //   if(analogRead(A7) > 300) {
  //     bottle = true;
  //     bottleStartTime = millis();
  //   }
  // } else if(millis() - bottleStartTime > 4000) {
  //   bottleStartTime = -1;
  //   bottle = false;
  // }

  // Serial.print(bottle);
  // Serial.print("  ");
  // Serial.println(bottleStartTime);

  // ---------- TEST I2C ---------------
  Serial.println("Testing motors over I2C");
  brain.test();

  // while(Wire.available()) { 
  //   char c = Wire.read(); // receive a byte as character
  //   Serial.print(c);         // print the character
  // }
  // Wire.requestFrom(2, 6);
  // while(Wire.available())    // slave may send less than requested
  // { 
  //   char c = Wire.read(); // receive a byte as character
  //   Serial.print(c);         // print the character
  // }



}

void initDynamixel() {
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  Dynamixel.setEndless(DYMX_ID_R, ON);
  Dynamixel.setEndless(DYMX_ID_L, ON);

  delay(1000);
}
