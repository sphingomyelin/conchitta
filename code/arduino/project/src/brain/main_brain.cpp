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

  // Set the A0 as input
  pinMode(A7, INPUT);

  // Dynamixel
  initDynamixel();

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
  delay(2);
  // Serial.println("move yo 2!");

  // for(int i = 0; i < 255; i++) {
  //   Serial.print("id: ");
  //   Serial.print(i);
  //   Serial.print("   ");
  //   Serial.println(Dynamixel.ping(i));
  // }

  // Read analog and send it to serial
  if(bottleStartTime == -1) {
    if(analogRead(A7) > 300) {
      bottle = true;
      bottleStartTime = millis();
    }
  } else if(millis() - bottleStartTime > 4000) {
    bottleStartTime = -1;
    bottle = false;
  }

  Serial.print(bottle);
  Serial.print("  ");
  Serial.println(bottleStartTime);

}

void initDynamixel() {
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  Dynamixel.setEndless(DYMX_ID_R, ON);
  Dynamixel.setEndless(DYMX_ID_L, ON);

  delay(1000);
}