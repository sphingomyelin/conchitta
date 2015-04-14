#include <Arduino.h>
#include "Brain.h"
#include "constants_mega.h"
#include "scheduler.h"

Brain brain;

// Tasks predefinition
void task_blink();

void setup() {

	Serial.begin(115200);
	delay(1000);

	// Set the LED as output
	pinMode(LED, OUTPUT);

  // Scheduler (NOT USED)
  //init_scheduler();
  //register_task(0, &task_blink, 20);

}

void loop() {

	//run_scheduler_update();
  //delay(1000);

}


// Tasks
void task_blink() {
  brain.blink();
}