#include <Arduino.h>
#include "Brain.h"
#include "constants_mega.h"
//#include "scheduler.h"

Brain brain;
void initDynamixel();
void initBluetooth();
bool bottle;
unsigned long bottleStartTime;

void setup() {

  // Serial USB connection to computer
  Serial.begin(115200);
  Serial.println("Start!");
  Serial.println("");

  // Serial2 Bluetooth connection to smartphone
  // 115200 = baud rate Bluetooth module
  initBluetooth();  

  // Set the LED as output
  pinMode(LED, OUTPUT);

  // Set the A7 as input from the IR sensor
  pinMode(A7, INPUT);

  // Dynamixel
  initDynamixel();

  // Set up the Wire for I2C
  Wire.begin();

  // Check for bottles (test)
  // bottle = false;
  // bottleStartTime = -1;

  // Scheduler (NOT USED)
  //init_scheduler();
  //register_task(0, &task_blink, 20);

}

void loop() {

  //run_scheduler_update();
  //delay(1000);
  //brain.test();
  // brain.test();
  //delay(10000);
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
  
  if(bottleStartTime == -1) {
    if(analogRead(A7) > 180) {
      bottle = true;
      bottleStartTime = millis();
    }    
    Serial.print(bottle);
    Serial.print("  ");
    Serial.print(bottleStartTime);
    Serial.print("  ");
    Serial.println(analogRead(A7));

  } else if(millis() - bottleStartTime > 100) {
    bottleStartTime = -1;
    bottle = false;
  }

  // ---------- TEST I2C ---------------
  // brain.test();

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

  //brain.run();

  Bluetooth.send(5);
  Bluetooth.send(4.89f);
  Bluetooth.send("Hello World!");
  //if(Serial2.available()) {
  //  Bluetooth.process();
  //}
}


void initDynamixel() {
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  // Dynamixel.setEndless(DYMX_ID_R, ON);
  // Dynamixel.setEndless(DYMX_ID_L, ON);
  // Dynamixel.setEndless(DYMX_ID_TRAP, OFF);

  delay(1000);
}

void initBluetooth() {
  Serial2.begin(115200);
  while(Serial2.available())  Serial2.read();// empty RX buffer
}