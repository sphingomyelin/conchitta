#include <Wire.h>

#include "MAVlink_bridge.h"
#include <mavlink.h>
#include "ultrasonic.h"
#include "range_scanner.h"
#include "scheduler.h"

// Arduino MAVLink test code.


void Hello() {
  Serial.print(millis());
  Serial.print("\n");
}

void mavlink_heartbeat() {
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);
}


void setup() {
  Serial.begin(57600);
  initRangeScanner();
  init_scheduler();

  register_task(0, &Calculate_Speeds, 50);
  register_task(1, &rangeScanUpdate, 30);
  register_task(2, &mavlink_heartbeat, 1000);
}

void loop() { 
 
      run_scheduler_update();	
//    comm_receive();
}


