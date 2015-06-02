#include <Arduino.h>
#include "Brain.h"
#include "constants_mega.h"
//#include "scheduler.h"

#define RUNTIME 3000

Brain brain;
void initDynamixel();
void initBluetooth();
bool bottle;
unsigned long bottleStartTime;
unsigned long time;
unsigned long time2;
unsigned long StartTime2;
int valueRPI;
char charRPI;
unsigned char ucharRPI;
char xRPIstr[4], yRPIstr[4];
char xRPIchar=-1, yRPIchar=-1;
int xRPI=0, yRPI=0;
long IRsum=0;
int IRval=0;
byte index=0;
int i;
int compteur =0;



void setup() {
	
	// SErial USB connection to RPI
	Serial.begin(9600);

  // Serial USB connection to computer
  //Serial.begin(115200);
  //Serial.println("Start!");
  //Serial.println("");

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
  
  bottleStartTime=millis();

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
  
  // if(bottleStartTime == -1) {
  //   if(analogRead(A7) > 180) {
  //     bottle = true;
  //     bottleStartTime = millis();
  //   }    
  //   Serial.print(bottle);
  //   Serial.print("  ");
  //   Serial.print(bottleStartTime);
  
	//~ for (i=0;i<50;i++){
	  //~ IRsum+=analogRead(A0);
	//~ }
	//~ IRval=(int)IRsum/50;
	//~ IRsum=0;
    //~ Serial.print("IR value:   ");
    //~ Serial.println(IRval);
    
  // } else if(millis() - bottleStartTime > 100) {
  //   bottleStartTime = -1;
  //   bottle = false;
  // }

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
  
  
  //---------TEST1 recieve from RPI ---------------------------
  if (Serial.available() > 0){
	  delay(5);
		  charRPI = Serial.read();
		  
		  if (charRPI=='x'){
				  while ((xRPIchar=Serial.read()) > 47) {
					  xRPIchar = xRPIchar - '0';
					  xRPIstr[index] = xRPIchar;
					  index++;
				  }
			  if (index==1)
				  xRPI=xRPIstr[0];
			  else if (index==2)
			      xRPI=xRPIstr[0]*10+xRPIstr[1];
			  else if (index==3)
			      xRPI=xRPIstr[0]*100+xRPIstr[1]*10+xRPIstr[2];
			  index=0;
		      //Serial.print("x =");
		      //Serial.println(xRPI);
		  }
		  
		  else if (charRPI=='y'){
				  while ((yRPIchar=Serial.read()) > 47) {
					  yRPIchar = yRPIchar -'0';
					  yRPIstr[index] = yRPIchar;
					  index++;
				  }
			  if (index==1)
				  yRPI=yRPIstr[0];
			  else if (index==2)
			      yRPI=yRPIstr[0]*10+yRPIstr[1];
			  else if (index==3)
			      yRPI=yRPIstr[0]*100+yRPIstr[1]*10+yRPIstr[2];
			  index=0;
		      //Serial.print("y =");
		      //Serial.println(yRPI);
		  }
		  
		  else{
			//Serial.println("wrong character");
		}
		charRPI=-1;
		Serial.flush();
   }
	  
//~ //---------TEST2 recieve from RPI ---------------------------
  //~ if (Serial.available() > 0) {
		//~ delay(5);
		//~ charRPI = Serial.read();
		//~ Serial.print(charRPI);
		//~ Bluetooth.send("In serial read");
		//~ Bluetooth.send(charRPI);
		//~ delay(500);
	//~ }
	
//~ //-------------TEST send to RPI------------------
	//~ time = millis()-bottleStartTime;
	//~ time2 = millis()-StartTime2;
	//~ if (time > 4000)	{
		//~ Serial.print("a");			//sends 's' on USB every 10s 
		//~ bottleStartTime+=time;
		//~ StartTime2+=time2;
//~ 
	//~ }
	//~ else if (time2 > 1500)	{
		//~ Serial.print("b");			//sends 's' on USB every 10s 
		//~ StartTime2+=time2;
	//~ }
		  
    brain.run();
	Bluetooth.send((int)xRPI);
	Bluetooth.send((float)yRPI);
	Bluetooth.send("in bottom loop");

//	Bluetooth.process();
  
  //~ if(Serial2.available()) {
    //~ Bluetooth.process();
  //~ }
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
