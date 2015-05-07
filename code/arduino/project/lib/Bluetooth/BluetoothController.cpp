#define VERSION     "\n\nAndroTest V2.0 - @kas2014\ndemo for V5.x App"

#include "BluetoothController.h"

void BluetoothController::processBluetooth() {
  if(Serial1.available())  {                            // data received from smartphone
    delay(2);
    cmd[0] = Serial1.read();
    Serial.println(cmd[0]);
    if(cmd[0] == STX)  {
      int i=1;      
      while(Serial1.available())  {
        delay(1);
        cmd[i] = Serial1.read();
        Serial.println(cmd[i]);
        if(cmd[i]>127 || i>7)                 break;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;
      }
      if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
  Serial.println("");
  sendBlueToothData();
  while(Serial1.available())  Serial1.read();         // empty RX buffer
}

void BluetoothController::sendBlueToothData()  {
  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
    previousMillis = currentMillis; 

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    Serial1.print((char)STX);                                             // Start of Transmission
    Serial1.print(getButtonStatusString());  Serial1.print((char)0x1);    // buttons status feedback
    Serial1.print(getDataInt());             Serial1.print((char)0x4);    // datafield #1
    Serial1.print(getDataFloat());           Serial1.print((char)0x5);    // datafield #2
    Serial1.print(getDataString());                                       // datafield #3
    Serial1.print((char)ETX);                                             // End of Transmission
  }  
}

String BluetoothController::getButtonStatusString()  {
  String bStatus = "";
  for(int i=0; i<6; i++)  {
    if(buttonStatus & (B100000 >>i))      bStatus += "1";
    else                                  bStatus += "0";
  }
  return bStatus;
}


void BluetoothController::send(int sendInt) {
  _sendInt = sendInt;
}

void BluetoothController::send(float sendFloat) {
  _sendFloat = sendFloat;
}

void BluetoothController::send(String sendString) {
  _sendString = sendString;
}

int BluetoothController::getDataInt()  {              // Data dummy values sent to Android device for demo purpose
  return _sendInt;
}

float BluetoothController::getDataFloat()  {           // Data dummy values sent to Android device for demo purpose
  return _sendFloat;
}

String BluetoothController::getDataString() {
  return _sendString;
}

void BluetoothController::getJoystickState(byte data[8])    {
  int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error

  // Your code here ...
  Serial.print("Joystick position:  ");
  Serial.print(joyX);  
  Serial.print(", ");  
  Serial.println(joyY); 
}

bool BluetoothController::getButtonState(int bStatus)  {
  Serial.print("Reading buttons: ");
  Serial.println(bStatus);
  bool status;
  switch (bStatus) {
    // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      Serial.println("\n** Button_1: ON **");
      status = true;
      Serial.println("LED <ON>");
      digitalWrite(ledPin, HIGH);
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      Serial.println("\n** Button_1: OFF **");
      status = false;
      Serial.println("LED <OFF>");
      digitalWrite(ledPin, LOW);
      break;

      // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      Serial.println("\n** Button_2: ON **");
      status = true;
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      Serial.println("\n** Button_2: OFF **");
      status = false;
      break;

      // -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      Serial.println("\n** Button_3: ON **");
      status = true;
      Serial.println("Motor #1 enabled");
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      Serial.println("\n** Button_3: OFF **");
      status = false;
      Serial.println("Motor #1 stopped");
      break;

      // -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      Serial.println("\n** Button_4: ON **");
      status = true;
      Serial.println("Datafield update <FAST>");
      sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      Serial.println("\n** Button_4: OFF **");
      status = false;
      Serial.println("Datafield update <SLOW>");
      sendInterval = SLOW;
      break;

      // -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
      //      buttonStatus |= B010000;        // ON
      Serial.println("\n** Button_5: ++ pushed ++ **");
      status = false;
      break;
      //   case 'J':
      //     buttonStatus &= B101111;        // OFF
      //     // your code...      
      //     break;
      status = false;
      // -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      Serial.println("\n** Button_6: ON **");
      status = true;
      break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      Serial.println("\n** Button_6: OFF **");
      status = false;
      break;
  }
  return status;
  // ---------------------------------------------------------------
}

BluetoothController Bluetooth;