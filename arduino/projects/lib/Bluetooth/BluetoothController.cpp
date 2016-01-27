#define VERSION     "\n\nAndroTest V2.0 - @kas2014\ndemo for V5.x App"

#include "BluetoothController.h"

void BluetoothController::process() {
  if(Serial2.available())  {                            // data received from smartphone
    delay(2);
    cmd[0] = Serial2.read();
    if(cmd[0] == STX)  {
      int i=1;      
      while(Serial2.available())  {
        delay(1);
        cmd[i] = Serial2.read();
        if(cmd[i]>127 || i>7)                 break;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;
      }
      if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
  sendBlueToothData();
  while(Serial2.available())  Serial2.read();         // empty RX buffer
}

void BluetoothController::sendBlueToothData()  {
  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
    previousMillis = currentMillis; 

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    Serial2.print((char)STX);                                             // Start of Transmission
    Serial2.print(getButtonStatusString());  Serial2.print((char)0x1);    // buttons status feedback
    Serial2.print(getDataInt());             Serial2.print((char)0x4);    // datafield #1
    Serial2.print(getDataFloat());           Serial2.print((char)0x5);    // datafield #2
    Serial2.print(getDataString());                                       // datafield #3
    Serial2.print((char)ETX);                                             // End of Transmission
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

int BluetoothController::getSpeed() {
  return _joySpeed;
}

int BluetoothController::getSteer() {
  return _joySteer;
}

void BluetoothController::getJoystickState(byte data[8])    {
  _joySteer = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  _joySpeed = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  _joySteer = _joySteer - 200;                                                  // Offset to avoid
  _joySpeed = _joySpeed - 200;                                                  // transmitting negative numbers

  if(_joySteer<-100 || _joySteer>100 || _joySpeed<-100 || _joySpeed>100)     return;      // commmunication error


  // Your code here ...
  // Serial.print("Joystick position:  ");
  // Serial.print(_joySteer);  
  // Serial.print(", ");  
  // Serial.println(_joySpeed); 
}

bool BluetoothController::buttonIsOn(int button) {
  return _buttonStatus[button-1];
}

void BluetoothController::getButtonState(int bStatus)  {
  switch (bStatus) {
    // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      // Serial.println("\n** Button_1: ON **");
      _buttonStatus[0] = true;
      // Serial.println("LED <ON>");
      digitalWrite(ledPin, HIGH);
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      // Serial.println("\n** Button_1: OFF **");
      _buttonStatus[0] = false;
      // Serial.println("LED <OFF>");
      digitalWrite(ledPin, LOW);
      break;

      // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      // Serial.println("\n** Button_2: ON **");
      _buttonStatus[1] = true;
      // Serial.println("Autonomous Mode <ON>");
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      // Serial.println("\n** Button_2: OFF **");
      _buttonStatus[1] = false;
      // Serial.println("Autonomous Mode <OFF>");
      break;

      // -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      // Serial.println("\n** Button_3: ON **");
      _buttonStatus[2] = true;
      // Serial.println("Motors enabled");
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      // Serial.println("\n** Button_3: OFF **");
      _buttonStatus[2] = false;
      // Serial.println("Motors stopped");
      break;

      // -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      // Serial.println("\n** Button_4: ON **");
      _buttonStatus[3] = true;
      //Serial.println("Datafield update <FAST>");
      //sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      // Serial.println("\n** Button_4: OFF **");
      _buttonStatus[3] = false;
      //Serial.println("Datafield update <SLOW>");
      //sendInterval = SLOW;
      break;

      // -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
      buttonStatus |= B010000;        // ON
      // Serial.println("\n** Button_5: ++ pushed ++ **");
      _buttonStatus[4] = true;
      break;
    case 'J':
      buttonStatus &= B101111;        // OFF
      _buttonStatus[4] = false;
      break;
      // -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      // Serial.println("\n** Button_6: ON **");
      _buttonStatus[5] = true;
      break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      // Serial.println("\n** Button_6: OFF **");
      _buttonStatus[5] = false;
      break;
  }
  // ---------------------------------------------------------------
}

BluetoothController Bluetooth;