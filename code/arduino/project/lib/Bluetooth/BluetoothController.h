#ifndef _BLUETOOTH_CONTROLLER_
#define _BLUETOOTH_CONTROLLER_

// V2.0  changed to pure ASCII Communication Protocol ** not backward compatible **
// V1.4  improved communication errors handling
// V1.3  renamed for publishing, posted on 09/05/2014
// V1.2  Text display   ** not backward compatible **
// V1.1  Integer display
// V1.0  6 buttons + 4 data char implemented

// Demo setup:
// Button #1 controls pin #13 LED
// Button #4 toggle datafield display rate
// Button #5 configured as "push" button (momentary)
// Other buttons display demo message

// Arduino UART1 RX to TX BlueTooth module
// Arduino UART1 TX to RX BlueTooth module
// make sure your BT board is set @115200 bps (AT commands)

#include "Arduino.h"

#define    STX          0x02
#define    ETX          0x03
#define    ledPin       13
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                            // Datafields refresh rate (ms)  

class BluetoothController {
  public:
    void process();
    void send(int);
    void send(float);
    void send(String);
    void getButtonState(int);
    int getSpeed();
    int getSteer(); 
    bool buttonIsOn(int button);

  private:
    int _sendInt;
    float _sendFloat;
    String _sendString;
    byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};             // bytes received
    byte buttonStatus = 0;                              // first Byte sent to Android device
    long previousMillis = 0;                            // will store last time Buttons status was updated
    long sendInterval = SLOW;                           // interval between Buttons status transmission (milliseconds)
    int _joySteer;
    int _joySpeed;
    bool _buttonStatus[6] = {false, false, false, false, false, false};

    String getButtonStatusString();
    void sendBlueToothData();
    int getDataInt();
    float getDataFloat();
    String getDataString();
    void getJoystickState(byte[8]);
};

extern BluetoothController Bluetooth;

#endif
