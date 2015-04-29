
#define LmotorA             3  // Left  motor H bridge, input A
#define LmotorB            11  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

#define RCleft              0  // Digital input 0
#define RCright             1  // Digital input 1

#define encLa               2  // Encoder Left first quadrature
#define encLb               4  // Encoder Left second quadrature
#define encRa               10 // Encoder Right first quadrature
#define encRb               12 // Encoder Right second quadrature

#define S5                  7  // Servo output 05
#define S6                  8  // Servo output 06
#define S4                  9  // Servo output 04


#define Battery             0  // Analog input 00
#define RmotorC             6  // Analog input 06
#define LmotorC             7  // Analog input 07
#define Charger            13  // Low=ON High=OFF


//------------------------- Define function of each I/O pin -------------------------------------------------

#define lmencpin     6  //  D6 - left  motor encoder input - optional
#define rmencpin     5  //  D5 - right motor encoder input - optional

#define lmbrkpin     4  //  D4 - left  motor brake        control    pin    HIGH = Brake 
#define lmdirpin     2  //  D2 - left  motor direction    control    pin    HIGH = Forward   Low = Reverse
#define lmpwmpin     3  //  D3 - left  motor pulse width  modulation pin    0 - 255          Speed and Brake 
#define lmcurpin     6  //  A6 - left  motor current      monitor    pin    0 - 1023         -20A to +20A   

#define rmbrkpin     9  //  D9 - right motor brake        control    pin    HIGH = Brake 
#define rmdirpin    10  // D10 - right motor direction    control    pin    HIGH = Forward   Low = Reverse
#define rmpwmpin    11  // D11 - right motor pulse width  modulation pin    0 - 255          Speed and Brake 
#define rmcurpin     7  //  A7 - right motor current      monitor    pin    0 - 1023         -20A to +20A   
