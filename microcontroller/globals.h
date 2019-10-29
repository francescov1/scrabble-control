/*
 * Global macro and variable defintions
 */

#define PORT_SPEED 115200
#define LED    13

//Definitions must match python
#define MSG_SIZE    6  //num of bytes

// RX/TX
#define HEARTBEAT   0
#define SET         1
#define GET         2
//Info
#define STATUS      0
#define ERROR       1
#define SUCCESS     2
#define ACK         3
#define SETPOINT    4
#define POSITION    5
// DEVICE
#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST       3
#define SUCTION     4

#define NUM_MOTORS  5

union DataUnion {
  uint8_t ui8[4];
  uint16_t ui16[2];
  int8_t si8[4];
  int16_t si16[2];
  float f32;
  int32_t;
  uint32_t ui32;
};

byte errorFlag;

Motor motors[NUM_MOTORS];
