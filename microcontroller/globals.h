/*
 * Global macro and variable defintions
 */

/* Shield pins
  STEPPER 1:
    STEP: 12
    DIR: 11
    DO: 50
    DI: 51
    CS: 48
    ERR: 10

  STEPPER 2:
    STEP: 49
    DIR: 40
    DO: 50
    DI: 51
    CS: 46
    ERR: 43

  STEPPER 3:
    PWM: 44
    IN_1: 32
    IN_2: 33
    PWM: 45
    IN_1: 30
    IN_2: 31

  SERVOS:
    PWM: 2, 3

  BUTTONS (DIGITAL):
    PINS: 18, 19

  POTENTIOMETERS:
    PINS: A1, A2
*/

#define PORT_SPEED 115200
#define LED        13
#define BUTTON     10
#define LIMIT      11

#define PWM_A      12
#define AIN_1      13
#define AIN_2      14

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
  int8_t i8[4];
  int16_t i16[2];
  float f32;
  int32_t i32;
  uint32_t ui32;
};

byte errorFlag;

Motor motors[NUM_MOTORS];
bool suctionState = false;
bool updateSuction = false;
