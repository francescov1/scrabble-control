/*
 * Global macro and variable defintions
 */

/* Shield pins
  STEPPER 1:
    STEP: 12
    DIR: 11
    DO: 50
    DI: 51
    CLK:
    CS: 17
    ERR: 10
    SLA: A7

  STEPPER 2:
    STEP: 49
    DIR: 48
    DO: 50
    DI: 51
    CLK:
    CS: 46
    ERR: 43
    SLA: A6

  STEPPER 3:
    STEP: 22
    DIR: N/A
    DO: 50
    DI: 51
    CLK:
    CS: 21
    ERR: 23
    SLA: A5

  SERVOS:
    PWM: 2, 3

  DC MOTORS:
    STBY: 27
    A1, A2: 30, 31
    B1, B2: 32, 33
    PWMA, PWMB: 44, 45 

  BUTTONS (DIGITAL):
    PINS (w/ 5V): 18, 19
    PINS: 14, 15, 16
    
  POTENTIOMETERS:
    PINS: A1, A2

  3.3V OUTPUT:
    PINS: 4, 5, 6, 7
*/

#define PORT_SPEED 115200
#define LED        13
#define LIMIT_A    14
#define LIMIT_B    15
#define LIMIT_C    16
#define BUTTON     A13
#define POT_A      A2
#define POT_B      A1

#define AIN_1      30
#define AIN_2      31
#define BIN_1      32
#define BIN_2      33
#define STBY       27
#define PWM_A      45
#define PWM_B      44

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
  uint32_t ui32;
};

byte errorFlag;

Motor motors[NUM_MOTORS];
bool suctionState = false;
bool updateSuction = false;
bool buttonState = false;
