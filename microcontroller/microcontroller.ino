#include <SPI.h>
#include <AMIS30543.h>
#include "src/RobotArm/RobotArm.h"

#define PORT_SPEED 115200
#define LED    13

//Definitions must match python
#define MSG_SIZE    6  //num of bytes

// RX/TX
#define HEARTBEAT   0
#define SET         1
#define STATUS      2
#define ERROR       3
#define SUCCESS     4
#define ACK         5
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
  uint32_t ui32;
};

byte errorFlag;
//for use when no data is intened to be sent
DataUnion empty;

Motor motors[NUM_MOTORS];

void setup() {
  errorFlag = SUCCESS;
  empty.ui32 = 0;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  SPI.begin();
  Serial.begin(PORT_SPEED);
  //inputPin, outputPin, dirPin, errPin, slaveSelect
  //inputPin, outputPin
  //motors[BASE].init(1,22,24,23,25);
  //motors[BASE].calibrate(CLOSED,
  //motors[BASE].start(10, 32);
  motors[SUCTION].init(7);
  motors[SUCTION].calibrate(0, 180);
  //DataUnion test;
  //test.ui16[0] = 0;
  //test.ui16[1] = 0;
  //Serial.println(test.ui16[0]);
  //Serial.println(test.ui16[1]);
  //while (true) {
  //}
}

void loop() {
  byte buffer[MSG_SIZE];
  if (checkMsgBuffer(buffer)) {
    byte msgType = buffer[0];
    byte msgId = buffer[1];
    DataUnion data;
    for (int i=0; i<sizeof(data.ui8); i++) {
      data.ui8[i] = buffer[i+2];
    }

    DataUnion response;
    switch (msgType) {
      case HEARTBEAT:
        sendMsg(HEARTBEAT, errorFlag, empty);
        break;
      case SET:
        motors[msgId].set(data.ui16[0]);
        response.ui32 = SUCCESS;
        sendMsg(ACK, msgId, response);
        break;
      case STATUS:
        response.ui16[0] = motors[msgId].latchedStatusFlags;
        response.ui16[1] = motors[msgId].nonLatchedStatusFlags;
        sendMsg(errorFlag, msgId, response);
        break;
    }
  }

  for (int i=0; i < NUM_MOTORS; i++) {
    motors[i].readErrors();
    if (!motors[i].disabled)
    {
      //needs to refresh much more often than minimum motor delay
      motors[i].update();
    }
  }
}

bool checkMsgBuffer(byte buffer[]) {
  if (Serial.available() == MSG_SIZE) {
    Serial.readBytes(buffer, MSG_SIZE);
    return true;
  }
  else {
    return false;
  }
}

void raiseRuntimeError() {
  errorFlag = false;
}

void sendMsg(byte msgType, byte msgId, DataUnion data) {
  byte msg[MSG_SIZE];
  msg[0] = msgType;
  msg[1] = msgId;
  for (int i=2; i<sizeof(data.ui8); i++) {
    msg[i] = data.ui8[i];
  }
  Serial.write(msg, MSG_SIZE);
}
