/*
 * Scrabble Robot
 * Matt Duke, Fall 2019
 * ENPH 454
 */

#include <SPI.h>
#include <AMIS30543.h>
#include "src/RobotArm/RobotArm.h"
#include "globals.h"

void setup() {
  errorFlag = SUCCESS;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  SPI.begin();
  Serial.begin(PORT_SPEED);
}

void loop() {
  for (int i=0; i < NUM_MOTORS; i++) {
    motors[i].read_errors();
    if (!motors[i].disabled)
    {
      motors[i].update();
    }
  }
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
      case HEARTBEAT: {
        response.ui32 = 0;
        sendMsg(HEARTBEAT, errorFlag, response);
        break;
      }
      case SET: {
        motors[msgId].set(data.ui16[0]);
        response.ui32 = 0;
        sendMsg(ACK, msgId, response);
        break;
      }
      case GET: {
        switch (data.ui32) {
          //STATUS
          case STATUS: {
            response.ui16[0] = motors[msgId].latchedStatusFlags;
            response.ui16[1] = motors[msgId].nonLatchedStatusFlags;
            break;
          }
          //setpoint
          case SETPOINT: {
            response.ui32 = motors[msgId].setpoint;
            break;
          }
          case POSITION: {
            response.f32 = motors[msgId].position();
            break;
          }
        }
        sendMsg(ACK, msgId, response);
        break;
      }
    }
  }
}
