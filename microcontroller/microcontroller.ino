/*
 * Scrabble Robot
 * Matt Duke, Fall 2019
 * ENPH 454
 */

//#include <SoftwareSerial.h>
#include <SPI.h>
#include <AMIS30543.h>
#include "src/RobotArm/RobotArm.h"
#include "globals.h"


void setup() {
  errorFlag = SUCCESS;
 

  SPI.begin();
  Serial.begin(9600); //Debug & FTDI port
  while(!Serial){};
  SwSerial.begin(PORT_SPEED); //Control PC port
  Serial.println("Setup"); 
  pinSetup();
  
  motorSetup();
  autoCalibrate(); //Zero base motors
}

void loop() {
  return;
  
  for (int i=0;i < NUM_MOTORS;i++) {
    motors[i].update();
  }
  if (updateSuction) {
    suctionControl();
  }

  // Check suction buttton update flag (Set by ISR)
  if (updateSuction) {
    //suctionControl();
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
        motors[msgId].set(data.ui32);
        response.ui32 = data.ui32;
        sendMsg(SET, msgId, response);
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
            response.f32 = motors[msgId].sensor.read();
            break;
          }
        }
        sendMsg(GET, msgId, response);
        break;
      }
    }
  }
}
