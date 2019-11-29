/*
 * Scrabble Robot
 * Matt Duke, Fall 2019
 * ENPH 454
 */

//#include <SoftwareSerial.h>
#include <SPI.h>
#include <AMIS30543.h>
#include <SoftwareSerial.h>
#include "src/RobotArm/RobotArm.h"
#include "globals.h"


void setup() {
  errorFlag = SUCCESS;

  SPI.begin();
  Serial.begin(9600); //Debug & FTDI port
  Serial1.begin(PORT_SPEED); //Control PC port

  while(!Serial || !Serial1){}
  
  //test();
  //pinSetup();
  
  //motorSetup();
  //autoCalibrate(); //Zero base motors
  Serial.println("Done setup");
}

void loop() {
    
  for (int i=0;i < NUM_MOTORS;i++) {
    motors[i].update();
  }

  // Check suction buttton update flag (Set by ISR)
  if (updateSuction) {
    suctionControl();
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
        motors[msgId].set(data.i32);
        response.i32 = data.i32;
        sendMsg(SET, msgId, response);
        Serial.println("Setting motor");
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
            response.i32 = motors[msgId].setpoint;
            Serial.println(response.i32);
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
