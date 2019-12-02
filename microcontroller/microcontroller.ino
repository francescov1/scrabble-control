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
  
  Serial.println("Setup"); 
  //test();
  pinSetup();

  motorSetup();
  //autoCalibrate(); //Zero base motors
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
    //Serial.println("Msg received: ");
    //Serial.print(msgType);
    //Serial.print(msgId);
    for (int i=0; i<sizeof(data.ui8); i++) {
      //Serial.println(buffer[i+2]);
      //Serial.println(sizeof(data.ui8)-i-1);
      data.ui8[sizeof(data.ui8)-i-1] = buffer[i+2];
    }
    /*
    for (int i=0; i<sizeof(data.ui8); i++) {
      Serial.println(data.ui8[i]);
    }*/

    DataUnion response;
    switch (msgType) {
      case HEARTBEAT:
        response.ui32 = 0;
        sendMsg(HEARTBEAT, errorFlag, response);
        break;
        
      case SET:
        Serial.println("SET");
        motors[msgId].set(data.ui32);
        Serial.println(data.ui32);
        response.ui32 = data.ui32;
        sendMsg(SET, msgId, response);
        break;
        
      case GET:
        Serial.println("GET");
        switch (data.ui32) {
          //STATUS
          case STATUS:
            response.ui16[0] = motors[msgId].latchedStatusFlags;
            response.ui16[1] = motors[msgId].nonLatchedStatusFlags;
            //Serial.println("status");
            break;
            
          //setpoint
          case SETPOINT:
            response.ui32 = (int32_t)(motors[msgId].setpoint);
            //Serial.println("setpoint");
            break;
          
          case POSITION:
            response.ui32 = (int32_t)(motors[msgId].sensor.read());
            //Serial.println("position");
            break;
        }
        sendMsg(GET, msgId, response);
        break;
    }
  }
}
