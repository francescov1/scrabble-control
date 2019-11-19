/*
 * Scrabble Robot
 * Matt Duke, Fall 2019
 * ENPH 454
 */

//#include <SoftwareSerial.h>
#include "src/RobotArm/RobotArm.h"
#include "globals.h"

//SoftwareSerial SwSerial(4, 5);

void setup() {
  errorFlag = SUCCESS;

  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);

  SPI.begin();
  Serial.begin(9600); //Debug & FTDI port
  while(!Serial){};
  //SwSerial.begin(PORT_SPEED); //Control PC port
  Serial.println("Setup");
  motorSetup();
  motors[BASE].set(70);
  //attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, RISING);

  //autoCalibrate(); //Zero base motor
}

void loop() {
  /*
  for (int i=0; i < NUM_MOTORS; i++) {
    motors[i].read_errors();
    if (!motors[i].disabled)
    {
      motors[i].update();
    }
  }*/
  //motors[BASE].read_errors();
  motors[BASE].update();
  //Serial.println(motors[BASE].sensor.read());
  //motors[BASE].read_errors();
  //motors[BASE].update();
  return;

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
