/*
 * Seperate helper functions and static arm motor definitions
 * Calibration and setup of motors contained here
 */

void motorSetup(){
  motors[BASE].init(22,23,25); //outputPin, errPin, slaveSelect
  motors[BASE].start(132, 32); //milliamps, stepmode (for stepper only)
  motors[BASE].set(90);

  /*
  motors[BASE].init(22,23,25); //outputPin, errPin, slaveSelect
  motors[BASE].controller(0, 1000, 1.0, 1.0);  //min, max (chosen units), Ki, Kp
  motors[BASE].sensor.init(2,3); //inputpinA, inputpinB
  motors[BASE].start(132, 32); //milliamps, stepmode (for stepper only)

  motors[SHOULDER].init(10, 11, 12);
  motors[SHOULDER].controller(0, 180, 1.0, 1.0);
  motors[SHOULDER].sensor.init(A1);
  motors[SHOULDER].calibrate(0, 1023); //minSetpoint, maxSetpoint
  motors[SHOULDER].start(132, 32);

  motors[ELBOW].init(13, 14, 15);
  motors[ELBOW].controller(0, 180, 1.0, 1.0);
  motors[ELBOW].sensor.init(A2);
  motors[ELBOW].calibrate(0, 1023);
  motors[ELBOW].start();

  motors[WRIST].init(16, 17, 18);
  motors[WRIST].controller(0, 180, 1.0, 1.0);
  motors[WRIST].sensor.init(A3);
  motors[WRIST].calibrate(0, 1023);
  motors[WRIST].start();

  motors[SUCTION].init(19, 20, 30);
  motors[SUCTION].controller(0, 180, 1.0, 1.0);
  motors[SUCTION].sensor.init(A4);
  motors[SUCTION].sensor.calibrate(0, 1023);
  motors[SUCTION].start();*/
}

static void buttonISR() {
  suctionState = !suctionState;
  if (suctionState) {
    motors[SUCTION].set(180);
  }
  else {
    motors[SUCTION].set(0);
  }
  delayMicroseconds(1000);
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
  for (int i=0; i<sizeof(data.ui8); i++) {
    msg[i+2] = data.ui8[i];
  }
  Serial.write(msg, MSG_SIZE);
}
