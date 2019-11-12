/*
 * Seperate helper functions and static arm motor definitions
 * Calibration and setup of motors contained here
 */

void motorSetup(){
  motors[BASE].init(9,23,25); //outputPin, errPin, slaveSelect
  motors[BASE].controller(0, 1000, 1.0, 1.0);  //min, max (chosen units), Ki, Kp
  motors[BASE].sensor.init(2,3); //inputpinA, inputpinB
  motors[BASE].sensor.calibrate(0, 100); //minSetpoint, maxSetpoint
  motors[BASE].start(132, 32); //milliamps, stepmode (for stepper only)

  while(true){
  }

  /*
  motors[SHOULDER].init(10, 11, 12);
  motors[SHOULDER].controller(0, 180, 1.0, 1.0);
  motors[SHOULDER].sensor.init(A1);
  motors[SHOULDER].sensor.calibrate(0, 1023);
  motors[SHOULDER].start(132, 32);

  motors[ELBOW].init(13, 14, 15);
  motors[ELBOW].controller(0, 180, 1.0, 1.0);
  motors[ELBOW].sensor.init(A2);
  motors[ELBOW].sensor.calibrate(0, 1023);
  motors[ELBOW].start();

  motors[WRIST].init(16, 17, 18);
  motors[WRIST].controller(0, 180, 1.0, 1.0);
  motors[WRIST].sensor.init(A3);
  motors[WRIST].sensor.calibrate(0, 1023);
  motors[WRIST].start();
  */
}

void autoCalibrate() {
  motors[BASE].set(-1);
  while (digitalRead(LIMIT) == LOW) {
    motors[BASE].sensor.zero();
    motors[BASE].update();
  }
  motors[BASE]._driver.setStepMode(128);
  while (digitalRead(LIMIT) == HIGH) {
    motors[BASE].set(motors[BASE].setpoint+1);
    motors[BASE].update();
  }
  motors[BASE].sensor.zero();
  motors[BASE]._driver.setStepMode(32);
}

static void buttonISR() {
  if (!updateSuction) {
    suctionState = !suctionState;
    updateSuction = true;
  }
}

void suctionControl() {
  if (suctionState) {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, LOW);
    analogWrite(PWM_A, 100);
  }
  else {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, LOW);
    analogWrite(PWM_A, 50);
    delay(2000);
    digitalWrite(AIN_1, LOW);
    analogWrite(PWM_A, 0);
  }
  updateSuction = false;
}

bool checkMsgBuffer(byte buffer[]) {
  if (SwSerial.available() == MSG_SIZE) {
    SwSerial.readBytes(buffer, MSG_SIZE);
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
  SwSerial.write(msg, MSG_SIZE);
}
