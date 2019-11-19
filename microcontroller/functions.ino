/*
 * Seperate helper functions and static arm motor definitions
 * Calibration and setup of motors contained here
 */

void motorSetup(){
  
  motors[BASE].init(12, 10, 11, 48); //outputPin, errPin, dirPin, slaveSelect
  motors[BASE].controller(2.0, 1.0);  //Kp, Ki
  motors[BASE].sensor.init(); //inputpinA, inputpinB
  motors[BASE].sensor.calibrate(0, 200*16, 0, 360); //minInput, maxInput, minReal, maxReal
  motors[BASE].start(2000, 16); //milliamps, stepmode (for stepper only)

  motors[SHOULDER].init(49, 43, 40, 46);
  motors[SHOULDER].controller(3.0, 1.0);
  motors[SHOULDER].sensor.init();
  motors[SHOULDER].sensor.calibrate(0, 20000*8*10, 0, 10);
  //motors[BASE].start(2000, 8);

 /*
  motors[ELBOW].init(44, 32, 33, 45, 30, 31);
  motors[ELBOW].controller(0, 180, 1.0, 1.0);
  motors[ELBOW].sensor.init();
  motors[ELBOW].sensor.calibrate(0, 180);
  motors[ELBOW].start();
  
  motors[WRIST].init(2);
  motors[WRIST].controller(0, 180, 1.0, 1.0);
  motors[WRIST].sensor.init(A1);
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
