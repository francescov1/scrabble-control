/*
 * Seperate helper functions and static arm motor definitions
 * Calibration and setup of motors contained here
 */

void pinSetup() {
  pinMode(LED, OUTPUT);
  
  pinMode(LIMIT_A, INPUT_PULLUP);
  pinMode(LIMIT_B, INPUT_PULLUP);
  pinMode(LIMIT_C, INPUT_PULLUP);
}

void motorSetup() {

  //49, 46, 43, 48, A6
  //12, 17, 10, 11, A7
  
  motors[BASE].init(12, 17, 10, 11, A7);
  motors[BASE].controller(0.5, 1.0);
  motors[BASE].sensor.init();
  motors[BASE].sensor.calibrate(0, 5000, 0, 90);
  motors[BASE].calibrate(200, 100);
  motors[BASE].start(4, 2000);
  Serial.println("BASE started");

  motors[ELBOW].init(49, 46, 43, 48, A6); //uint8_t outputPin, uint8_t slaveSelect=0, uint8_t errPin=0, uint8_t dirPin=0, uint8_t slaPin=0
  motors[ELBOW].controller(0.5, 1.0);  //Kp, Ki
  motors[ELBOW].sensor.init(); //inputpinA, inputpinB
  motors[ELBOW].sensor.calibrate(0, 100, 0, 90); //minInput, maxInput, minReal, maxReal
  motors[ELBOW].calibrate(2000, 5000);
  motors[ELBOW].start(16, 1500); //stepmode, milliamps
  Serial.println("ELBOW started");
  
 /*
  motors[SHOULDER].init();
  motors[SHOULDER].controller(1.0, 1.0);
  motors[SHOULDER].sensor.init();
  motors[SHOULDER].sensor.calibrate(0, 200*8*2, 0, 180);
  motors[SHOULDER].start(8, 3000);
  
  motors[WRIST].init(3);
  motors[WRIST].controller(0.5, 1.0);
  motors[WRIST].calibrate(750, 2300);
  motors[WRIST].sensor.init(A2);
  motors[WRIST].sensor.calibrate(230, 830, 0, 180);
  motors[WRIST].start();
  */
}

void autoCalibrate() {
  motors[BASE].set(-1);
  while (digitalRead(LIMIT_A) == LOW) {
    motors[BASE].sensor.zero();
    motors[BASE].update();
  }
  motors[BASE]._driver.setStepMode(128);
  while (digitalRead(LIMIT_A) == HIGH) {
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

/*
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
*/

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
