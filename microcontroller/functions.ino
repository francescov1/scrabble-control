/*
 * Seperate helper functions and static arm motor definitions
 * Calibration and setup of motors contained here
 */

void test() {
  while(true) {
    Serial.println(analogRead(POT_A));
  }
}

void pinSetup() {  
  pinMode(LED, OUTPUT);
  
  pinMode(LIMIT_A, INPUT_PULLUP);
  pinMode(LIMIT_B, INPUT_PULLUP);
  pinMode(LIMIT_C, INPUT_PULLUP);

  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH);

  //attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, RISING);
}

void motorSetup() {

  //49, 46, 43, 48, A6
  //12, 17, 10, 11, A7
  //22, 21, 23, 0, A5
  
  motors[BASE].init(12, 17, 10, 11, A7);
  motors[BASE].controller(0.5, 1.0);
  motors[BASE].sensor.init();
  motors[BASE].sensor.calibrate(0, 20000, 0, 5000);
  motors[BASE].calibrate(1000, 2000);
  motors[BASE].start(2, 1800);
  Serial.println("BASE started");

  motors[ELBOW].init(49, 46, 43, 48, A6); //uint8_t outputPin, uint8_t slaveSelect=0, uint8_t errPin=0, uint8_t dirPin=0, uint8_t slaPin=0
  motors[ELBOW].controller(0.5, 1.0);  //Kp, Ki
  motors[ELBOW].sensor.init(POT_A); //inputpinA, inputpinB
  motors[ELBOW].sensor.calibrate(53, 550, -70, 80); //minInput, maxInput, minReal, maxReal
  motors[ELBOW].calibrate(6000, 9000);
  motors[ELBOW].start(16, 1800); //stepmode, milliamps
  Serial.println("ELBOW started");

  motors[SHOULDER].init(22, 21, 23, 0, A5); //uint8_t outputPin, uint8_t slaveSelect=0, uint8_t errPin=0, uint8_t dirPin=0, uint8_t slaPin=0
  motors[SHOULDER].controller(0.5, 1.0);  //Kp, Ki
  motors[SHOULDER].sensor.init(); //inputpinA, inputpinB
  motors[SHOULDER].sensor.calibrate(0, 5000, 0, 90); //minInput, maxInput, minReal, maxReal
  motors[SHOULDER].calibrate(2000, 5000);
  motors[SHOULDER].start(4, 1800); //stepmode, milliamps
  Serial.println("SHOULDER started");
  
 /*
  
  motors[WRIST].init(3);
  motors[WRIST].controller(0.5, 1.0);
  motors[WRIST].calibrate(750, 2300);
  motors[WRIST].sensor.init(A2);
  motors[WRIST].sensor.calibrate(230, 830, 0, 180);
  motors[WRIST].start();
  */
}

void autoCalibrate() {
  motors[SHOULDER].set(10);
  while(!motors[SHOULDER].update()){};
  delay(1000);
  
  motors[ELBOW].set(-20);
  while(!motors[ELBOW].update()){};
  delay(1000);

  int i=0;
  bool limitStateB = digitalRead(LIMIT_B);
  while (digitalRead(LIMIT_B) == limitStateB) {
    motors[SHOULDER].read_errors();
    motors[SHOULDER].step(false);
    if (i % 4 == 0) {
       motors[ELBOW].step(true);
    }
    delayMicroseconds(300);
    i++;
  }
  motors[SHOULDER].sensor.zero();

  motors[SHOULDER].set(90);
  motors[ELBOW].set(-20);
  while(!motors[SHOULDER].update() && !motors[ELBOW].update()){}
  delay(1000);
  
  motors[BASE].set(1000);
  while (!motors[BASE].update()) {}
  bool limitStateA = digitalRead(LIMIT_A);
  while (digitalRead(LIMIT_A) == limitStateA) {
    motors[BASE].read_errors();
    motors[BASE].step(false);
    delayMicroseconds(300);
  }
  motors[BASE].sensor.zero();
  motors[BASE].set(1000);
  while (!motors[BASE].update()) {}

  delay(1000);  
}

static void buttonISR() {
  if (!updateSuction) {
    suctionState = !suctionState;
    updateSuction = true;
  }
}

void suctionControl() {
  if (suctionState) {
    digitalWrite(AIN_1, LOW);
    digitalWrite(AIN_2, HIGH);
    digitalWrite(PWM_A, HIGH);
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
  if (Serial1.available() == MSG_SIZE) {
    Serial1.readBytes(buffer, MSG_SIZE);
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
  Serial1.write(msg, MSG_SIZE);
}
