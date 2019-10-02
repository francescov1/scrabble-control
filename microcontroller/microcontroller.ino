#define PORT_SPEED 115200
#define MSG_SIZE 3


//Definitions must match python

// RX
#define HEARTBEAT 0
#define SET 1
#define GET 2
// MOTOR
#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define WRIST 3
#define SUCTION 4

#define SUCCESS 1
#define FAILURE 0

struct Servo {
  int inputPin;
  int analogPin;
  int setPoint;
};

struct Stepper {
  int inputPin;
  int setPoint;
};

Servo servos[4];

Stepper steppers[1];

void setup() {
  Serial.begin(PORT_SPEED);

  servos[SHOULDER].inputPin = 1;
  servos[SHOULDER].analogPin = 1;
  servos[SHOULDER].inputPin = 2;
  servos[SHOULDER].analogPin = 2;
  servos[BASE].inputPin = 2;
}

void loop() {
  while (Serial.available() < MSG_SIZE) {
  }
  byte data[MSG_SIZE];
  Serial.readBytes(data, MSG_SIZE);
  switch (data[0]) {
    case HEARTBEAT:
      heartbeat(data);
      break;
    case SET:
      switch (data[1]) {
        case BASE:
          setStepper(steppers[data[1]]);
          break;
        default:
          setServo(servos[data[1]]);
          break;
      }
      break;
  }  
}

void heartbeat(byte data[MSG_SIZE]) {
  Serial.write(data, MSG_SIZE);
}

void setServo(Servo servo) {
  Serial.write(SUCCESS);
}

void setStepper(Stepper stepper) {
  Serial.write(SUCCESS);
}
