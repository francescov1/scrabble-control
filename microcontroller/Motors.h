#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

#https://www.arduino.cc/en/Hacking/LibraryTutorial

class Servo
{
  public:
    Servo(uint8_t inputPin);
    void set();
  private:
    uint8_t _setPoint;
	uint8_t _inputPin;
};

struct Stepper {
  uint8_t inputPin;
  uint8_t setPoint;
  uint8_t dirPin;
  uint8_t stepPin;
  uint8_t slaveSelect;
};

#endif