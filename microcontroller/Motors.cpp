#include "Arduino.h"
#include "Motors.h"

#include "SPI.h"
#include "AMIS30543.h"
#include "PID_v1.h"

Servo::Servo(uint8_t inputPin)
{
	pinMode(inputPin, INPUT);
	_inputPin = inputPin;
}

Stepper::Stepper(uint8_t inputPin, uint8_t dirPin, uint8_t stepPin, )
{
	pinMode(inputPin, )
	_inputPin = inputPin;
}