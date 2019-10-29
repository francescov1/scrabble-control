#include "Arduino.h"
#include "RobotArm.h"

#define MIN_STEPPER_DELAY  200
#define MAX_STEPPER_DELAY  2000
#define MIN_SERVO_DELAY    550
#define MAX_SERVO_DELAY    2450
#define SERVO_PERIOD       20000


Sensor* Sensor::sSensor = 0;

Sensor::Sensor(){
	this->_value = 0;
	this->minInput = 0;   // default
	this->maxInput = 100; // default
}

void Sensor::zero() {
	if (_type == DIGITAL) {
		this->_value = 0;
	}
}

void Sensor::calibrate(uint16_t minInput, uint16_t maxInput) {
	this->minInput = minInput;
	this->maxInput = maxInput;
}

void Sensor::init(uint8_t inputPinA, uint8_t inputPinB=0) {
	_inputPinA = inputPinA;
	pinMode(_inputPinA, INPUT);
	_inputPinB = inputPinB;
	if (_inputPinB == 0) {
		_type = ANALOG;
	} else {
		_type = DIGITAL;
		pinMode(_inputPinB, INPUT);
	}

	switch (_type) {
		case DIGITAL: {
			sSensor = this;
			attachInterrupt(digitalPinToInterrupt(_inputPinA), Sensor::updateSensorISR, CHANGE);
			attachInterrupt(digitalPinToInterrupt(_inputPinB), Sensor::updateSensorISR, CHANGE);
			break;
		}
		case ANALOG:
		break;
	}
}

void Sensor::updateSensorISR() {
  if (sSensor != 0)
      sSensor->update();
}

void Sensor::update() {
	switch(_type) {
		case ANALOG: {
			uint8_t iterations = 3;
			float average = 0;
			for (int i=0; i<iterations; i++) {
				average += analogRead(_inputPinA);
				delay(1);
			}
			average = average / iterations;
			this->_value = average;
			break;
		}
		case DIGITAL: {
			bool A_state = digitalRead(_inputPinA);
			bool B_state = digitalRead(_inputPinB);
			if (A_state == B_state) {
				this->_value++;
			}
			else {
				this->_value--;
			}
			break;
		}
	}
}

int32_t Sensor::read() {
	if (_type==ANALOG) {
		update();
	}
	return this->_value;
}

Motor::Motor()
{
	nonLatchedStatusFlags = 0;
	latchedStatusFlags = 0;
	disabled = true;
	_mode = OPEN;
	_type = 0;
}

//STEPPER
void Motor::init(uint8_t outputPin, uint8_t errPin=0, uint8_t slaveSelect=0)
{
	if (!errPin || !slaveSelect) {
		_type = SERVO;
		_outputPin = outputPin;
		pinMode(_outputPin, OUTPUT);
	}
	else {
		_type = STEPPER;
		_errPin = errPin;
		_driver.init(_slaveSelect);
	}
}

void Motor::controller(uint16_t minOutput, uint16_t maxOutput, float Ki, float Kp) {
	_maxOutput = minOutput;
	_maxOutput = minOutput;
	_Ki = Ki;
	_Kp = Kp;
	_mode = CLOSED;
}

//STEPPER
void Motor::start(uint16_t milliamps=0, uint16_t stepmode=0) {
	if (_mode!=OPEN && !sensor.initialized) {
		return; //not ready to start, sensor not initialized
	}
	switch(_type){
		case STEPPER: {
			digitalWrite(_outputPin, LOW);
			delay(1);
			_driver.resetSettings();
			_driver.setCurrentMilliamps(milliamps);
			_driver.setStepMode(stepmode);
			_driver.enableDriver();
			delay(1);
		}
		case SERVO: {
			digitalWrite(_outputPin, LOW);
			disabled = false;
		}
	}
}

void Motor::set(uint16_t value) {
	if (_minOutput >= _maxOutput) {
		//ignore min/max (likely both defaulted to 0)
		setpoint = value;
	}
	else {
		setpoint = constrain(value, _minOutput, _maxOutput);
	}
}

int32_t Motor::position() {
	int32_t position = map(sensor.read(), sensor.minInput, sensor.maxInput, _maxOutput, _maxOutput);
	return position;
}

bool Motor::update()
{
	if (_mode == OPEN) {
		step(setpoint);
		return false;
	}
	else {
		// PI control implemented here

		//both position() and setpoint are in human readable format (ie. degrees or centimeters)
		int16_t error = position() - setpoint;
		// reset and return if setpoint acheived
		if (error == 0) {
			_accumulatedError = 0;
			return true;
		}
		uint32_t time = millis();
		_accumulatedError += error * (time - _lastStepTime - time);
		int16_t target = _Kp * error + _Ki * _accumulatedError;
		//step(target);
		return false;
	}
}

void Motor::read_errors()
{
	if (_type != STEPPER){
		//errors only detected for stepper motors
		return;
	}
	nonLatchedStatusFlags = _driver.readNonLatchedStatusFlags();
	if (digitalRead(_errPin)) {
		latchedStatusFlags = _driver.readLatchedStatusFlagsAndClear();
	} else {
		latchedStatusFlags = 0;
	}
	bool verify = _driver.verifySettings();
	if (!verify && latchedStatusFlags + nonLatchedStatusFlags == 0) {
		//Set power issue flags (custom addition, not in library)
		nonLatchedStatusFlags = 1;
		latchedStatusFlags = 1;
	}
}

//target acts as system output; translates to _delayTime
void Motor::step(int16_t target)
{
	// do not reset if motion disabled or if waiting to enforce speed
	if (disabled || _delayTime+_lastStepTime < millis()) {
		return;
	}

	if (_type == STEPPER) {
		bool direction = target >= 0;
		_delayTime = map(abs(target), _minOutput, _maxOutput, MAX_STEPPER_DELAY, MIN_STEPPER_DELAY);
		if (_driver.getDirection() != direction) {
			_driver.setDirection(direction);
		}
		// The NXT/STEP _minInputInputimum high pulse width is 2 microseconds.
		digitalWrite(_outputPin, HIGH);
		delayMicroseconds(3);
		digitalWrite(_outputPin, LOW);
		delayMicroseconds(3);
	}
	else if (_type == SERVO) {
		_delayTime = SERVO_PERIOD;
		uint16_t pulse_time = map(target, _minOutput, _maxOutput, MIN_SERVO_DELAY, MAX_SERVO_DELAY);
		digitalWrite(_outputPin, HIGH);
		delayMicroseconds(pulse_time);
		digitalWrite(_outputPin, LOW);
	}
	_lastStepTime = millis();
}
