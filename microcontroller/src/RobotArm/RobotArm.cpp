#include "Arduino.h"
#include "RobotArm.h"

#define MIN_TARGET         -1000
#define MAX_TARGET         1000

//units: microseconds
#define MIN_STEPPER_DELAY  10000
#define MAX_STEPPER_DELAY  40000

Sensor* Sensor::sSensor = 0;

Sensor::Sensor(){
	this->_value = 0;
	this->minInput = 0;   // default
	this->maxInput = 100; // default
	this->minReal = 0;
	this->maxReal = 100;
}

void Sensor::zero() {
	if (type == ANALOG) {
		return;
	}
	this->_value = 0;
}

void Sensor::calibrate(int16_t minInput, int16_t maxInput, int16_t minReal, int16_t maxReal) {
	this->minInput = minInput;
	this->maxInput = maxInput;
	this->minReal = minReal;
	this->maxReal = maxReal;
}

void Sensor::init(uint8_t inputPinA=0, uint8_t inputPinB=0) {
	_inputPinA = inputPinA;
	_inputPinB = inputPinB;
	if (_inputPinA == 0 && _inputPinB == 0) {
		type = COUNTER;
	} else if(_inputPinA != 0 && _inputPinB == 0) {
		type = ANALOG;
	} else if(_inputPinA == 0 && _inputPinB != 0) {
		type = ANALOG;
		_inputPinA = _inputPinB;
		_inputPinB = 0;
	}
	else {
		type = DIGITAL;
	}

	switch (type) {
		case DIGITAL: {
			pinMode(_inputPinA, INPUT);
			pinMode(_inputPinB, INPUT);
			sSensor = this;
			attachInterrupt(digitalPinToInterrupt(_inputPinA), Sensor::updateSensorISR, CHANGE);
			attachInterrupt(digitalPinToInterrupt(_inputPinB), Sensor::updateSensorISR, CHANGE);
			break;
		}
		case ANALOG:
			pinMode(_inputPinA, INPUT);
		break;
	}
}

void Sensor::updateSensorISR() {
  if (sSensor != 0)
      sSensor->update();
}

void Sensor::update() {
	switch(type) {
		case ANALOG: {
			uint8_t iterations = 3;
			float average = 0;
			for (int i=0; i<iterations; i++) {
				average += analogRead(_inputPinA);
				delayMicroseconds(10);
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
		case COUNTER: {
			// Nothing can be done here: must be updated from Motor::step function
			break;
		}
	}
}

int32_t Sensor::read() {
	if (type==ANALOG) {
		update();
	}
	return value;
}

Motor::Motor()
{
	nonLatchedStatusFlags = 0;
	latchedStatusFlags = 0;
	disabled = true;
	_type = 0;
	_minPulse = 1000;
	_maxPulse = 2000;
}

//STEPPER
void Motor::init(uint8_t outputPin, uint8_t slaveSelect=0, uint8_t errPin=0, uint8_t dirPin=0, uint8_t slaPin=0)
{
	_outputPin = outputPin;
	_dirPin = dirPin;
	_errPin = errPin;
	_slaPin = slaPin;
	_slaveSelect = slaveSelect;
	pinMode(_outputPin, OUTPUT);
	if (slaveSelect == 0) {
		_type = SERVO;
	}
	else {
		_type = STEPPER;
		pinMode(_dirPin, OUTPUT);
		pinMode(_errPin, INPUT);
		pinMode(_slaPin, INPUT);
		pinMode(_slaveSelect, OUTPUT);
		_driver.init(_slaveSelect);
	}
}

void Motor::controller(float Kp, float Ki) {
	_Ki = Ki;
	_Kp = Kp;
}

//STEPPER
void Motor::start(uint16_t stepmode=1, uint16_t milliamps=0) {
	_stepmode = stepmode;
	switch(_type){
		case STEPPER: {
			digitalWrite(_outputPin, LOW);
			delay(1);
			_driver.resetSettings();
			_driver.setCurrentMilliamps(milliamps);
			_driver.setStepMode(_stepmode);
			_driver.enableDriver();
			delay(1);
		}
		case SERVO: {
			servo.attach(_outputPin);
			disabled = false;
		}
	}
}

void Motor::set(int16_t value) {
	this->setpoint = constrain(value, sensor.minReal, sensor.maxReal);
}


void Motor::read_errors()
{
	if (_type != STEPPER){
		//errors only detected for amis stepper motor driver
		disabled = false;
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
		Serial.println("Verify settings failed; Check power");
		nonLatchedStatusFlags = 1;
		latchedStatusFlags = 1;
	}
	if (latchedStatusFlags != 0 || nonLatchedStatusFlags != 0) {
		disabled = true;
	}
	else {
		disabled = false;
	}
}

void Motor::step(dir) {
	if (dir != stepper.getDirection()) {
		stepper.setDirection(desiredDirection);
	}
	digitalWrite(_stepPin, HIGH);
	delayMicroseconds(3);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(3);
}

bool Motor::update() {
	if (disabled || ((micros() - _lastStepTime) < _delayTime)) {
		return;
	}
	
	// PI control implemented here
	int32_t error = sensor.read() - setpoint;
	// reset and return if setpoint acheived
	if (error == 0) {
		_accumulatedError = 0;
		return true;
	}
	
	uint32_t time = micros();
	_accumulatedError += error * ((time - _lastStepTime)*1e-7);
	int32_t target = _Kp * error + _Ki * _accumulatedError;
	target = constrain(target, MIN_TARGET, MAX_TARGET);
	bool direction = target < 0;
	
	if (_type == STEPPER) {
		_delayTime = map(abs(target), MIN_TARGET, MAX_TARGET, MAX_STEPPER_DELAY, MIN_STEPPER_DELAY);
		_delayTime = _delayTime / _stepmode;
		if (sensor.type == COUNTER) {
			sensor._value += direction ? 1: -1;
		step(direction);
		if (sensor.type == COUNTER) {
			sensor.value += dir ? 1 : -1;
		}
		
		_lastStepTime = micros();
	}
	else if (_type == SERVO) {
		uint16_t setpointPulse = map(setpoint, sensor.minReal, sensor.maxReal, _minPulse, _maxPulse);
		uint16_t currentPulse = map(sensor.read(), sensor.minInput, sensor.maxInput, _minPulse, _maxPulse);
		uint16_t pulse_time;
		if (direction) {
			pulse_time = map(abs(target), 0, MAX_TARGET, currentPulse, setpointPulse)
		} else {
			pulse_time = map(abs(target), MAX_TARGET, 0, currentPulse, setpointPulse)
		}
		servo.writeMicroseconds(pulse_time);
	}
	else {
		Serial.println("Unknown motor type");
	}
	return false;
}