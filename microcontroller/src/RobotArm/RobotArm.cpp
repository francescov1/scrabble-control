#include "Arduino.h"
#include "RobotArm.h"

#define MIN_TARGET         -1000
#define MAX_TARGET         1000

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

void Sensor::calibrate(uint32_t minInput, uint32_t maxInput, uint32_t minReal, uint32_t maxReal) {
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
	if (type==ANALOG) {
		this->_value = analogRead(_inputPinA);
	}
	else if(type==DIGITAL){
		bool A_state = digitalRead(_inputPinA);
		bool B_state = digitalRead(_inputPinB);
		if (A_state == B_state) {
			this->_value++;
		}
		else {
			this->_value--;
		}
	}
	else{
			// Nothing can be done here: must be updated from Motor::step function
	}
}

int32_t Sensor::read() {
	if (type==ANALOG) {
		this->update();
	}
	return this->_value;
}

Motor::Motor()
{
	nonLatchedStatusFlags = 0;
	latchedStatusFlags = 0;
	disabled = true;
	_type = 0;
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
		this->_type = SERVO;
		_minPulse = 1000;
		_maxPulse = 2000;
	}
	else {
		_type = STEPPER;
		_minPulse = 1000;
		_maxPulse = 5000;
	}
}

void Motor::controller(float Kp, float Ki) {
	_Ki = Ki;
	_Kp = Kp;
}

//STEPPER
void Motor::start(uint16_t stepmode=1, uint16_t milliamps=0) {
	_stepmode = stepmode;
	if (_type == STEPPER) {
		pinMode(_outputPin, OUTPUT);
		digitalWrite(_outputPin, LOW);
		delay(1);
		_driver.init(_slaveSelect);
		//digitalWrite(_dirPin, LOW);
		//pinMode(_dirPin, OUTPUT);
		delay(1);
		_driver.resetSettings();
		_driver.setCurrentMilliamps(milliamps);
		_driver.setStepMode(_stepmode);
		_driver.enableDriver();
		delay(1);
		this->read_errors();
	} else if (_type == SERVO) {
		servo.attach(_outputPin);
		disabled = false;
	} else {
		Serial.println("Motor type not defined");
	}
}

void Motor::set(uint32_t value) {
	this->setpoint = constrain(value, sensor.minReal, sensor.maxReal);
}

void Motor::calibrate(uint16_t minPulse, uint16_t maxPulse) {
	_minPulse = minPulse;
	_maxPulse = maxPulse;
}

void Motor::read_errors()
{
	if (_type != STEPPER){
		//errors only detected for amis stepper motor driver
		latchedStatusFlags = 0;
		nonLatchedStatusFlags = 0;
		return;
	}
	nonLatchedStatusFlags = _driver.readNonLatchedStatusFlags();
	if (digitalRead(_errPin) && latchedStatusFlags == 0) {
		latchedStatusFlags = _driver.readLatchedStatusFlagsAndClear();
	} else {
		latchedStatusFlags = 0;
	}

	bool power = true;
	if (!_driver.verifySettings()) {
		_driver.applySettings();
		power = _driver.verifySettings();
	}
	if (!power && (latchedStatusFlags + nonLatchedStatusFlags == 0)) {
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

void Motor::step(bool dir) {
	Serial.println("Step");
	if (_type != STEPPER) {
		Serial.println("Can't step on non-stepper");
		return;
	}
	if (dir != _driver.getDirection()) {
		Serial.println("Changing direction");
		_driver.setDirection(dir);
	}
	digitalWrite(_outputPin, HIGH);
	delayMicroseconds(3);
    digitalWrite(_outputPin, LOW);
    delayMicroseconds(3);
}

bool Motor::update() {
	//read_errors();
	if (disabled) {
		return false;
	}
	if (((uint16_t)(micros() - _lastStepTime) < _delayTime)) {
		return false;
	}
	_lastStepTime = micros();
	// PI control implemented here
	uint32_t mSetpoint = map(setpoint, sensor.minReal, sensor.maxReal, sensor.minInput, sensor.maxInput);
	if (sensor.type==COUNTER) {
		mSetpoint = mSetpoint*_stepmode;
	}
	int32_t error = sensor.read() - mSetpoint;
	//Serial.println(error);
	// reset and return if setpoint acheived
	if (error == 0) {
		_accumulatedError = 0;
		moving = false;
		return true;
	} else {
		moving = true;
	}
	
	//uint32_t time = micros();
	//_accumulatedError += error * ((time - _lastStepTime)*1e-7);
	int32_t target = _Kp * error; //+ _Ki * _accumulatedError;
	target = constrain(target, MIN_TARGET, MAX_TARGET);
	bool direction = error < 0;
	
	if (_type == STEPPER) {
		_delayTime = map(abs(target), MIN_TARGET, MAX_TARGET, _maxPulse, _minPulse);
		_delayTime = _delayTime;
		step(direction);
		if (sensor.type == COUNTER) {
			sensor._value += direction ? 1 : -1;
		}
	}
	else if (_type == SERVO) {
		/*
		uint16_t setpointPulse = map(setpoint, sensor.minReal, sensor.maxReal, _minPulse, _maxPulse);
		uint16_t currentPulse = map(sensor.read(), sensor.minInput, sensor.maxInput, _minPulse, _maxPulse);
		uint16_t pulse_time;
		if (direction) {
			pulse_time = map(abs(target), 0, MAX_TARGET, currentPulse, setpointPulse);
		} else {
			pulse_time = map(abs(target), MAX_TARGET, 0, currentPulse, setpointPulse);
		}
		Serial.println(target);
		*/
		uint16_t pulse_time = map(setpoint, sensor.minReal, sensor.maxReal, _minPulse, _maxPulse);
		servo.writeMicroseconds(mSetpoint);
		_delayTime = 10000;
	}
	else {
		Serial.println("Unknown motor type");
	}
	return false;
}
