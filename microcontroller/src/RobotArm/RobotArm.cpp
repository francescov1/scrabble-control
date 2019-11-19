#include "Arduino.h"
#include "RobotArm.h"

#define MIN_OUTPUT         -1000
#define MAX_OUTPUT         1000

//units: microseconds
#define MIN_STEPPER_DELAY  10000
#define MAX_STEPPER_DELAY  40000
#define MIN_SERVO_DELAY    550000
#define MAX_SERVO_DELAY    2450000
#define SERVO_PERIOD       20000000


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
	uint16_t realVal = map(_value, minInput, maxInput, minReal, maxReal);
	return realVal;
}

Motor::Motor()
{
	nonLatchedStatusFlags = 0;
	latchedStatusFlags = 0;
	disabled = true;
	_type = 0;
}

//STEPPER
void Motor::init(uint8_t outputPin, uint8_t pin1=0, uint8_t pin2=0, uint8_t pin3=0, uint8_t pin4=0, uint8_t pin5=0, uint8_t pin6=0)
{
	_outputPin = outputPin;
	pinMode(_outputPin, OUTPUT);
	if (pin1>0 && !pin2 && !pin3 && !pin4 && !pin5 && !pin6) {
		_type = SERVO;
	}
	else if (pin1>0 && pin2>0 && pin3>0 && !pin4 && !pin5 && !pin6) {
		_type = AMIS_STEPPER;
		_errPin = pin1;
		_dirPin = pin2;
		_slaveSelect = pin3;
		pinMode(_errPin, INPUT);
		pinMode(_dirPin, INPUT);
		pinMode(_slaveSelect, OUTPUT);
		_driver.init(_slaveSelect);
	}
	else {
		_type = STEPPER;
		_pwmA = pin1;
		_a1 = pin2;
		_a2 = pin3;
		_pwmB = pin4;
		_b1 = pin5;
		_b2 = pin6;
		pinMode(_pwmA, OUTPUT);
		pinMode(_a1, OUTPUT);
		pinMode(_a2, OUTPUT);
		pinMode(_pwmB, OUTPUT);
		pinMode(_b1, OUTPUT);
		pinMode(_b2, OUTPUT);
		_coilADir = 0;
		_coilBDir = 0;
	}
}

void Motor::controller(float Kp, float Ki) {
	_Ki = Ki;
	_Kp = Kp;
}

//STEPPER
void Motor::start(uint16_t milliamps=0, uint16_t stepmode=1) {
	_stepmode = stepmode;
	switch(_type){
		case STEPPER: {
			digitalWrite(_a1, LOW);
			digitalWrite(_a2, LOW);
			digitalWrite(_b1, LOW);
			digitalWrite(_b2, LOW);
			break;
		}
		case AMIS_STEPPER: {
			digitalWrite(_outputPin, LOW);
			delay(1);
			_driver.resetSettings();
			_driver.setCurrentMilliamps(milliamps);
			_driver.setStepMode(_stepmode);
			_driver.enableDriver();
			delay(1);
		}
		case SERVO: {
			digitalWrite(_outputPin, LOW);
			disabled = false;
		}
	}
}

void Motor::set(int16_t value) {
	this->setpoint = value;
}


void Motor::read_errors()
{
	if (_type != AMIS_STEPPER){
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

void Motor::amis_step(bool dir) {
	if (_driver.getDirection() != dir) {
		Serial.println("Change direction");
		_driver.setDirection(dir);
	}
	// The NXT/STEP minimum high pulse width is 2 microseconds.
	digitalWrite(_outputPin, HIGH);
	delayMicroseconds(3);
	digitalWrite(_outputPin, LOW);
	delayMicroseconds(3);
}

void Motor::manual_step(bool dir) {
	int cycleNum = _stepCount % _stepmode;
	if (cycleNum == 0) {
		// trigger next coil state
		_stepCount = 0;
		if (_coilADir && _coilBDir) {
			_coilADir = !dir;
			_coilBDir = dir;
		}
		else if(!_coilADir && !_coilADir) {
			_coilADir = dir;
			_coilBDir = !dir;
		}
		else if(_coilADir && !_coilBDir) {
			_coilADir = dir;
			_coilADir = dir;
		}
		else if (!_coilADir && _coilBDir) {
			_coilADir = !dir;
			_coilBDir = !dir;
		}
	}
	if (_coilADir) {
		// forward
		digitalWrite(_a1, HIGH);
		digitalWrite(_a2, LOW);
	} else {
		digitalWrite(_a1, HIGH);
		digitalWrite(_a2, LOW);
	}
	if (_coilBDir) {
		// forward
		digitalWrite(_b1, HIGH);
		digitalWrite(_b2, LOW);
	} else {
		digitalWrite(_b1, HIGH);
		digitalWrite(_b2, LOW);
	}
	int coilAPwr = int((cycleNum+1)/_stepmode)*255;
	analogWrite(_pwmA, coilAPwr); // change for microstep
	analogWrite(_pwmB, 255); // change for microstep

	_stepCount += dir ? 1: -1;
}

//target acts as system output; translates to _delayTime
void Motor::step(int32_t target)
{
	// do not reset if motion disabled or if waiting to enforce speed
	if (disabled || (micros() < (_delayTime + _lastStepTime))) {
		return;
	}
	bool direction = target < 0;

	if (_type == SERVO) {
		_delayTime = SERVO_PERIOD;
		uint16_t pulse_time = map(abs(target), sensor.minInput, sensor.maxInput, MIN_SERVO_DELAY, MAX_SERVO_DELAY);
		digitalWrite(_outputPin, HIGH);
		delayMicroseconds(pulse_time);
		digitalWrite(_outputPin, LOW);
	}
	else {
		//http://www.tigoe.com/pcomp/code/circuits/motors/stepper-motors/
		//https://www.pololu.com/docs/pdf/0J15/motor_driver_application_note.pdf
		_delayTime = map(abs(target), MIN_OUTPUT, MAX_OUTPUT, MAX_STEPPER_DELAY, MIN_STEPPER_DELAY);
		_delayTime = _delayTime / _stepmode;
		Serial.println(target);
		//Serial.println(_delayTime);
		if (sensor.type == COUNTER) {
			sensor._value += direction ? 1: -1;
		}
		if (_type == STEPPER) {
			manual_step(direction);
		} else if(_type == AMIS_STEPPER) {
			amis_step(direction);
		}
	}
	_lastStepTime = micros();
}

bool Motor::update()
{		
	// PI control implemented here
	int32_t error = sensor.read() - setpoint;
	// reset and return if setpoint acheived
	if (error == 0) {
		_accumulatedError = 0;
		return true;
	}
	uint32_t time = micros();
	_accumulatedError += error * ((time - _lastStepTime)*1e-6);
	int32_t target = _Kp * error; //_Ki * _accumulatedError;
	target = constrain(target, MIN_OUTPUT, MAX_OUTPUT);
	step(target);
	return false;
}