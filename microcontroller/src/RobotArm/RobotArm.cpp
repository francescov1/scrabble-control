#include "Arduino.h"
#include "RobotArm.h"

#define MIN_STEPPER_DELAY  200
#define MAX_STEPPER_DELAY  2000
#define MIN_SERVO_DELAY    550
#define MAX_SERVO_DELAY    2450
#define SERVO_PERIOD       20000


Motor::Motor()
{
	nonLatchedStatusFlags = 0;
	latchedStatusFlags = 0;
	disabled = true;
	_mode = OPEN;
}

//STEPPER
void Motor::init(uint8_t outputPin, uint8_t dirPin, uint8_t errPin, uint8_t slaveSelect)
{
	_type = STEPPER;
	_errPin = errPin;
	pinMode(_inputPin, INPUT);
	_driver.init(_slaveSelect);
	disabled = digitalRead(_errPin);
}

//SERVO
void Motor::init(uint8_t outputPin)
{
	_type = SERVO;
	_outputPin = outputPin;
	pinMode(_outputPin, OUTPUT);
	disabled = false;
}

void Motor::controller(uint8_t inputPin, uint16_t minInput, uint16_t maxInput, float Ki, float Kp) {
	_maxInput = minInput;
	_maxInput = minInput;
	_inputPin = inputPin;
	_Ki = Ki;
	_Kp = Kp;
	_mode = CLOSED;
}

//STEPPER
void Motor::start(uint16_t milliamps, uint16_t stepmode) {
	if (_type != STEPPER){
		return;
	}
	_stepmode = stepmode;
	digitalWrite(_outputPin, LOW);
	digitalWrite(_dirPin, LOW);
	delay(1);
	_driver.resetSettings();
	_driver.setCurrentMilliamps(milliamps);
	_driver.setStepMode(_stepmode);
	_driver.enableDriver();
	disabled = false;
}

//SERVO
void Motor::start() {
	if (_type != SERVO) {
		return;
	}
	digitalWrite(_outputPin, LOW);
	disabled = false;
}

void Motor::calibrate(uint16_t minOutput, uint16_t maxOutput) {

}

void Motor::set(uint16_t value) {
	if (_minOutput >= _maxOutput) {
		//ignore min/max (likely defaulted to 0)
		setpoint = value;
	}
	else {
		setpoint = constrain(value, _minOutput, _maxOutput);
	}
}

float Motor::get_position()
{
	float position;
	if (_mode == OPEN) {
		position = -1;
	}
	else {
		position = float(map(analogRead(_inputPin), _minInput, _maxInput, _minOutput, _maxOutput));
	}
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
		int16_t error = get_position() - setpoint;
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

void Motor::readErrors()
{
	if (_type != STEPPER){
		//errors only detected for stepper motors
		return;
	}
	if (!digitalRead(_errPin)) {
		return;
	}
	latchedStatusFlags = _driver.readLatchedStatusFlagsAndClear();
	nonLatchedStatusFlags = _driver.readNonLatchedStatusFlags();
	bool verify = _driver.verifySettings();
	if (latchedStatusFlags + nonLatchedStatusFlags > 0 || verify) {
		disabled = true;
		_driver.disableDriver();
	}
	else {
		disabled = false;
		_driver.enableDriver();
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
		digitalWrite(_dirPin, !(direction));
		// The NXT/STEP _minInputInputimum high pulse width is 2 microseconds.
		digitalWrite(_outputPin, HIGH);
		delayMicroseconds(3);
		digitalWrite(_outputPin, LOW);
		delayMicroseconds(3);
		_totalSteps += 1;
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
