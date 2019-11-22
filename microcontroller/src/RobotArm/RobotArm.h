#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "SPI.h"
#include "AMIS30543.h"
#include "Servo.h"

//https://www.arduino.cc/en/Hacking/LibraryTutorial

class Sensor {
  public:
    #define DIGITAL  0
    #define ANALOG   1
    #define COUNTER  2
    #define OPEN     3

    Sensor();
    void init(uint8_t inputPinA=0, uint8_t inputPinB=0);
    void calibrate(int16_t minInput, int16_t maxInput, int16_t minReal, int16_t maxReal);
    int32_t read();
    void zero(); //for tareing sensor (digital mode only)
    int16_t minInput;
    int16_t maxInput;
	int16_t minReal;
    int16_t maxReal;
    int32_t _value; //access value using read(). Public for testing only
    uint8_t type;
  private:
    void update();
    uint8_t _inputPinA;
    uint8_t _inputPinB;
    static Sensor* sSensor;
    static void updateSensorISR();
};

class Motor {
  public:
    #define STEPPER        1
    #define AMIS_STEPPER   2
    #define SERVO          3

    Motor();
    void init(uint8_t outputPin, uint8_t slaveSelect=0, uint8_t errPin=0, uint8_t dirPin=0, uint8_t slaPin=0);
    void start(uint16_t stepmode=1, uint16_t milliamps=0);
    void controller(float Kp, float Ki);
    void set(int16_t value);
	bool update();
	void step(bool dir); //for testing
    void read_errors();
	
    uint16_t nonLatchedStatusFlags;
    uint16_t latchedStatusFlags;
    bool disabled;
    int16_t setpoint;
    uint8_t _type;
    AMIS30543 _driver;
	Servo servo;
	Sensor sensor;

  private:
    // AMIS STEPPER
    uint8_t _errPin;
    uint8_t _dirPin;
	uint8_t _slaPin;
    int16_t _minOutput;
    int16_t _maxOutput;
    uint8_t _outputPin;
    uint8_t _slaveSelect;
    uint8_t _stepmode;
	// SERVO
	uint16_t _minPulse;
	uint16_t _maxPulse;
	uint16_t _servoDelayTime;

    float _Kp;
    float _Ki;
    uint32_t _lastStepTime;
    int32_t _accumulatedError;
    uint32_t _delayTime; //microseconds
};

#endif
