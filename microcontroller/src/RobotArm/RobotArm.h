#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "SPI.h"
#include "AMIS30543.h"

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
    Sensor sensor;
    void init(uint8_t outputPin, uint8_t pin1=0, uint8_t pin2=0, uint8_t pin3=0, uint8_t pin4=0, uint8_t pin5=0, uint8_t pin6=0);
    void start(uint16_t milliamps=0, uint16_t stepmode=1);
    void controller(float Kp, float Ki);
    void set(int16_t value);
    bool update();
    void read_errors();
    void manual_step(bool dir); //only for non AMIS stepper (rename later)
	void amis_step(bool dir);  //for testing, bypasses control system
	
    uint16_t nonLatchedStatusFlags;
    uint16_t latchedStatusFlags;
    bool disabled;
    int16_t setpoint;
    uint8_t _type;
    AMIS30543 _driver;

  private:
    void step(int32_t target);
    // AMIS STEPPER
    uint8_t _errPin;
    uint8_t _dirPin;
    int16_t _minOutput;
    int16_t _maxOutput;
    uint8_t _outputPin;
    uint8_t _slaveSelect;
    uint8_t _stepmode;

    // MANUAL STEPPER
    bool _coilADir;
    bool _coilBDir;
    uint8_t _stepCount; //Not to be used with PI control
    uint8_t _a1;
    uint8_t _a2;
    uint8_t _b1;
    uint8_t _b2;
    uint8_t _pwmA;
    uint8_t _pwmB;

    float _Kp;
    float _Ki;
    uint32_t _lastStepTime;
    int32_t _accumulatedError;
    uint32_t _delayTime; //microseconds
};

#endif
