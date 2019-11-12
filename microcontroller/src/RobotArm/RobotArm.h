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
    void calibrate(int16_t minInput, int16_t maxInput);
    int32_t read();
    void zero(); //for tareing sensor (digital mode only)
    int16_t minInput;
    int16_t maxInput;
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
    #define STEPPER   1
    #define SERVO     2

    Motor();
    Sensor sensor;
    //stepper
    void init(uint8_t outputPin, uint8_t errPin=0, uint8_t slaveSelect=0);
    void start(uint16_t milliamps=0, uint16_t stepmode=0);
    void controller(uint16_t minOutput, uint16_t maxOutput, float Ki, float Kp);
    void set(int16_t value);
    int32_t position(); //maps robot interpreted position to human readable
    bool update();
    void read_errors();

    uint16_t nonLatchedStatusFlags;
    uint16_t latchedStatusFlags;
    bool disabled;
    int16_t setpoint;
    uint8_t _type;
    AMIS30543 _driver;

  private:
    void step(int16_t target);
    uint8_t _errPin;
    uint16_t _minOutput;
    uint16_t _maxOutput;
    uint8_t _outputPin;
    uint8_t _slaveSelect;
    float _Kp;
    float _Ki;
    uint32_t _lastStepTime;
    int32_t _accumulatedError;
    uint32_t _delayTime; //microseconds
};

#endif
