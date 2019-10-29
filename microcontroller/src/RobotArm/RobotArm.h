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

    Sensor();
    void init(uint8_t inputPinA, uint8_t inputPinB=0);
    void calibrate(uint16_t minInput, uint16_t maxInput);
    int32_t read();
    void zero(); //for tareing sensor (digital mode only)
    uint16_t minInput;
    uint16_t maxInput;
    int32_t _value; //access value using read(). Public for testing only
    bool initialized;
  private:
    void update();
    uint8_t _type;
    uint8_t _inputPinA;
    uint8_t _inputPinB;
    static Sensor* sSensor;
    static void updateSensorISR();
};

class Motor {
  public:
    #define STEPPER   1
    #define SERVO     2
    #define CLOSED    0
    #define OPEN      1

    Motor();
    Sensor sensor;
    //stepper
    void init(uint8_t outputPin, uint8_t errPin=0, uint8_t slaveSelect=0);
    void start(uint16_t milliamps=0, uint16_t stepmode=0);
    void controller(uint16_t minOutput, uint16_t maxOutput, float Ki, float Kp);
    void set(uint16_t value);
    int32_t position(); //maps robot interpreted position to human readable
    bool update();
    void read_errors();

    uint16_t nonLatchedStatusFlags;
    uint16_t latchedStatusFlags;
    bool disabled;
    uint16_t setpoint;
    uint8_t _type;

  private:
    void step(int16_t target);
    uint8_t _errPin;
    uint8_t _mode;
    uint16_t _minOutput;
    uint16_t _maxOutput;
    uint8_t _outputPin;
    uint8_t _slaveSelect;
    float _Kp;
    float _Ki;
    AMIS30543 _driver;
    uint32_t _lastStepTime;
    int32_t _accumulatedError;
    uint16_t _delayTime; //microseconds
};

#endif
