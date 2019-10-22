#ifndef RobotArm_h
#define RobotArm_h

#include "Arduino.h"
#include "SPI.h"
#include "AMIS30543.h"

//https://www.arduino.cc/en/Hacking/LibraryTutorial

class Motor {
  public:
    #define STEPPER   0
    #define SERVO     1
    #define CLOSED    0
    #define OPEN      1


    Motor();
    //stepper
    void init(uint8_t outputPin, uint8_t dirPin, uint8_t errPin, uint8_t slaveSelect);
    //servo
    void init(uint8_t outputPin);
    //stepper
    void start(uint16_t milliamps, uint16_t stepmode);
    //servo
    void start();
    void calibrate(uint16_t minOutput, uint16_t maxOutput);
    void controller(uint8_t inputPin, uint16_t minInput, uint16_t maxInput, float Ki, float Kp);
    void set(uint16_t value);
    bool update();
    void readErrors();
    float get_position();

    uint16_t nonLatchedStatusFlags;
    uint16_t latchedStatusFlags;
    bool disabled;
    uint16_t setpoint;

  private:
    void step(int16_t target);
    
    uint8_t _errPin;
    uint8_t _mode;
    uint16_t _minInput;
    uint16_t _maxInput;
    uint16_t _minOutput;
    uint16_t _maxOutput;
    uint8_t _type;
	  uint8_t _inputPin;
    uint8_t _dirPin;
    uint8_t _outputPin;
    uint8_t _slaveSelect;
    float _Kp;
    float _Ki;
    AMIS30543 _driver;
    uint32_t _lastStepTime;
    int32_t _accumulatedError;
    uint16_t _delayTime; //microseconds
    int32_t _totalSteps;
};

#endif
