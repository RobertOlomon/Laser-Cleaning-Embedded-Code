#pragma once
#include <Arduino.h>

#include "TMCStepper.h"

/**
 * This class wraps the TMC5160 driver to provide a simple interface for controlling a stepper
 * motor. It moves the begin of the TMC5160 class to the constructor and ties a break to the
 * interface.
 *
 */
class StepperMotor : public TMC5160Stepper
{
public:
    StepperMotor(
        uint8_t CS_PIN,
        TMC5160::PowerStageParameters powerStageParams_,
        TMC5160::MotorParameters motorParams_);
    int begin();
    void kill();
    void setRunCurrent(uint8_t currentLimit);
    void turnOff();

private:
    TMC5160Stepper stepper_;  // The wrapped driver instance
    TMC5160::PowerStageParameters powerStageParams_;
    TMC5160::MotorParameters motorParams_;
    const int breakPin = 10;  // Replace with actual pin definition or move to constructor
};
