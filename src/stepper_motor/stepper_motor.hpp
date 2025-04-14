#pragma once
#include <Arduino.h>
#include "TMC5160.h"

class StepperMotor {
public:
    StepperMotor(uint8_t CS_PIN);
    void begin();
    void initializeStepper(float maxSpeed, float acceleration);
    void kill();
    void setRunCurrent(uint8_t currentLimit);

private:
    TMC5160_SPI stepper_;  // The wrapped driver instance
    TMC5160::PowerStageParameters powerStageParams_;
    TMC5160::MotorParameters motorParams_;
    const int breakPin = 10; // Replace with actual pin definition or move to constructor
};

