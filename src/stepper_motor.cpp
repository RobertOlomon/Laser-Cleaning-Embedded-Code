#include "stepper_motor.hpp"

#include "TMC5160.h"

// Constructor: initialize wrapped TMC5160 instance
StepperMotor::StepperMotor(
    uint8_t CS_PIN,
    TMC5160::PowerStageParameters powerStageParams_,
    TMC5160::MotorParameters motorParams_)
    : stepper_(CS_PIN),
      powerStageParams_(powerStageParams_),
      motorParams_(motorParams_)
{
}

void StepperMotor::kill()
{
    digitalWrite(breakPin, LOW);
    stepper_.stop();
}

void StepperMotor::setRunCurrent(uint8_t currentLimit) { motorParams_.irun = currentLimit; }

int StepperMotor::begin()
{
    if (stepper_.begin(powerStageParams_, motorParams_, TMC5160::NORMAL_MOTOR_DIRECTION) !=
        EXIT_SUCCESS)
    {
        Serial.println("Failed to initialize stepper motor.");
        return EXIT_FAILURE;
    }
    stepper_.setRampMode(TMC5160::POSITIONING_MODE);
    stepper_.setRampSpeeds(0, 0.1, 100);
    stepper_.setMaxSpeed(200);
    stepper_.setAccelerations(250, 350, 500, 700);
    return EXIT_SUCCESS;
}

void StepperMotor::turnOff()
{
    stepper_.disable();
    digitalWrite(breakPin, LOW);
}