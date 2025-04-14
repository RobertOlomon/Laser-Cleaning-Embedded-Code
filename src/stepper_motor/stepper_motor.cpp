#include "stepper_motor.hpp"
#include "TMC5160.h"
#include "pin_def.hpp"

// Constructor: initialize wrapped TMC5160 instance
StepperMotor::StepperMotor(uint8_t CS_PIN) : stepper_(CS_PIN) {
  motorParams_.globalScaler = 98; // Adapt to your driver and motor
  motorParams_.irun = 31;
  motorParams_.ihold = 16;
}

void StepperMotor::initializeStepper(float maxSpeed, float acceleration) {
  stepper_.setCurrentPosition(0);
  stepper_.setMaxSpeed(maxSpeed);
  stepper_.setAcceleration(acceleration);
}

void StepperMotor::kill() {
  digitalWrite(breakPin, LOW);
  stepper_.stop();
}

void StepperMotor::setRunCurrent(uint8_t currentLimit) {
  motorParams_.irun = currentLimit;
}

void StepperMotor::begin() {
  stepper_.begin(powerStageParams_, motorParams_, TMC5160::NORMAL_MOTOR_DIRECTION);
  stepper_.setRampMode(TMC5160::POSITIONING_MODE);
  stepper_.setRampSpeeds(0, 0.1, 100);
  stepper_.setMaxSpeed(200);
  stepper_.setAccelerations(250, 350, 500, 700);
}
