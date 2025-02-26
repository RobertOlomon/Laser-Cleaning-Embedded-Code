#include <AccelStepper.h>
#include "stepper_command.hpp"
#include "pin_def_cleaner.hpp"

// Define the stepper motor interface type
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

// Create an instance of the AccelStepper class
StepperCommand::StepperCommand(uint8_t stepPin, uint8_t dirPin)
    : stepPin(stepPin), dirPin(dirPin), stepper(MOTOR_INTERFACE_TYPE, stepPin, dirPin) {}

void StepperCommand::initializeStepper(float maxSpeed, float acceleration) {
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
}

void StepperCommand::moveTo(long position) {
    stepper.moveTo(position);
}

void StepperCommand::runStepper() {
    stepper.run();
}

void StepperCommand::setMaxSpeed(float speed) {
    stepper.setMaxSpeed(speed);
}

void StepperCommand::setAcceleration(float accel) {
    stepper.setAcceleration(accel);
}

void StepperCommand::kill() {
    digitalWrite(breakPin, LOW);
    stepper.stop();
}

bool StepperCommand::isRunning() {
    return stepper.isRunning();
}