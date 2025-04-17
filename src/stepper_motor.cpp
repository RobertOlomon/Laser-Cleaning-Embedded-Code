#include "stepper_motor.hpp"

#include "TMCStepper.h"

// Constructor: initialize wrapped TMC5160 instance
StepperMotor::StepperMotor(
    uint8_t CS_PIN,
    float R_SENSE,
    uint8_t SW_MOSI,
    uint8_t SW_MISO,
    uint8_t SW_SCK,
    uint8_t BrakePin)
    : stepper_driver_(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK)
{
    // initalized the break if it has one
    this->BrakePin = BrakePin;
    if (BrakePin != 255)
    {
        pinMode(BrakePin, OUTPUT);
        digitalWrite(BrakePin, !BrakeOn);
    }
}

void StepperMotor::kill()
{
    // turn on the break and disable the driver
    if (BrakePin)
    {
        pinMode(BrakePin, OUTPUT);
        digitalWrite(BrakePin, BrakeOn);
    }
    stepper_driver_.toff(0);  // Disable driver in software
}

int StepperMotor::begin()
{
    stepper_driver_.begin();
    stepper_driver_.toff(5);           // Enables driver in software
    stepper_driver_.rms_current(600);  // Set motor RMS current
    stepper_driver_.microsteps(16);    // Set microsteps to 1/16th
    stepper_driver_.en_pwm_mode(1);    // Enable extremely quiet stepping
    stepper_driver_.pwm_autoscale(1);  // Slowest value (1...15)

    return EXIT_SUCCESS;
}

void StepperMotor::turnOff()
{
    if (BrakePin = !255)
    {
        digitalWrite(BrakePin, !BrakeOn);
    }
    stepper_driver_.toff(0);
}