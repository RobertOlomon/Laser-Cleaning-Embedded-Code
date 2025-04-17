#pragma once
#include <optional>

#include <AccelStepper.h>
#include <Arduino.h>

#include "TMCStepper.h"

/**
 * This class wraps the TMC5160 driver to provide a simple interface for controlling a stepper
 * motor. It moves the begin of the TMC5160 class to the constructor and ties a break to the
 * interface.
 *
 */
class StepperMotor : public AccelStepper
{
public:
    static constexpr float TMC5160_PLUS_RSENSE = 0.022f;
    static constexpr float TMC5160_PRO_RSENSE  = 0.075f;
    struct MoterParams
    {
        float max_speed     = 1000.0f;  // Maximum speed in steps per second
        float acceleration  = 1000.0f;  // Acceleration in steps per second^2
        float current_limit = 2.0f;     // Current limit in A
    };
    
    StepperMotor(
        uint8_t CS_PIN,
        float R_SENSE,
        uint8_t BrakePin = 255);

    StepperMotor(
        uint8_t CS_PIN,
        float R_SENSE,
        uint8_t SW_MOSI,
        uint8_t SW_MISO,
        uint8_t SW_SCK,
        uint8_t BrakePin = 255);

    int begin();
    void kill();
    void setRunCurrent(uint8_t currentLimit);
    void turnOff();

private:
    uint8_t BrakePin;                // Pin used to break the motor
    TMC5160Stepper stepper_driver_;  // The wrapped driver instance
    bool BrakeOn = LOW;              // Define which direction for the pin to activate the break.
};
