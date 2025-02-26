#ifndef STEPPER_COMMAND_HPP
#define STEPPER_COMMAND_HPP

#include <AccelStepper.h>

class StepperCommand {
public:
    StepperCommand(uint8_t stepPin, uint8_t dirPin);
    void moveTo(long position);
    void initializeStepper(float maxSpeed, float acceleration);
    void setMaxSpeed(float speed);
    void setAcceleration(float accel);
    void runStepper();
    void kill();
    bool isRunning();

private:
    AccelStepper stepper;
    uint8_t stepPin;
    uint8_t dirPin;
};

#endif // STEPPER_COMMAND_HPP