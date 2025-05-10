#include "stepper_motor.hpp"

#include "TMCStepper.h"
#include "pin_defs.hpp"


StepperMotor::StepperMotor(const StepperMotor::StaticConfig& cfg)
    : AccelStepper(AccelStepper::DRIVER, cfg.pins.step, cfg.pins.dir),
      cfg_(cfg),
      stepper_driver_(
          cfg.pins.cs,
          cfg.rSense,
          (cfg.pins.mosi != 255 ? cfg.pins.mosi : HW_MOSI),  // Choose soft vs. hw SPI
          (cfg.pins.miso != 255 ? cfg.pins.miso : HW_MISO),
          (cfg.pins.sck != 255 ? cfg.pins.sck : HW_SCK))
{
}

void StepperMotor::kill()
{
    // turn on the break and disable the driver
    if (!BrakePin == 255)
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
    stepper_driver_.rms_current(elec_.runCurrent_mA);  // Set motor RMS current
    stepper_driver_.microsteps(elec_.microsteps);    // Set microsteps

    return EXIT_SUCCESS;
}

void StepperMotor::apply(const MotionParams& p) {
    motion_ = p;
    setMaxSpeed(p.maxSpeed);
    setAcceleration(p.acceleration);
};
void StepperMotor::apply(const ElectricalParams& p){
    elec_ = p;
    setRunCurrent(p.runCurrent_mA);
    stepper_driver_.microsteps(p.microsteps);  // Set microsteps to 1/16th
    steps_to_rotation_ = p.microsteps * 200.0f;  // 200 steps per revolution
};

void StepperMotor::setDesStepperLocation(){};

void StepperMotor::turnOff()
{
    if (BrakePin = !255)
    {
        digitalWrite(BrakePin, !BrakeOn);
    }
    stepper_driver_.toff(0);
}

void StepperMotor::setRunCurrent(uint16_t current)
{
    stepper_driver_.rms_current(current);
}