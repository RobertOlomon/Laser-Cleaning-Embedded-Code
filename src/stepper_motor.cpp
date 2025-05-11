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
    // Check to make sure that the driver is powered before enabling it if a pin is defined
    if (ESTOP_VSAMPLE_PIN != 255)
    {
        pinMode(ESTOP_VSAMPLE_PIN, INPUT);
        constexpr int ESTOP_VSAMPLE_THRESHOLD = 1023/2;
        if (analogRead(ESTOP_VSAMPLE_PIN) < ESTOP_VSAMPLE_THRESHOLD)
        {
            Serial.println("Driver voltage low, E-STOP likely engaged, not starting motor.");
            return EXIT_FAILURE;
        }
    }

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
    stepper_driver_.rms_current(elec_.runCurrent_mA);
    stepper_driver_.microsteps(p.microsteps); 
};

void StepperMotor::apply(const PhysicalParams& p) {
    phys_ = p;
};