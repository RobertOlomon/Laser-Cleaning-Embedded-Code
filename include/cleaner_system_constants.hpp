#pragma once
#include "stepper_motor.hpp"
#include "pin_defs.hpp"

constexpr StepperMotor::StaticConfig jawRotationCfg{
    /* pins */ {JAW_ROTATION_CS_PIN, JAW_ROTATION_STEP_PIN, JAW_ROTATION_DIR_PIN, 255},
    /* rSense */ StepperMotor::TMC5160_PLUS_RSENSE,
    /* name   */ "Jaw Rotation Motor"};

constexpr StepperMotor::StaticConfig jawPosCfg{
    {JAW_POSITION_CS_PIN, 255, 255, 255},
    StepperMotor::TMC5160_PRO_RSENSE,
    "Jaw Position Motor"};

constexpr StepperMotor::StaticConfig clampCfg{
    {CLAMP_CS_PIN, 255, 255, 255},  // hardware SPI -> MOSI/MISO/SCK = 255
    StepperMotor::TMC5160_PRO_RSENSE,
    "Clamp Motor"};

/* Motion Presets */
constexpr StepperMotor::MotionParams JawRotationMotion{50000, 100000};

/* Electrical Presets */
constexpr StepperMotor::ElectricalParams JawRotationElectrical{600,16};
constexpr StepperMotor::ElectricalParams lowCurrent{500,16};

/* Physical Presets */
constexpr StepperMotor::PhysicalParams JawRotationPhysical{M_PI / 200 / JawRotationElectrical.microsteps};  // 200 steps per revolution