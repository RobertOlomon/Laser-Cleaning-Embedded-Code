#pragma once
#include "pin_defs.hpp"
#include "stepper_motor.hpp"


constexpr StepperMotor::StaticConfig jawRotationCfg{
    /* pins */ {JAW_ROTATION_CS_PIN, JAW_ROTATION_STEP_PIN, JAW_ROTATION_DIR_PIN, 255},
    /* rSense */ StepperMotor::TMC5160_PRO_RSENSE,
    /* name   */ "Jaw Rotation Motor"};

constexpr StepperMotor::StaticConfig jawPosCfg{
    {JAW_POSITION_CS_PIN, JAW_POSITION_STEP_PIN, JAW_POSITION_DIR_PIN, 255},
    StepperMotor::TMC5160_PRO_RSENSE,
    "Jaw Position Motor"};

constexpr StepperMotor::StaticConfig clampCfg{
    {CLAMP_CS_PIN, CLAMP_DIR_PIN, CLAMP_STEP_PIN, 255},  // hardware SPI -> MOSI/MISO/SCK = 255
    StepperMotor::TMC5160_PRO_RSENSE,
    "Clamp Motor"};

/* Electrical Presets */
constexpr StepperMotor::ElectricalParams JawRotationElectrical{800, 4};
constexpr StepperMotor::ElectricalParams JawPositionElectrical{1200, 32};
constexpr StepperMotor::ElectricalParams lowCurrent{500, 16};

/* Physical Presets */
constexpr StepperMotor::PhysicalParams JawRotationPhysical{
    M_PI / 200 / JawRotationElectrical.microsteps};  // 200 steps per revolution
constexpr StepperMotor::PhysicalParams JawPositionPhysical{
    5.0f / 200 / JawPositionElectrical.microsteps};  // 200 steps per revolution, 5mm pitch

/* Motion Presets */
constexpr StepperMotor::MotionParams JawRotationMotion{10000, 10000};
constexpr StepperMotor::MotionParams JawPositionMotion{
    800 * JawPositionElectrical.microsteps,
    20 * JawPositionElectrical.microsteps};