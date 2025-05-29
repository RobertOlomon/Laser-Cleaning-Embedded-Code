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
    {CLAMP_CS_PIN, CLAMP_STEP_PIN, CLAMP_DIR_PIN, 255},  // hardware SPI -> MOSI/MISO/SCK = 255
    StepperMotor::TMC5160_PRO_RSENSE,
    "Clamp Motor"};

/* Electrical Presets */
constexpr StepperMotor::ElectricalParams JawRotationElectrical{1500, 32};
constexpr StepperMotor::ElectricalParams JawPositionElectrical{1200, 32};
constexpr StepperMotor::ElectricalParams clampElectrical{1200, 32};

/* Physical Presets */
constexpr StepperMotor::PhysicalParams JawRotationPhysical{
    M_TWOPI / 200 / JawRotationElectrical.microsteps / 10.0f};  // 200 steps per revolution, 10:1 ratio
constexpr StepperMotor::PhysicalParams JawPositionPhysical{
    5.0f / 200 / JawPositionElectrical.microsteps};  // 200 steps per revolution, 5mm pitch
constexpr StepperMotor::PhysicalParams clampPhysical{M_TWOPI / 200 / clampElectrical.microsteps};  // 200 steps per revolution

/* Motion Presets */
constexpr StepperMotor::MotionParams JawRotationMotion{
    400 * JawRotationElectrical.microsteps,
    10 * JawRotationElectrical.microsteps};
constexpr StepperMotor::MotionParams JawPositionMotion{
    800 * JawPositionElectrical.microsteps,
    20 * JawPositionElectrical.microsteps};
constexpr StepperMotor::MotionParams ClampMotion{200 * clampElectrical.microsteps,
    10 * clampElectrical.microsteps};