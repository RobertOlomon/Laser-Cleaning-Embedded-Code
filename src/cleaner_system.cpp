#include "cleaner_system.hpp"

#include "TMC5160.h"

Cleaner::Cleaner()
    : jaw_power_params_(),
      jaw_motor_params_(makeJawMotorParams()),
      jaw_rotation_motor_(JAW_ROTATION_CS_PIN, jaw_power_params_, jaw_motor_params_),

      jaw_pos_power_params_(),
      jaw_pos_motor_params_(makeJawPosMotorParams()),
      jaw_pos_motor_(JAW_POSITION_CS_PIN, jaw_pos_power_params_, jaw_pos_motor_params_),

      clamp_power_params_(),
      clamp_motor_params_(makeClampMotorParams()),
      clamp_motor_(CLAMP_CS_PIN, clamp_power_params_, clamp_motor_params_)
{
    reset();
}

TMC5160::MotorParameters Cleaner::makeJawMotorParams()
{
    TMC5160::MotorParameters p;
    p.globalScaler   = 32;
    p.irun           = 16;
    p.ihold          = 0;
    p.freewheeling   = TMC5160_Reg::FREEWHEEL_NORMAL;
    p.pwmOfsInitial  = 30;
    p.pwmGradInitial = 0;
    return p;
}

TMC5160::MotorParameters Cleaner::makeJawPosMotorParams()
{
    TMC5160::MotorParameters p;
    p.globalScaler   = 32;
    p.irun           = 16;
    p.ihold          = 0;
    p.freewheeling   = TMC5160_Reg::FREEWHEEL_NORMAL;
    p.pwmOfsInitial  = 30;
    p.pwmGradInitial = 0;
    return p;
}

TMC5160::MotorParameters Cleaner::makeClampMotorParams()
{
    TMC5160::MotorParameters p;
    p.globalScaler   = 32;
    p.irun           = 16;
    p.ihold          = 0;
    p.freewheeling   = TMC5160_Reg::FREEWHEEL_NORMAL;
    p.pwmOfsInitial  = 30;
    p.pwmGradInitial = 0;
    return p;
}

Cleaner::~Cleaner() = default;

int Cleaner::reset()
{
    state_.jaw_rotation = 0.0f;
    state_.jaw_pos      = 0.0f;
    state_.clamp_pos    = 0.0f;
    state_.is_clamped   = false;

    des_state_.jaw_rotation = 0.0f;
    des_state_.jaw_pos      = 0.0f;
    des_state_.clamp_pos    = 0.0f;
    des_state_.is_clamped   = false;

    // Initialize motors

    if (jaw_rotation_motor_.begin() != EXIT_SUCCESS)
    {
        Serial.println("Failed to initialize jaw rotation motor.");
        return EXIT_FAILURE;
    }
    if (!jaw_pos_motor_.begin() != EXIT_SUCCESS)
    {
        Serial.println("Failed to initialize jaw position motor.");
        return EXIT_FAILURE;
    }
    if (!clamp_motor_.begin() != EXIT_SUCCESS)
    {
        Serial.println("Failed to initialize clamp motor.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}