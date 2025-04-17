#include "cleaner_system.hpp"

#include "TMCStepper.h"
#include "stepper_motor.hpp"

Cleaner::Cleaner()
    : jaw_rotation_motor_(
          JAW_ROTATION_CS_PIN,
          StepperMotor::TMC5160_PLUS_RSENSE,
          SW_MOSI,
          SW_MISO,
          SW_SCK),
      jaw_pos_motor_(
          JAW_POSITION_CS_PIN,
          StepperMotor::TMC5160_PLUS_RSENSE,
          SW_MOSI,
          SW_MISO,
          SW_SCK),
      clamp_motor_(CLAMP_CS_PIN, StepperMotor::TMC5160_PLUS_RSENSE, SW_MOSI, SW_MISO, SW_SCK),
      jaw_encoder_(ENCODER_CS_PIN)
{
    reset();
    jaw_rotation_motor_.setMaxSpeed(1000.0f);
    jaw_rotation_motor_.setAcceleration(1000.0f);
    jaw_pos_motor_.setMaxSpeed(1000.0f);
    jaw_pos_motor_.setAcceleration(1000.0f);
    clamp_motor_.setMaxSpeed(1000.0f);
    clamp_motor_.setAcceleration(1000.0f);
    jaw_encoder_.init();  // Initialize the encoder
}

Cleaner::~Cleaner() = default;

void Cleaner::home()
{
    // No operation
}

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

Cleaner::State Cleaner::getRealState()
{
    jaw_encoder_.update();  // Update the encoder state
    state_.jaw_rotation = jaw_encoder_.getAngle();
    return state_;
}

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    return EXIT_SUCCESS;
}
