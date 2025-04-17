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
      clamp_motor_(CLAMP_CS_PIN, StepperMotor::TMC5160_PLUS_RSENSE), // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false)
{
    encoder_.begin();
    reset();

    jaw_pos_motor_.setRunCurrent(0.5f);  // Set the run current to 0.5A
    jaw_pos_motor_.setClampCurrent(0.5f);  // Set the clamp current to 0.5A
    
    jaw_rotation_motor_.setMaxSpeed(1000.0f);
    jaw_rotation_motor_.setAcceleration(1000.0f);
    jaw_pos_motor_.setMaxSpeed(1000.0f);
    jaw_pos_motor_.setAcceleration(1000.0f);
    clamp_motor_.setMaxSpeed(1000.0f);
    clamp_motor_.setAcceleration(1000.0f);
}

Cleaner::~Cleaner() = default;

void Cleaner::home()
{
    while(digitalRead(LIMIT_SWITCH_PIN))
    {
        clamp_motor_.setSpeed(HOMING_SPEED);
        clamp_motor_.run();
    }
    state_.clamp_pos = 0.0f;
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
    static unsigned long last_read_time = 0;
    static const unsigned long min_interval_us = 1e6 / ENCODER_READ_RATE_HZ;

    unsigned long now = micros();
    if (now - last_read_time >= min_interval_us)
    {
        state_.jaw_rotation = encoder_.getRotationInRadians();
        last_read_time = now;
    }

    return state_;
}

void Cleaner::processCommand(SerialReceiver::CommandMessage command)
{
    if (command.G0.received)
                    {
                        // Move command, modify the state to the desired state
                        des_state_.jaw_rotation = command.G0.a;  // jaw rotation
                        des_state_.jaw_pos      = command.G0.c;  // jaw position
                        des_state_.clamp_pos    = command.G0.y;  // clamp position
                        command.G0.received                        = false;  // reset the received
                    }
                    if (command.G4.received)
                    {
                        // Dwell command, wait for a certain time
                        delay(command.G4.val);  // kinda sucks it's blocking but good enough for now
                        command.G4.received = false;  // reset the received
                    }
                    if (command.G28.received)
                    {
                        // Home command, reset the system state to the default state
                        reset();
                        home();
                    }
}

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    return EXIT_SUCCESS;
}
