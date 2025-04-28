#include "cleaner_system.hpp"
#include "RotaryEncoder.h"

#include "TMCStepper.h"
#include "butterworth.hpp"
#include "stepper_motor.hpp"

Cleaner::Cleaner()
    : jaw_rotation_motor_(
          JAW_ROTATION_CS_PIN,
          StepperMotor::TMC5160_PLUS_RSENSE,
          SW_MOSI,
          SW_MISO,
          SW_SCK,
          255,
          "Jaw Rotation Motor"),
      jaw_pos_motor_(
          JAW_POSITION_CS_PIN,
          StepperMotor::TMC5160_PLUS_RSENSE,
          SW_MOSI,
          SW_MISO,
          SW_SCK,
          255,
          "Jaw Position Motor"),
      clamp_motor_(
          CLAMP_CS_PIN,
          StepperMotor::TMC5160_PLUS_RSENSE,
          255,
          "Clamp Motor"),  // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false),
      natural_coeffs_{},
      forced_coeffs_{},
      encoderFilter(natural_coeffs_, forced_coeffs_),
      encoder_jaw_rotation_(ENCODER_JAW_ROTATION_PIN1, ENCODER_JAW_ROTATION_PIN2, IOExtender_),
      encoder_jaw_pos_(ENCODER_JAW_POSITION_PIN1, ENCODER_JAW_POSITION_PIN2, IOExtender_),
      encoder_clamp_(ENCODER_CLAMP_PIN1, ENCODER_CLAMP_PIN2, IOExtender_)
{
    Wire.begin();
    IOExtender_.begin();
    
    motors[0] = &jaw_rotation_motor_;
    motors[1] = &jaw_pos_motor_;
    motors[2] = &clamp_motor_;

    Butterworth<2> encoderLowpass(500, 1.0f / ENCODER_READ_RATE_HZ);
    natural_coeffs_ = encoderLowpass.getNaturalResponseCoefficients();
    forced_coeffs_  = encoderLowpass.getForcedResponseCoefficients();

    encoder_.begin();
    reset();

    jaw_pos_motor_.setRunCurrent(0.5f * 1000);  // Set the run current to 0.5A

    jaw_rotation_motor_.setMaxSpeed(1000.0f);
    jaw_rotation_motor_.setAcceleration(1000.0f);
    jaw_pos_motor_.setMaxSpeed(1000.0f);
    jaw_pos_motor_.setAcceleration(1000.0f);
    clamp_motor_.setMaxSpeed(1000.0f);
    clamp_motor_.setAcceleration(1000.0f);
}

Cleaner::~Cleaner() = default;

void Cleaner::run()
{
    State error = des_state_ - state_;
    if (error.is_Estopped)
    {
        // oh no oh crap
        shutdown();
        return;
    }
    // seems kinda strange but I think this will work
    jaw_rotation_motor_.moveTo(jaw_rotation_motor_.currentPosition() + error.jaw_rotation);
    jaw_pos_motor_.moveTo(jaw_pos_motor_.currentPosition() + error.jaw_pos);
    clamp_motor_.moveTo(clamp_motor_.currentPosition() + error.clamp_pos);

    // run all motors
    for (const auto& motor : motors)
    {
        motor->run();
    }
}

void Cleaner::home()
{
    while (!digitalRead(LIMIT_SWITCH_PIN))
    {
        clamp_motor_.setSpeed(HOMING_SPEED);
        clamp_motor_.run();
    }
    state_.clamp_pos = 0.0f;
}

int Cleaner::reset()
{
    // Reset the state to default values
    memset(&state_, 0, sizeof(state_));
    memset(&des_state_, 0, sizeof(des_state_));

    // Initialize motors

    for (const auto& motor : motors)
    {
        if (motor->begin() != EXIT_SUCCESS)
        {
            Serial.printf("Failed to initialize %s motor.\n", motor->getName());
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

Cleaner::State Cleaner::getRealState()
{
    // Reads the encoder at the fixed rate
    static unsigned long last_read_time        = 0;
    static const unsigned long min_interval_us = 1e6 / ENCODER_READ_RATE_HZ;

    unsigned long now = micros();
    if (now - last_read_time >= min_interval_us)
    {
        state_.jaw_rotation = encoderFilter.filterData(encoder_.getRotationInRadians());
        last_read_time      = now;
    }

    // Get the values from accel stepper

    state_.jaw_pos = jaw_pos_motor_.currentPosition();

    return state_;
}

Cleaner::State Cleaner::getDesStateManual()
{
    /* Set the des_state_ to avoid having strange leftover interactions */
    des_state_ = state_;

    // Should have the behavior that if both are pressed, nothing changes.
    auto jog = [](bool forward, bool backward, float current, float error) -> float {
        if (forward) current += error;
        if (backward) current -= error;
        return current;
    };

    des_state_.jaw_pos =
        jog(digitalRead(JAW_JOG_FORWARD_PIN),
            digitalRead(JAW_JOG_BACKWARD_PIN),
            state_.jaw_pos,
            JAW_JOG_ERROR);

    des_state_.jaw_rotation =
        jog(digitalRead(JAW_ROTATION_JOG_FORWARD_PIN),
            digitalRead(JAW_ROTATION_JOG_BACKWARD_PIN),
            state_.jaw_rotation,
            JAW_ROTATION_JOG_ERROR);

    des_state_.clamp_pos =
        jog(digitalRead(CLAMP_JOG_FORWARD_PIN),
            digitalRead(CLAMP_JOG_BACKWARD_PIN),
            state_.clamp_pos,
            CLAMP_JOG_ERROR);

    return des_state_;
}

void Cleaner::processCommand(SerialReceiver::CommandMessage command)
{
    if (command.G0.received)
    {
        // Move command, modify the state to the desired state
        des_state_.jaw_rotation = command.G0.a;  // jaw rotation
        des_state_.jaw_pos      = command.G0.c;  // jaw position
        des_state_.clamp_pos    = command.G0.y;  // clamp position
        command.G0.received     = false;         // reset the received
    }
    if (command.G4.received)
    {
        // Dwell command, wait for a certain time
        delay(command.G4.val);        // kinda sucks it's blocking but good enough for now
        command.G4.received = false;  // reset the received
    }
    if (command.G28.received)
    {
        // Home command, reset the system state to the default state
        reset();
        home();
    }
    if (command.G90.received)
    {
        // Absolute positioning command, not yet implemented
        Serial.print("I ain't doin that\n");
    }
    if (command.M80.received)
    {
        // Set max speed command only if the speed command is not 0
        // Kinda bad since it won't let you actually set the speed to 0
        // TODO: fix this
        if (command.M80.a != 0)
        {
            jaw_rotation_motor_.setMaxSpeed(command.M80.a);
        }
        if (command.M80.y != 0)
        {
            jaw_pos_motor_.setMaxSpeed(command.M80.y);
        }
        if (command.M80.c != 0)
        {
            clamp_motor_.setMaxSpeed(command.M80.c);
        }
    }
    if (command.M17.received)
    {
        if (command.M17.a != 0)
        {
            jaw_rotation_motor_.setAcceleration(command.M80.a);
        }
        if (command.M17.y != 0)
        {
            jaw_pos_motor_.setAcceleration(command.M80.y);
        }
        if (command.M17.c != 0)
        {
            clamp_motor_.setAcceleration(command.M80.c);
        }
    }
    if (command.M906.received)
    {
        if (command.M906.a != 0)
        {
            jaw_rotation_motor_.setRunCurrent(command.M906.a * 1000);  // set current limit in mA
        }
        if (command.M906.y != 0)
        {
            jaw_pos_motor_.setRunCurrent(command.M906.y * 1000);  // set current limit in mA
        }
        if (command.M906.c != 0)
        {
            clamp_motor_.setRunCurrent(command.M906.c * 1000);  // set current limit in mA
        }
    }
    Serial.print(SERIAL_ACK);  // send the ack message back to the sender
}

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    return EXIT_SUCCESS;
}
