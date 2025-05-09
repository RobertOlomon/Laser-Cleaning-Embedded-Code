#include "cleaner_system.hpp"

#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "butterworth.hpp"
#include "pin_defs.hpp"
#include "stepper_motor.hpp"

volatile bool updatePCF8575_flag = false;  // Flag to indicate if the PCF8575 has been updated

/* -------- static tables live in flash (constexpr) ------------------- */
constexpr StepperMotor::StaticConfig jawRotationCfg{
    /* pins */ {JAW_ROTATION_CS_PIN, 8, 9, 255},
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

/* reusable runtime presets */
constexpr StepperMotor::MotionParams fastMotion{1000.0f, 1000.0f};
constexpr StepperMotor::ElectricalParams defaultCurrent{1000};  // 1 A
constexpr StepperMotor::ElectricalParams lowCurrent{500};       // 0.5 A

Cleaner::Cleaner()
    : jaw_rotation_motor_(jawRotationCfg),
      jaw_pos_motor_(jawPosCfg),
      clamp_motor_(clampCfg),  // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false),
      natural_coeffs_{},
      forced_coeffs_{},
      encoderFilter(natural_coeffs_, forced_coeffs_),
      encoder_jaw_rotation_(ENCODER_JAW_ROTATION_PIN1, ENCODER_JAW_ROTATION_PIN2, IOExtender_),
      encoder_jaw_pos_(ENCODER_JAW_POSITION_PIN1, ENCODER_JAW_POSITION_PIN2, IOExtender_),
      encoder_clamp_(ENCODER_CLAMP_PIN1, ENCODER_CLAMP_PIN2, IOExtender_)
{
    // Add motors to the array
    motors[0] = &jaw_rotation_motor_;
    motors[1] = &jaw_pos_motor_;
    motors[2] = &clamp_motor_;

    // Create and set the lowpass filter for the encoder
    Butterworth<2, LOWPASS> encoderLowpass(500, 1.0f / ENCODER_READ_RATE_HZ);
    natural_coeffs_ = encoderLowpass.getNaturalResponseCoefficients();
    forced_coeffs_  = encoderLowpass.getForcedResponseCoefficients();

    noInterrupts();
    attachInterrupt(
        digitalPinToInterrupt(IO_EXTENDER_INT),
        is_updatePCF8575_message,
        FALLING);  // Attach interrupt to ESTOP_PIN
    interrupts();

    jaw_rotation_motor_.apply(fastMotion);
    jaw_pos_motor_.apply(fastMotion);
    clamp_motor_.apply(fastMotion);

    jaw_rotation_motor_.apply(defaultCurrent);
    jaw_pos_motor_.apply(lowCurrent);  // gentler on the position motor
    clamp_motor_.apply(defaultCurrent);

    reset();
}

Cleaner::~Cleaner() = default;

int Cleaner::begin()
{
    // Initialize the motors
    for (auto* motor : motors)
    {
        if (motor->begin() != EXIT_SUCCESS)
        {
            Serial.printf("Failed to initialize %s motor.\n", motor->getName());
            return EXIT_FAILURE;
        }
    }
    // Initialize the communication bus
    SPI.begin();
    Wire.begin(); // Initialize I2C bus
    IOExtender_.begin();

    // Initialize the encoder
    encoder_.begin();

    return EXIT_SUCCESS;
}

void Cleaner::is_updatePCF8575_message()
{
    // this is an interrupt to be ran whenever the int pin on the PCF goes
    updatePCF8575_flag = true;
}

// this is an interrupt to be ran whenever the int pin on the PCF goes
void Cleaner::updatePCF8575()
{
    // update the encoders
    encoder_clamp_.tick();
    encoder_jaw_pos_.tick();
    encoder_jaw_rotation_.tick();

    // read current state
    bool current_jaw_rotation_button_state = IOExtender_.read(ENCODER_JAW_ROTATION_BUTTON_PIN);
    bool current_jaw_position_button_state = IOExtender_.read(ENCODER_JAW_POSITION_BUTTON_PIN);
    bool current_clamp_button_state        = IOExtender_.read(ENCODER_CLAMP_BUTTON_PIN);

    // Check if the button is currently pressed (state is high) and was not pressed previously
    // (state was false). If this condition is met, toggle the corresponding speed variable to
    // switch between high and low speed modes.
    if (current_jaw_rotation_button_state && !last_jaw_rotation_button_state)
    {
        ENCODER_CLAMP_SPEED_HIGH = !ENCODER_CLAMP_SPEED_HIGH;
    }
    if (current_jaw_position_button_state && !last_jaw_position_button_state)
    {
        ENCODER_JAW_POSITION_SPEED_HIGH = !ENCODER_JAW_POSITION_SPEED_HIGH;
    }
    if (current_clamp_button_state && !last_clamp_button_state)
    {
        ENCODER_JAW_ROTATION_SPEED_HIGH = !ENCODER_JAW_ROTATION_SPEED_HIGH;
    }

    // update the last button states
    last_jaw_rotation_button_state = current_jaw_rotation_button_state;
    last_jaw_position_button_state = current_jaw_position_button_state;
    last_clamp_button_state        = current_clamp_button_state;
}

void Cleaner::run()
{
    if (state_.is_Estopped)
    {
        // oh no oh crap
        shutdown();
        return;
    }
    // update the flag when triggered
    if (updatePCF8575_flag)
    {
        updatePCF8575();
        updatePCF8575_flag = false;
    }

    // seems kinda strange but I think this will work
    State error = des_state_ - state_;
    // jaw_rotation_motor_.moveTo(jaw_rotation_motor_.currentPosition() + error.jaw_rotation);
    // jaw_pos_motor_.moveTo(jaw_pos_motor_.currentPosition() + error.jaw_pos);
    // clamp_motor_.moveTo(clamp_motor_.currentPosition() + error.clamp_pos);

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

    // Get the values from the encoders
    if (updatePCF8575_flag)
    {
        updatePCF8575_flag = false;
        encoder_jaw_rotation_.tick();
        encoder_jaw_pos_.tick();
        encoder_clamp_.tick();
    }
    // Get the values from accel stepper

    state_.jaw_pos = jaw_pos_motor_.currentPosition();

    return state_;
}

Cleaner::State Cleaner::getDesStateManual()
{
    des_state_.jaw_pos = encoder_jaw_pos_.getPosition() * ENCODER_JAW_POSITION_SENSITIVITY *
                         (ENCODER_JAW_POSITION_SPEED_HIGH ? 1 : 0.5f);

    des_state_.jaw_rotation = encoder_jaw_rotation_.getPosition() *
                              ENCODER_JAW_ROTATION_SENSITIVITY *
                              (ENCODER_JAW_ROTATION_SPEED_HIGH ? 1 : 0.5f);

    des_state_.clamp_pos = encoder_clamp_.getPosition() * ENCODER_CLAMP_SENSITIVITY *
                           (ENCODER_CLAMP_SPEED_HIGH ? 1 : 0.5f);

    return des_state_;
}

void Cleaner::initializeManualMode()
{
    // Set the state to the current one to make everything relative to when switching
    des_state_ = state_;

    // Reset the drive knob encoder position to 0 so it doesn't freak out
    encoder_jaw_rotation_.setPosition(0);
    encoder_jaw_pos_.setPosition(0);
    encoder_clamp_.setPosition(0);

    begin();  // Initialize the motors and communication bus
}

void Cleaner::initializeAutoMode()
{
    begin();  // Initialize the motors and communication bus
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
            jaw_rotation_motor_.setAcceleration(command.M17.a);
        }
        if (command.M17.y != 0)
        {
            jaw_pos_motor_.setAcceleration(command.M17.y);
        }
        if (command.M17.c != 0)
        {
            clamp_motor_.setAcceleration(command.M17.c);
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

void Cleaner::printDriverDebug() { jaw_rotation_motor_.printDriverDebug(); }

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    return EXIT_SUCCESS;
}