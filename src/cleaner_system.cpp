#include "cleaner_system.hpp"

#include "BindArg.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "butterworth.hpp"
#include "pin_defs.hpp"
#include "stepper_motor.hpp"

/* -------- static tables live in flash (constexpr) ------------------- */
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

/* reusable runtime presets */
constexpr StepperMotor::MotionParams fastMotion{50000, 100000};
constexpr StepperMotor::ElectricalParams defaultCurrent{600,16};
constexpr StepperMotor::ElectricalParams lowCurrent{500,16};

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

    jaw_rotation_motor_.apply(fastMotion);
    jaw_pos_motor_.apply(fastMotion);
    clamp_motor_.apply(fastMotion);

    jaw_rotation_motor_.apply(defaultCurrent);
    jaw_pos_motor_.apply(lowCurrent);  // gentler on the position motor
    clamp_motor_.apply(defaultCurrent);

    // JAW_ROTATION_DISTANCE = 1/(200.0f*jaw_rotation_motor_.getMicrosteps());
    JAW_ROTATION_DISTANCE = 1.0f / (200.0f * 16.0f);
    // JAW_ROTATION_DISTANCE = 100;
    jaw_rotation_motor_.setRotationDistance(JAW_ROTATION_DISTANCE);
}

Cleaner::~Cleaner() = default;

void Cleaner::is_updatePCF8575_message()
{
    // this is a message to be sent to the main loop
    updatePCF8575_flag = true;
}

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
    Wire.begin();  // Initialize I2C bus
    IOExtender_.begin();

    // Initialize the encoder
    encoder_.begin();

    // Register the interrupt for the PCF8575
    pinMode(IO_EXTENDER_INT, INPUT_PULLUP);
    // now the ISR service is up—this will succeed
    attachInterrupt(IO_EXTENDER_INT, bindArgGateThisAllocate(&Cleaner::is_updatePCF8575_message, this), FALLING);

    return EXIT_SUCCESS;
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

    // Check if the button is currently pressed (state is low) and if it was high last time
    // If this condition is met, toggle the corresponding speed variable to
    // switch between high and low speed modes.
    Serial.println(current_jaw_rotation_button_state);
    if (!current_jaw_rotation_button_state && last_jaw_rotation_button_state)
    {
        ENCODER_JAW_ROTATION_SPEED_HIGH = !ENCODER_JAW_ROTATION_SPEED_HIGH;
    }
    if (!current_jaw_position_button_state && last_jaw_position_button_state)
    {
        ENCODER_JAW_POSITION_SPEED_HIGH = !ENCODER_JAW_POSITION_SPEED_HIGH;
    }
    if (!current_clamp_button_state && last_clamp_button_state)
    {
        ENCODER_CLAMP_SPEED_HIGH = !ENCODER_CLAMP_SPEED_HIGH;
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

    // seems kinda strange but I think this will work
    State error = des_state_ - state_;
    // jaw_rotation_motor_.moveTo(jaw_rotation_motor_.currentPosition() + error.jaw_rotation);
    // jaw_pos_motor_.moveTo(jaw_pos_motor_.currentPosition() + error.jaw_pos);
    // clamp_motor_.moveTo(clamp_motor_.currentPosition() + error.clamp_pos);

    jaw_rotation_motor_.moveToUnits(des_state_.jaw_rotation);

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

    // Get the values from accel stepper
    // TODO: NOT DO THIS LATER IT OVERWRITES FOR DEBUGGING
    state_.jaw_rotation = jaw_rotation_motor_.currentPositionUnits();

    return state_;
}


Cleaner::State Cleaner::updateDesStateManual()
{
  if (updatePCF8575_flag)
  {
    updatePCF8575_flag = false;
    updatePCF8575();
  }

  // 1) read raw counts
  int cur_jaw_rot = encoder_jaw_rotation_.getPosition();
  int cur_jaw_pos = encoder_jaw_pos_.getPosition();
  int cur_clamp   = encoder_clamp_.getPosition();

  // 2) compute deltas
  int  delta_rot = cur_jaw_rot - last_enc_jaw_rot_;
  int  delta_pos = cur_jaw_pos - last_enc_jaw_pos_;
  int  delta_cl  = cur_clamp   - last_enc_clamp_;

  // 3) apply to desired state
  float rot_factor = ENCODER_JAW_ROTATION_SPEED_HIGH ? 1.0f : 0.1f;
  float pos_factor = ENCODER_JAW_POSITION_SPEED_HIGH ? 1.0f : 0.5f;
  float clp_factor = ENCODER_CLAMP_SPEED_HIGH       ? 1.0f : 0.5f;

  des_state_.jaw_rotation += delta_rot * ENCODER_JAW_ROTATION_SENSITIVITY * rot_factor;
  des_state_.jaw_pos      += delta_pos * ENCODER_JAW_POSITION_SENSITIVITY * pos_factor;
  des_state_.clamp_pos    += delta_cl  * ENCODER_CLAMP_SENSITIVITY       * clp_factor;

  // 4) remember raw values for next time
  last_enc_jaw_rot_ = cur_jaw_rot;
  last_enc_jaw_pos_ = cur_jaw_pos;
  last_enc_clamp_   = cur_clamp;

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