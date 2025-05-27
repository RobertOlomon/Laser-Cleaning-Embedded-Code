#include "cleaner_system.hpp"

#include "BindArg.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "butterworth.hpp"
#include "cleaner_system_constants.hpp"
#include "macros.hpp"
#include "pin_defs.hpp"
#include "stepper_motor.hpp"

Cleaner::Cleaner()
    : jaw_rotation_motor_(jawRotationCfg),
      jaw_pos_motor_(jawPosCfg),
      clamp_motor_(clampCfg),  // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false),
      encoderLowpassFilter(filter::butterworth<2, filter::LOWPASS>(50.0f, 1.0f / RUN_RATE_HZ)),
      JawRotationPID(controller::PIDControllerCoefficients(10.0f, 0.0f, 0.0f, 1.0f / RUN_RATE_HZ)),
      JawPositionPID(controller::PIDControllerCoefficients(10.0f, 0.0f, 0.0f, 1.0f / RUN_RATE_HZ)),
      ClampPID(controller::PIDControllerCoefficients(10.0f, 0.0f, 0.0f, 1.0f / RUN_RATE_HZ)),
      encoder_jaw_rotation_(
          ENCODER_JAW_ROTATION_PIN1,
          ENCODER_JAW_ROTATION_PIN2,
          [&](int pin) { return IOExtender_.readNoUpdate(pin); }),
      encoder_jaw_pos_(
          ENCODER_JAW_POSITION_PIN1,
          ENCODER_JAW_POSITION_PIN2,
          [&](int pin) { return IOExtender_.readNoUpdate(pin); }),
      encoder_clamp_(ENCODER_CLAMP_PIN1, ENCODER_CLAMP_PIN2, [&](int pin) {
          return IOExtender_.readNoUpdate(pin);
      })
{
    // Add motors to the array
    motors[0] = &jaw_rotation_motor_;
    motors[1] = &clamp_motor_;
    motors[2] = &jaw_pos_motor_;

    jaw_rotation_motor_.apply(JawRotationMotion);
    jaw_pos_motor_.apply(JawPositionMotion);
    clamp_motor_.apply(ClampMotion);

    jaw_rotation_motor_.apply(JawRotationElectrical);
    jaw_pos_motor_.apply(JawPositionElectrical);  // gentler on the position motor
    clamp_motor_.apply(clampElectrical);

    jaw_rotation_motor_.apply(JawRotationPhysical);
    jaw_pos_motor_.apply(JawPositionPhysical);
    clamp_motor_.apply(clampPhysical);

    reset();
}

Cleaner::~Cleaner() = default;

void Cleaner::PCFMessageRec()
{
    // this is a message to be sent to the main loop
    updatePCF8575_flag = true;
}
int Cleaner::begin()
{
    // Initialize the communication bus
    Wire.begin();  // Initialize I2C bus
    Wire.setClock(400000);

    IOExtender_.begin();
    
    // Initialize the motors
    for (auto* motor : motors)
    {
        if (motor->begin() != EXIT_SUCCESS)
        {
            Serial.printf("Failed to initialize %s motor.\n", motor->getName());
            return EXIT_FAILURE;
        }
    }
    // Initialize the encoder
    encoder_.begin();

    if (IO_EXTENDER_INT != 255)
    {
        // Register the interrupt for the PCF8575
        pinMode(IO_EXTENDER_INT, INPUT_PULLUP);
        detachInterrupt(IO_EXTENDER_INT);
        attachInterrupt(
            IO_EXTENDER_INT,
            bindArgGateThisAllocate(&Cleaner::PCFMessageRec, this),
            CHANGE);
    }
    // Initialize the pins
    pinMode(LIMIT_SWITCH_PIN_JAW_ROTATION, INPUT_PULLUP);
    pinMode(ESTOP_PIN, INPUT_PULLUP);

    return EXIT_SUCCESS;
}

// This function is called in the main loop whenever the interrupt is triggered
void Cleaner::updatePCF8575()
{
    // update the encoders
    encoder_clamp_.tick();
    encoder_jaw_pos_.tick();
    encoder_jaw_rotation_.tick();

    // read current state

    ENCODER_BUTTONS[0].current = IOExtender_.readNoUpdate(ENCODER_JAW_ROTATION_BUTTON_PIN);
    ENCODER_BUTTONS[1].current = IOExtender_.readNoUpdate(ENCODER_JAW_POSITION_BUTTON_PIN);
    ENCODER_BUTTONS[2].current = IOExtender_.readNoUpdate(ENCODER_CLAMP_BUTTON_PIN);

    for (auto& button : ENCODER_BUTTONS)
    {
        toggleButton(button);
    }

    des_state_.is_Brake = IOExtender_.readNoUpdate(ROLLER_BRAKE_PIN) == LOW;

    IOExtender_.writeNoUpdate(
        ENCODER_JAW_ROTATION_SPEED_LED,
        ENCODER_JAW_ROTATION_SPEED_HIGH ? LOW : HIGH);
    IOExtender_.writeNoUpdate(
        ENCODER_JAW_POSITION_SPEED_LED,
        ENCODER_JAW_POSITION_SPEED_HIGH ? LOW : HIGH);
    IOExtender_.writeNoUpdate(ENCODER_CLAMP_SPEED_LED, ENCODER_CLAMP_SPEED_HIGH ? LOW : HIGH);

    AutoMode = IOExtender_.readNoUpdate(MODE_PIN) == LOW;

    IOExtender_.update();
}

void Cleaner::run()
{
    if (state_.is_Estopped)
    {
        return;
    }
    // Runs at a fixed rate of RUN_RATE_HZ
    static unsigned long last_read_time        = 0;
    static const unsigned long min_interval_us = 1e6 / RUN_RATE_HZ;
    unsigned long now                          = micros();

    if (now - last_read_time >= min_interval_us)
    {
        updateRealState();

        // seems kinda strange but I think this will work
        State error = des_state_ - state_;

        float jaw_rotation_speed = JawRotationPID.filterData(error.jaw_rotation);
        jaw_rotation_motor_.setSpeedUnits(jaw_rotation_speed);

        float jaw_pos_speed = JawPositionPID.filterData(error.jaw_pos);
        jaw_pos_motor_.setSpeedUnits(jaw_pos_speed);

        float clamp_speed = ClampPID.filterData(error.clamp_pos);
        clamp_motor_.setSpeedUnits(clamp_speed);

        if (error.is_Brake)
        {
            // Invert what's currently on the brake
            digitalWrite(ROLLER_BRAKE_PIN, !digitalRead(ROLLER_BRAKE_PIN));
        }

        last_read_time = now;
        State errortol{1e-3, 1e-1, 1e-1, false, false};

        // if (error < errortol)
        // {
        //     Serial.print(SERIAL_ACK);
        // }
    }
    // run all motors
    for (const auto& motor : motors)
    {
        motor->runSpeed();
        // motor->dumpDRV(motor->driver(), motor->getName());
    }
}

// use on command, not allowed to modify
void Cleaner::home(SerialReceiver::CommandMessage command)
{
    // reset the states based on what is received
    if (command.M80.a > 0)
    {
        state_.jaw_rotation     = 0.0f;
        des_state_.jaw_rotation = 0.0f;
        // enter a loop until we hit the limit switch
        while (!digitalRead(LIMIT_SWITCH_PIN_JAW_ROTATION))
        {
            jaw_rotation_motor_.setSpeedUnits(HOMING_SPEED);
            jaw_rotation_motor_.runSpeed();
        }
        // encoder_.setZeroPosition(encoder_.getRawRotation());
        state_.jaw_rotation = 0.0f;
    }

    if (command.M80.y > 0)
    {
        state_.jaw_pos     = 0.0f;
        des_state_.jaw_pos = 0.0f;
    }

    if (command.M80.c > 0)
    {
        state_.clamp_pos     = 0.0f;
        des_state_.clamp_pos = 0.0f;
    }
}

int Cleaner::reset()
{
    // Reset the state to default values
    memset(&state_, 0, sizeof(state_));
    memset(&des_state_, 0, sizeof(des_state_));

    return EXIT_SUCCESS;
}

void Cleaner::stop()
{
    // Stop all motors
    for (auto* motor : motors)
    {
        motor->stop();
        motor->run();
    }
}

Cleaner::State Cleaner::updateRealState()
{
    if (ESTOP_PIN != 255 && !digitalRead(ESTOP_PIN))
    {
        state_.is_Estopped = true;
        // oh no oh crap
        shutdown();
        return state_;
    }

    state_.jaw_rotation = encoderLowpassFilter.filterData(-encoder_.getRotationUnwrappedInRadians());
    // state_.jaw_rotation = -encoder_.getRotationUnwrappedInRadians();
    // state_.jaw_rotation = jaw_rotation_motor_.currentPositionUnits();
    state_.jaw_pos   = jaw_pos_motor_.currentPositionUnits();
    state_.clamp_pos = clamp_motor_.currentPositionUnits();

    state_.is_Brake = digitalRead(ROLLER_BRAKE_PIN);

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
    int delta_rot = cur_jaw_rot - last_enc_jaw_rot_;
    int delta_pos = cur_jaw_pos - last_enc_jaw_pos_;
    int delta_cl  = cur_clamp - last_enc_clamp_;

    // 3) apply to desired state
    float rot_factor = ENCODER_JAW_ROTATION_SPEED_HIGH ? 1.0f : 0.05f;
    float pos_factor = ENCODER_JAW_POSITION_SPEED_HIGH ? 1.0f : 0.1f;
    float clp_factor = ENCODER_CLAMP_SPEED_HIGH ? 1.0f : 0.1f;

    des_state_.jaw_rotation += delta_rot * ENCODER_JAW_ROTATION_SENSITIVITY * rot_factor;
    des_state_.jaw_pos += delta_pos * ENCODER_JAW_POSITION_SENSITIVITY * pos_factor;
    des_state_.clamp_pos += delta_cl * ENCODER_CLAMP_SENSITIVITY * clp_factor;

    // 4) remember raw values for next time
    last_enc_jaw_rot_ = cur_jaw_rot;
    last_enc_jaw_pos_ = cur_jaw_pos;
    last_enc_clamp_   = cur_clamp;

    return des_state_;
}

// Used to see if we need to switch modes
void Cleaner::updateModeAuto()
{
    if (updatePCF8575_flag)
    {
        updatePCF8575_flag = false;
        updatePCF8575();
    }
}

void Cleaner::initializeManualMode()
{
    updateRealState();  // Update the real state to get the current position
    updateDesStateManual();

    // Set the state to the current one to make everything relative to when switching
    des_state_ = state_;
    des_state_.jaw_rotation = -encoder_.getRotationUnwrappedInRadians();
    encoderLowpassFilter.fill(des_state_.jaw_rotation);
}

void Cleaner::initializeAutoMode()
{
}

void Cleaner::processCommand(SerialReceiver::CommandMessage command)
{
    if (command.G0.received)
    {
        // Move command, modify the state to the desired state
        command.G0.received     = false;         // reset the received
        des_state_.jaw_rotation = command.G0.a;  // jaw rotation
        des_state_.jaw_pos      = command.G0.y;  // jaw position
        des_state_.clamp_pos    = command.G0.c;  // clamp position
    }
    if (command.G4.received)
    {
        // Dwell command, wait for a certain time
        command.G4.received = false;  // reset the received
        delay(command.G4.val);        // kinda sucks it's blocking but good enough for now
    }
    if (command.G28.received)
    {
        // Home command
        command.G28.received = false;
        home(command);
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
        // TODO: change the clamping to be relative of the potentiometer value
        if (command.M906.a != 0)
        {
            StepperMotor::ElectricalParams electricalParams =
                jaw_rotation_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.a * 1000;
            jaw_rotation_motor_.apply(electricalParams);  // set scurrent limit in mA
        }
        if (command.M906.y != 0)
        {
            StepperMotor::ElectricalParams electricalParams = jaw_pos_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.y * 1000;
            jaw_pos_motor_.apply(electricalParams);  // set current limit in mA
        }
        if (command.M906.c != 0)
        {
            StepperMotor::ElectricalParams electricalParams = clamp_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.c * 1000;
            clamp_motor_.apply(electricalParams);  // set current limit in mA
        }
    }
}

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    state_.is_Estopped = true;

    return EXIT_SUCCESS;
}