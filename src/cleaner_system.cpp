#include "cleaner_system.hpp"

#include "BindArg.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "butterworth.hpp"
#include "cleaner_system_constants.hpp"
#include "pin_defs.hpp"
#include "stepper_motor.hpp"

Cleaner::Cleaner()
    : jaw_rotation_motor_(jawRotationCfg),
      jaw_pos_motor_(jawPosCfg),
      clamp_motor_(clampCfg),  // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false),
      encoderLowpassFilter(encoder_natural_coeffs_, encoder_forced_coeffs_),
      JawRotationPID(natural_coeffs_jaw_rotation_PID, forced_coeffs_jaw_rotation_PID),
      encoder_jaw_rotation_(ENCODER_JAW_ROTATION_PIN1, ENCODER_JAW_ROTATION_PIN2, IOExtender_),
      encoder_jaw_pos_(ENCODER_JAW_POSITION_PIN1, ENCODER_JAW_POSITION_PIN2, IOExtender_),
      encoder_clamp_(ENCODER_CLAMP_PIN1, ENCODER_CLAMP_PIN2, IOExtender_)

{
    // Add motors to the array
    motors[0] = &jaw_rotation_motor_;
    motors[1] = &jaw_pos_motor_;
    motors[2] = &clamp_motor_;

    // Create and set the lowpass filter for the encoder
    Butterworth<2, LOWPASS> encoderLowpass(500, 1.0f / RUN_RATE_HZ);
    encoder_natural_coeffs_ = encoderLowpass.getNaturalResponseCoefficients();
    encoder_forced_coeffs_  = encoderLowpass.getForcedResponseCoefficients();

    // Create and set the PID filter for the jaw rotation
    PIDControllerCoefficients jawRotationPID(0.1f, 0.01f, 0.01f, 1.0f / RUN_RATE_HZ);
    natural_coeffs_jaw_rotation_PID = jawRotationPID.getNaturalResponseCoefficients();
    forced_coeffs_jaw_rotation_PID  = jawRotationPID.getForcedResponseCoefficients();

    jaw_rotation_motor_.apply(JawRotationMotion);
    jaw_pos_motor_.apply(JawRotationMotion);
    clamp_motor_.apply(JawRotationMotion);

    jaw_rotation_motor_.apply(JawRotationElectrical);
    jaw_pos_motor_.apply(lowCurrent);  // gentler on the position motor
    clamp_motor_.apply(JawRotationElectrical);

    jaw_rotation_motor_.apply(JawRotationPhysical);

    reset();
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
    detachInterrupt(IO_EXTENDER_INT);
    attachInterrupt(
        IO_EXTENDER_INT,
        bindArgGateThisAllocate(&Cleaner::is_updatePCF8575_message, this),
        FALLING);


    // Initialize the pins
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
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

    ENCODER_BUTTONS[0].current = IOExtender_.read(ENCODER_JAW_ROTATION_BUTTON_PIN);
    ENCODER_BUTTONS[1].current = IOExtender_.read(ENCODER_JAW_POSITION_BUTTON_PIN);
    ENCODER_BUTTONS[2].current = IOExtender_.read(ENCODER_CLAMP_BUTTON_PIN);

    for (auto& button : ENCODER_BUTTONS)
    {
        toggleButton(button);
    }

    IOExtender_.write(ENCODER_JAW_ROTATION_SPEED_LED, ENCODER_JAW_ROTATION_SPEED_HIGH ? HIGH : LOW);
    IOExtender_.write(ENCODER_JAW_POSITION_SPEED_LED, ENCODER_JAW_POSITION_SPEED_HIGH ? HIGH : LOW);
    IOExtender_.write(ENCODER_CLAMP_SPEED_LED, ENCODER_CLAMP_SPEED_HIGH ? HIGH : LOW);
}

void Cleaner::run()
{
    // Runs at a fixed rate of RUN_RATE_HZ
    static unsigned long last_read_time        = 0;
    static const unsigned long min_interval_us = 1e6 / RUN_RATE_HZ;
    unsigned long now                          = micros();

    if (now - last_read_time >= min_interval_us)
    {
        updateRealState();

        // seems kinda strange but I think this will work
        State error = des_state_ - state_;
        // jaw_rotation_motor_.moveTo(jaw_rotation_motor_.currentPosition() + error.jaw_rotation);
        // jaw_pos_motor_.moveTo(jaw_pos_motor_.currentPosition() + error.jaw_pos);
        // clamp_motor_.moveTo(clamp_motor_.currentPosition() + error.clamp_pos);

        float jaw_rotation_speed = JawRotationPID.filterData(error.jaw_rotation) /
                                   jaw_rotation_motor_.getPhysicalParams().stepDistance;
        jaw_rotation_motor_.setSpeed(jaw_rotation_speed);

        // run all motors
        for (const auto& motor : motors)
        {
            motor->runSpeed();
        }

        last_read_time = now;
        State errortol{1e-3, 1e-1, 1e-1, false};
        if (error > errortol){
            Serial.print(SERIAL_ACK);
        }
    }
}

void Cleaner::home()
{
    reset(); // Reset the state to default values
    while (!digitalRead(LIMIT_SWITCH_PIN))
    {
        // While going, check for no estop just in case
        updateRealState();

        jaw_rotation_motor_.setSpeed(HOMING_SPEED / clamp_motor_.getPhysicalParams().stepDistance);
        jaw_rotation_motor_.runSpeed();
    }
    encoder_.setZeroPosition(encoder_.getRawRotation());
    state_.jaw_rotation = 0.0f;
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
    if (!digitalRead(ESTOP_PIN)){
        state_.is_Estopped = true;
        // oh no oh crap
        shutdown();
        return state_;
    }

    state_.jaw_rotation = encoderLowpassFilter.filterData(encoder_.getRotationInRadians());
    state_.jaw_pos      = jaw_pos_motor_.currentPositionUnits();
    state_.clamp_pos    = clamp_motor_.currentPositionUnits();

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

    /************** Rotary Encoder ***************/
    int delta_rot = encoder_jaw_rotation_.getRPM();
    int delta_pos = encoder_jaw_pos_.getRPM();
    int delta_cl  = encoder_clamp_.getRPM();

    // apply to desired state
    float rot_factor = ENCODER_JAW_ROTATION_SPEED_HIGH ? 1.0f : 0.1f;
    float pos_factor = ENCODER_JAW_POSITION_SPEED_HIGH ? 1.0f : 0.5f;
    float clp_factor = ENCODER_CLAMP_SPEED_HIGH ? 1.0f : 0.5f;

    des_state_.jaw_rotation += delta_rot * ENCODER_JAW_ROTATION_SENSITIVITY * rot_factor;
    des_state_.jaw_pos += delta_pos * ENCODER_JAW_POSITION_SENSITIVITY * pos_factor;
    des_state_.clamp_pos += delta_cl * ENCODER_CLAMP_SENSITIVITY * clp_factor;

    return des_state_;
}

void Cleaner::initializeManualMode()
{
    // Set the state to the current one to make everything relative to when switching
    des_state_ = state_;

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
        // Home command
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
            StepperMotor::ElectricalParams electricalParams =
                jaw_rotation_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.a * 1000;  // set current limit in mA
            jaw_rotation_motor_.apply(electricalParams);             // set current limit in mA
        }
        if (command.M906.y != 0)
        {
            StepperMotor::ElectricalParams electricalParams = jaw_pos_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.y * 1000;  // set current limit in mA
            jaw_pos_motor_.apply(electricalParams);                  // set current limit in mA
        }
        if (command.M906.c != 0)
        {
            StepperMotor::ElectricalParams electricalParams = clamp_motor_.getElectricalParams();
            electricalParams.runCurrent_mA = command.M906.c * 1000;  // set current limit in mA
            clamp_motor_.apply(electricalParams);                    // set current limit in mA
        }
    }
}

void Cleaner::printDriverDebug() { jaw_rotation_motor_.printDriverDebug(); }

int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    return EXIT_SUCCESS;
}