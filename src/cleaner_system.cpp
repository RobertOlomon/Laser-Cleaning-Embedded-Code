#include "cleaner_system.hpp"

#include <cmath>

#include "BindArg.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "butterworth.hpp"
#include "cleaner_system_constants.hpp"
#include "macros.hpp"
#include "pin_defs.hpp"
#include "stepper_motor.hpp"

/**
 * @brief Constructs a Cleaner object and initializes all member components.
 *
 * This constructor initializes the jaw rotation, jaw position, and clamp motors with their respective configurations.
 * It also sets up the encoder for clamp position, lowpass filters for clamp and jaw encoder signals, and the PID controller for the clamp.
 * The jaw rotation, jaw position, and clamp encoders are initialized with their respective pins and IO extender read callbacks.
 * Motors are added to the internal array for management, and default motion, electrical, and physical profiles are applied to each motor.
 * Finally, the system is reset to its initial state.
 */
Cleaner::Cleaner()
    : jaw_rotation_motor_(jawRotationCfg),
      jaw_pos_motor_(jawPosCfg),
      clamp_motor_(clampCfg),  // Assume hardware SPI for now
      encoder_(ENCODER_CS_PIN, false),
      clampLowpassFilter(filter::butterworth<2, filter::LOWPASS>(50.0f, 1.0f / RUN_RATE_HZ)),
      jawEncoderLowpassFilter(filter::butterworth<2, filter::LOWPASS>(300.0f, 1.0f / RUN_RATE_HZ)),
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
    motors[1] = &jaw_pos_motor_;
    motors[2] = &clamp_motor_;

    jaw_rotation_motor_.apply(JawRotationMotion);
    jaw_pos_motor_.apply(JawPositionMotion);
    clamp_motor_.apply(ClampMotion);

    jaw_rotation_motor_.apply(JawRotationElectrical);
    jaw_pos_motor_.apply(JawPositionElectrical);
    clamp_motor_.apply(clampElectrical);

    jaw_rotation_motor_.apply(JawRotationPhysical);
    jaw_pos_motor_.apply(JawPositionPhysical);
    clamp_motor_.apply(clampPhysical);

    reset();
}

Cleaner::~Cleaner() = default;

/**
 * @brief Initializes the Cleaner system and its components. Designed to be ran once at start
 *
 * This function performs the following initialization steps:
 * - Initializes the I2C communication bus.
 * - Initializes all motors in the system. If any motor fails to initialize,
 *   an error message is printed to the serial port and the function returns failure.
 * - Initializes the encoder.
 * - Registers an interrupt handler for the IO extender if the interrupt pin is defined.
 * - Initializes the IO extender.
 * - Configures the necessary GPIO pins for limit switch, emergency stop, and roll brake.
 *
 * @return int Returns EXIT_SUCCESS on successful initialization, or EXIT_FAILURE if any component fails to initialize.
 */
int Cleaner::begin()
{
    // Initialize the communication bus
    Wire.begin();  // Initialize I2C bus
    Wire.setClock(400000);

    // Initialize the motors
    for (auto* motor : motors)
    {
        if (motor->begin() != EXIT_SUCCESS)
        {
            std::string errorMessage =
                std::string("Failed to initialize ") + motor->getName() + " motor.\n";
            if (Serial.availableForWrite() > strlen(errorMessage.c_str()))
            {
                Serial.print(errorMessage.c_str());
            }
            return EXIT_FAILURE;
        }
    }

    // Initialize the encoder
    encoder_.begin();

    // Register the interrupt for the PCF8575
    if (IO_EXTENDER_INT != 255)
    {
        pinMode(IO_EXTENDER_INT, INPUT_PULLUP);
        detachInterrupt(IO_EXTENDER_INT);
        attachInterrupt(
            IO_EXTENDER_INT,
            bindArgGateThisAllocate(&Cleaner::PCFMessageRec, this),
            CHANGE);
    }
    // Initialize the IO extender
    IOExtender_.begin();

    // Initialize the pins
    pinMode(LIMIT_SWITCH_PIN_JAW_ROTATION, INPUT_PULLUP);
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(ROLL_BRAKE_REAL_PIN, OUTPUT);

    return EXIT_SUCCESS;
}

/**
 * @brief Executes the ~300kHz control loop for the Cleaner system.
 *
 * This function performs the following actions:
 * - Checks if the system is in an emergency stop (E-Stop) state and exits early if so.
 * - Executes control logic at a fixed rate using the DO_EVERY macro.
 * - Iterates through all motors, running each one.
 * - For the clamp motor, synchronizes its speed with the jaw rotation motor and adjusts
 *   the speed to account for any additional required movement based on error.
 * 
 * The function ensures coordinated movement between the jaw rotation and clamp motors,
 * and enforces speed limits for safe operation.
 */
void Cleaner::run()
{
    // Do not run if we're E-Stopped
    if (state_.is_Estopped)
    {
        return;
    }

    DO_EVERY(1.0f / RUN_RATE_HZ, runControl());
    // run all motors
    for (const auto& motor : motors)
    {
        /* If we're moving the jaw rotation, sync the clamp motor to it
         * and add any additional speed needed to move the clamp when it
         * too has error
         */
        clamp_motor_.setSpeed(limit_val(
            jaw_rotation_motor_.speed() * 2 +
                desired_clamp_speed / clamp_motor_.getPhysicalParams().stepDistance,
            -clamp_motor_.maxSpeed(),
            clamp_motor_.maxSpeed()));

        motor->run();
    }

    if (command_in_progress_ && atTarget())
    {
        Serial.println(SERIAL_ACK);
        command_in_progress_ = false;
    }
}
/**
 * @brief Runs the 1kHz control loop
 * 
 */
void Cleaner::runControl()
{
    updateRealState();

    State error = des_state_ - state_;
    jaw_rotation_motor_.moveToUnits(des_state_.jaw_rotation);

    jaw_pos_motor_.moveToUnits(des_state_.jaw_pos);

    const float percentOfMax = .25f;
    desired_clamp_speed      = limit_val(
        clampLowpassFilter.filterData(ClampPID.filterData(error.clamp_pos)),
        -clamp_motor_.maxSpeedUnits() * percentOfMax,
        clamp_motor_.maxSpeedUnits() * percentOfMax);

    if (abs(desired_clamp_speed) < 0.0f)
    {
        desired_clamp_speed = 0;
    }

    if (error.is_Brake)
    {
        // Invert what's currently on the brake
        digitalWrite(ROLL_BRAKE_REAL_PIN, !digitalRead(ROLL_BRAKE_REAL_PIN));
    }
}

/**
 * @brief Updates and returns the real-time state of the Cleaner system.
 *
 * This function checks the emergency stop (ESTOP) pin and, if triggered,
 * sets the system to an emergency stopped state and initiates shutdown.
 * It then updates the jaw rotation and clamp position based on the current
 * positions of their respective motors. The clamp position is calculated
 * relative to the jaw rotation. The function also updates the brake state
 * by reading the corresponding hardware pin.
 *
 * @return Cleaner::State The updated state of the Cleaner system, reflecting
 *         the latest hardware readings and safety status.
 */
Cleaner::State Cleaner::updateRealState()
{
    if (ESTOP_PIN != 255 && !digitalRead(ESTOP_PIN))
    {
        state_.is_Estopped = true;
        // oh no oh crap
        shutdown();
        return state_;
    }

    state_.jaw_rotation = jaw_rotation_motor_.currentPositionUnits();
    state_.clamp_pos    = clamp_motor_.currentPositionUnits() -
                       state_.jaw_rotation;  // clamp is relative to jaw rotation

    state_.is_Brake = digitalRead(ROLL_BRAKE_REAL_PIN);

    return state_;
}

/**
 * @brief Updates the desired state of the cleaner system in manual mode based on encoder inputs.
 *
 * This function reads the current positions from the jaw rotation, jaw position, and clamp encoders,
 * computes the deltas since the last update, and applies these changes to the desired state using
 * configurable sensitivity and speed factors. It also updates the brake state based on the current
 * brake switch status. The function ensures that encoder values are stored for the next update cycle.
 *
 * @return Cleaner::State The updated desired state of the cleaner system.
 */
Cleaner::State Cleaner::updateDesStateManual()
{
    DO_EVERY(.05, updatePCF8575());
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

    des_state_.is_Brake = breakSwitchedOn;

    return des_state_;
}

/**
 * @brief Used to see if we need to switch
 * from auto mode
 * 
 */
void Cleaner::updateModeAuto()
{
    DO_EVERY(.05, updatePCF8575());
    if (updatePCF8575_flag)
    {
        updatePCF8575_flag = false;
        updatePCF8575();
    }
}

/**
 * @brief Starts the manual mode by setting des_state_
 * to make all the jog dials relative to the position you're
 * in when hitting the switch
 */
void Cleaner::initializeManualMode()
{
    updateRealState();  // Update the real state to get the current position
    updateDesStateManual();
    ClampPID.reset();
    des_state_ = state_;
}

/**
 * @brief Initialises the auto mode by resetting the system state
 * and reseting the receiver line to prevent the system from running to
 * the last received message and wait for a new one.
 *
 * @param receiver Handle to the serial receiver being used
 */
void Cleaner::initializeAutoMode(SerialReceiverTransmitter& receiver)
{
    reset();
    receiver.reset();
    ClampPID.reset();
    updateRealState();
}

/**
 * @brief ISR for the PCF8575
 */
void Cleaner::PCFMessageRec() { updatePCF8575_flag = true; }

// This function is called in the main loop whenever the interrupt is triggered
/**
 * @brief Updates the state of the PCF8575 I/O expander and related system components.
 *
 * This function performs the following tasks:
 * - Updates the state of the encoder objects for clamp, jaw position, and jaw rotation.
 * - Reads the raw state of the encoder dial buttons from the I/O expander and updates their state.
 * - Toggles the button state for each encoder button.
 * - Reads the state of the roll brake switch and updates the corresponding flag.
 * - Updates the LED indicators for encoder speed based on the current speed settings.
 * - Reads the mode pin to determine if the system is in auto mode.
 * - Sends the accumulated I2C commands to the PCF8575 I/O expander.
 *
 * This function should be if the updatePCF8575_flag is true
 */
void Cleaner::updatePCF8575()
{
    // update the encoders
    encoder_clamp_.tick();
    encoder_jaw_pos_.tick();
    encoder_jaw_rotation_.tick();

    /* Encoder Dial button tracking */
    ENCODER_BUTTONS[0].rawState = IOExtender_.readNoUpdate(ENCODER_JAW_ROTATION_BUTTON_PIN);
    ENCODER_BUTTONS[1].rawState = IOExtender_.readNoUpdate(ENCODER_JAW_POSITION_BUTTON_PIN);
    ENCODER_BUTTONS[2].rawState = IOExtender_.readNoUpdate(ENCODER_CLAMP_BUTTON_PIN);

    for (auto& button : ENCODER_BUTTONS)
    {
        toggleButton(button);
    }

    breakSwitchedOn = IOExtender_.readNoUpdate(ROLL_BRAKE_BUT_PIN) == HIGH;

    // Update the LEDS with the current button state
    IOExtender_.writeNoUpdate(
        ENCODER_JAW_ROTATION_SPEED_LED,
        ENCODER_JAW_ROTATION_SPEED_HIGH ? LOW : HIGH);
    IOExtender_.writeNoUpdate(
        ENCODER_JAW_POSITION_SPEED_LED,
        ENCODER_JAW_POSITION_SPEED_HIGH ? LOW : HIGH);
    IOExtender_.writeNoUpdate(ENCODER_CLAMP_SPEED_LED, ENCODER_CLAMP_SPEED_HIGH ? LOW : HIGH);

    AutoMode = IOExtender_.readNoUpdate(MODE_PIN) == LOW;

    // Actually send the i2c call
    IOExtender_.update();
}

// use on command, not allowed to modify
void Cleaner::home(SerialReceiverTransmitter::CommandMessage command)
{
    // reset the states based on what is received
    if (command.M80.a > 0)
    {
        state_.jaw_rotation     = 0.0f;
        des_state_.jaw_rotation = 0.0f;
        // enter a loop until we hit the limit switch

        while (digitalRead(LIMIT_SWITCH_PIN_JAW_ROTATION))
        {
            jaw_rotation_motor_.setSpeedUnits(HOMING_SPEED);
            jaw_rotation_motor_.runSpeed();
        }
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

/**
 * @brief Resets the cleaner system to its default state.
 *
 * This function zeroes out the internal state and desired state structures,
 * and resets the current position of all motors to zero.
 *
 * @return EXIT_SUCCESS on successful reset.
 */
int Cleaner::reset()
{
    // Reset the state to default values
    memset(&state_, 0, sizeof(state_));
    memset(&des_state_, 0, sizeof(des_state_));

    for (auto* motor : motors)
    {
        motor->setCurrentPosition(0);
    }
    return EXIT_SUCCESS;
}

/**
 * @brief Stops all motors managed by the Cleaner instance.
 *
 * This function iterates through all motors, issuing stop commands and running
 * each motor until all have completely stopped. The function blocks until no
 * motors are reported as running.
 */
void Cleaner::stop()
{
    uint8_t numRunning = 0;
    while (numRunning > 0)
    {
        numRunning = 0;  // Reset the counter
        // Stop all motors
        for (auto* motor : motors)
        {
            motor->stop();
            motor->run();
            if (motor->isRunning())
            {
                numRunning++;
            }
        }
    }
}

/**
 * @brief Processes a received command and updates the system state accordingly.
 *
 * This function interprets the provided command message and performs actions such as
 * moving motors, setting speeds, accelerations, current limits, or executing homing and dwell
 * commands. Each command type (G0, G4, G28, G90, M80, M17, M906) is handled individually, updating
 * the desired state or hardware parameters as required.
 *
 * @param command The command message received from the serial interface, containing
 *                various possible instructions for the cleaner system.
 */
void Cleaner::processCommand(SerialReceiverTransmitter::CommandMessage command)
{
    bool ack = false;
    if (command.G0.received)
    {
        // Move command, modify the state to the desired state
        command.G0.received     = false;           // reset the received
        des_state_.jaw_rotation = command.G0.a;    // jaw rotation
        des_state_.jaw_pos      = command.G0.y;    // jaw position
        des_state_.clamp_pos    = command.G0.c;    // clamp position
        des_state_.is_Brake     = command.G0.val;  // brake
        command_in_progress_    = true;
    }
    if (command.G4.received)
    {
        // Dwell command, wait for a certain time
        command.G4.received = false;  // reset the received
        delay(command.G4.val);        // kinda sucks it's blocking but good enough for now
        ack = true;
    }
    if (command.G28.received)
    {
        // Home command
        command.G28.received = false;
        home(command);
        ack = true;
    }
    if (command.G90.received)
    {
        // Absolute positioning command, not yet implemented
        Serial.print("I ain't doin that\n");
        ack = true;
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
        ack = true;
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
        ack = true;
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
            electricalParams.runCurrent_mA                  = command.M906.y * 1000;
            jaw_pos_motor_.apply(electricalParams);  // set current limit in mA
        }
        if (command.M906.c != 0)
        {
            StepperMotor::ElectricalParams electricalParams = clamp_motor_.getElectricalParams();
            electricalParams.runCurrent_mA                  = command.M906.c * 1000;
            clamp_motor_.apply(electricalParams);  // set current limit in mA
        }
        ack = true;
    }
    if (ack && !command_in_progress_)
    {
        Serial.println(SERIAL_ACK);
    }
}

bool Cleaner::atTarget() const
{
    State error = des_state_ - state_;
    return fabs(error.jaw_rotation) < POSITION_EPSILON &&
           fabs(error.jaw_pos) < POSITION_EPSILON &&
           fabs(error.clamp_pos) < POSITION_EPSILON;
}

/**
 * @brief Safely shuts down the cleaner system by stopping all motors and setting the emergency stop
 * flag.
 *
 * This function kills the jaw rotation motor, jaw position motor, and clamp motor to ensure
 * all moving parts are halted. It also sets the emergency stop state to true, indicating
 * that the system is in a stopped condition.
 *
 * @return int Returns EXIT_SUCCESS upon successful shutdown.
 */
int Cleaner::shutdown()
{
    jaw_rotation_motor_.kill();
    jaw_pos_motor_.kill();
    clamp_motor_.kill();

    state_.is_Estopped = true;

    return EXIT_SUCCESS;
}