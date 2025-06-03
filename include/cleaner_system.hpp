#pragma once

#include <vector>

#include "AS5048A.hpp"
#include "PCF8575.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
#include "controllers.hpp"
#include "discrete_filter.hpp"
#include "pin_defs.hpp"
#include "serial_receiver.hpp"
#include "stepper_motor.hpp"

class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int begin();
    int reset();
    int shutdown();
    void home(SerialReceiver::CommandMessage command);
    void processCommand(SerialReceiver::CommandMessage command);
    void run();
    void initializeManualMode();
    void initializeAutoMode();

    void stop();

    void PCFMessageRec();
    void updatePCF8575();

    enum CleanerOperatorMode
    {
        MANUAL = 0,
        AUTO   = 1,
        DEBUG
    };

    struct State
    {
        float jaw_rotation;
        float jaw_pos;
        float clamp_pos;
        bool is_Estopped;
        bool is_Brake;

        State() : jaw_rotation(0), jaw_pos(0), clamp_pos(0), is_Estopped(false), is_Brake(false) {}
        State(float jaw_rotation, float jaw_pos, float clamp_pos, bool is_Estopped, bool is_Brake)
            : jaw_rotation(jaw_rotation),
              jaw_pos(jaw_pos),
              clamp_pos(clamp_pos),
              is_Estopped(is_Estopped),
              is_Brake(is_Brake)
        {
        }

        // subtraction operator, is_Estopped is or
        // Brake returns if they're not equal
        State operator-(const State& other) const
        {
            return {
                jaw_rotation - other.jaw_rotation,
                jaw_pos - other.jaw_pos,
                clamp_pos - other.clamp_pos,
                is_Estopped || other.is_Estopped,
                is_Brake != other.is_Brake};
        }
        // is only greater when all states are greater
        bool operator>(const State& other) const
        {
            return (jaw_pos > other.jaw_pos) && (jaw_rotation > other.jaw_rotation) &&
                   (clamp_pos > other.clamp_pos);
        }
        bool operator<(const State& other) const
        {
            return (jaw_pos < other.jaw_pos) && (jaw_rotation < other.jaw_rotation) &&
                   (clamp_pos < other.clamp_pos);
        }

        // Print method
        void print(const char* label = "State") const
        {
            Serial.print(label);
            Serial.println(":");

            Serial.print("  Jaw Rotation: ");
            Serial.println(jaw_rotation, 3);

            Serial.print("  Jaw Position: ");
            Serial.println(jaw_pos, 3);

            Serial.print("  Clamp Position: ");
            Serial.println(clamp_pos, 3);

            Serial.print("  Emergency Stop: ");
            Serial.println(is_Estopped ? "YES" : "NO");

            Serial.print("  Brake: ");
            Serial.println(is_Brake ? "YES" : "NO");

            Serial.println();  // blank line
        }
    };

    bool updatePCF8575_flag = false;
    void updateModeAuto();

    State updateDesStateManual();
    State updateRealState();

    /* -------- Getters -------- */

    bool isAutoMode() const { return AutoMode; }
    PCF8575 getIOExpander() const { return IOExtender_; }

    StepperMotor& getJawRotationMotor() { return jaw_rotation_motor_; }
    StepperMotor& getJawPosMotor() { return jaw_pos_motor_; }
    StepperMotor& getClampMotor() { return clamp_motor_; }

    AS5048A& getEncoder() { return encoder_; }

private:
    void runControl();

    static constexpr uint32_t DEBOUNCE_TIME_MS = 10;
    struct ToggleButtonState
    {
        const char* name;
        bool rawState, rawStateLast;  // Raw input sampled at 1 kHz
        bool debouncedState, debouncedStateLast;
        bool& target;
        uint32_t lastDebounceTime;

        ToggleButtonState(const char* n, bool& targetRef)
            : name(n),
              rawState(false),
              rawStateLast(false),
              debouncedState(false),
              debouncedStateLast(false),
              target(targetRef),
              lastDebounceTime(0)
        {
        }
    };

    /* Check if the button is currently pressed (state is low) and if it was high last time
     * If this condition is met, toggle the corresponding speed variable to
     * switch between high and low speed modes.
     */
    inline void toggleButton(ToggleButtonState& button)
    {
        uint32_t now = millis();

        // If raw state has changed, reset debounce timer
        if (button.rawState != button.rawStateLast)
        {
            button.lastDebounceTime = now;
        }

        // If debounce time has passed, update debounced state
        if ((now - button.lastDebounceTime) >= DEBOUNCE_TIME_MS)
        {
            button.debouncedState = button.rawState;

            if (!button.debouncedState && button.debouncedStateLast)
            {
                // Button was released, toggle the target state
                button.target = !button.target;
            }
        }
        button.rawStateLast       = button.rawState;
        button.debouncedStateLast = button.debouncedState;
    }

    std::vector<ToggleButtonState> ENCODER_BUTTONS = {
        {"Jaw Rotation", ENCODER_JAW_ROTATION_SPEED_HIGH},
        {"Jaw Position", ENCODER_JAW_POSITION_SPEED_HIGH},
        {"Clamp", ENCODER_CLAMP_SPEED_HIGH}};

    State state_;
    State des_state_;

    constexpr static const char* SERIAL_ACK = "At Pos\r";

    constexpr static float ENCODER_JAW_ROTATION_SENSITIVITY = M_TWOPI / 100.0f;
    constexpr static float ENCODER_JAW_POSITION_SENSITIVITY = 1.0f;
    constexpr static float ENCODER_CLAMP_SENSITIVITY = 0.1f;

    constexpr static const float RUN_RATE_HZ  = 1000.0f;
    constexpr static const float HOMING_SPEED = 100.0f;  // Speed for homing in mm/s

    float last_enc_jaw_rot_;
    float last_enc_jaw_pos_;
    float last_enc_clamp_;

    bool ENCODER_CLAMP_SPEED_HIGH        = false;
    bool ENCODER_JAW_POSITION_SPEED_HIGH = false;
    bool ENCODER_JAW_ROTATION_SPEED_HIGH = false;

    bool AutoMode = false;
    bool breakSwitchedOn = false;

    PCF8575 IOExtender_;  // Must be defined before the rotary encoders

    AS5048A encoder_;

    RotaryEncoder encoder_jaw_rotation_;
    RotaryEncoder encoder_jaw_pos_;
    RotaryEncoder encoder_clamp_;

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

    // handy array of all motors
    StepperMotor* motors[3];

    // Filters and Controllers
    DiscreteFilter<3> clampLowpassFilter;

    DiscreteFilter<3> JawRotationPID;
    DiscreteFilter<3> JawPositionPID;
    DiscreteFilter<3> ClampPID;

    float potValue     = 0;
    float lastPotValue = 0;

    float desired_clamp_speed = 0;

    unsigned long last_read_time = 0;
};