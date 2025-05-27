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

    State updateDesStateManual();
    bool updatePCF8575_flag = false;

    State updateRealState();

    PCF8575 getIOExpander() { return IOExtender_; }

    StepperMotor getJawRotationMotor() { return jaw_rotation_motor_; }
    StepperMotor getJawPosMotor() { return jaw_pos_motor_; }
    StepperMotor getClampMotor() { return clamp_motor_; }
    RotaryEncoder getJawRotationEncoder() { return encoder_jaw_rotation_; }

    AS5048A& getEncoder() { return encoder_; }

private:
    struct ToggleButtonState
    {
        const char* name;
        bool current;
        bool last;
        bool& target;

        ToggleButtonState(const char* n, bool& targetRef)
            : name(n),
              current(false),
              last(false),
              target(targetRef)
        {
        }
    };

    /* Check if the button is currently pressed (state is low) and if it was high last time
     * If this condition is met, toggle the corresponding speed variable to
     * switch between high and low speed modes.
     */
    inline void toggleButton(ToggleButtonState& button)
    {
        if (!button.current && button.last)
        {
            button.target = !button.target;
        }
        button.last = button.current;
    }

    std::vector<ToggleButtonState> ENCODER_BUTTONS = {
        {"Jaw Rotation", ENCODER_JAW_ROTATION_SPEED_HIGH},
        {"Jaw Position", ENCODER_JAW_POSITION_SPEED_HIGH},
        {"Clamp", ENCODER_CLAMP_SPEED_HIGH}};

    State state_;
    State des_state_;

    float last_enc_jaw_rot_;
    float last_enc_jaw_pos_;
    float last_enc_clamp_;

    constexpr static const char* SERIAL_ACK = "At Pos\r";

    constexpr static float ENCODER_JAW_ROTATION_SENSITIVITY =
        M_TWOPI / 10.0f;  // Sensitivity for jaw rotation encoder
    constexpr static float ENCODER_JAW_POSITION_SENSITIVITY =
        1.0f;  // Sensitivity for jaw position encoder
    constexpr static float ENCODER_CLAMP_SENSITIVITY = 1.0f;  // Sensitivity for clamp encoder

    constexpr static const float RUN_RATE_HZ  = 1000.0f;  // Change this to desired Hz
    constexpr static const float HOMING_SPEED = 100.0f;   // Speed for homing in mm/s

    bool ENCODER_CLAMP_SPEED_HIGH        = false;
    bool ENCODER_JAW_POSITION_SPEED_HIGH = false;
    bool ENCODER_JAW_ROTATION_SPEED_HIGH = false;

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
    DiscreteFilter<3> encoderLowpassFilter;

    DiscreteFilter<3> JawRotationPID;
    DiscreteFilter<3> JawPositionPID;
    DiscreteFilter<3> ClampPID;
};
