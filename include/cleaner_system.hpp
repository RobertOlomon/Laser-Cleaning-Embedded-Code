#pragma once

#include <vector>

#include "AS5048A.h"
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
    void home();
    void processCommand(SerialReceiver::CommandMessage command);
    void run();
    void initializeManualMode();
    void initializeAutoMode();

    void stop();

    void is_updatePCF8575_message();
    void updatePCF8575();
    void printDriverDebug();

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

        // subtraction operator, is_Estopped is or
        State operator-(const State& other) const
        {
            return {
                jaw_pos - other.jaw_pos,
                jaw_rotation - other.jaw_rotation,
                clamp_pos - other.clamp_pos,
                is_Estopped || other.is_Estopped};
        }
        bool operator>(const State& other) const{
            return (jaw_pos > other.jaw_pos) && (jaw_rotation > other.jaw_rotation) && (clamp_pos > other.clamp_pos);
        }

        State() : jaw_rotation(0), jaw_pos(0), clamp_pos(0), is_Estopped(false) {}
        State(float jaw_rotation, float jaw_pos, float clamp_pos, bool is_Estopped) :
        jaw_rotation(jaw_rotation), jaw_pos(jaw_pos), clamp_pos(clamp_pos), is_Estopped(is_Estopped) {}
    };

    State updateDesStateManual();

    State updateRealState();

    // StepperMotor* getJawRotationMotor() { return &jaw_rotation_motor_; }
    // StepperMotor* getJawPosMotor() { return &jaw_pos_motor_; }
    // StepperMotor* getClampMotor() { return &clamp_motor_; }
    // RotaryEncoder* getJawRotationEncoder() { return &encoder_jaw_rotation_; }

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

    AS5048A encoder_;

    bool updatePCF8575_flag;

    constexpr static char SERIAL_ACK = 'A';

    constexpr static float ENCODER_JAW_ROTATION_SENSITIVITY =
        0.1f;  // Sensitivity for jaw rotation encoder
    constexpr static float ENCODER_JAW_POSITION_SENSITIVITY =
        1.0f;  // Sensitivity for jaw position encoder
    constexpr static float ENCODER_CLAMP_SENSITIVITY = 1.0f;  // Sensitivity for clamp encoder

    constexpr static const float JAW_JOG_ERROR          = 10.0f;
    constexpr static const float JAW_ROTATION_JOG_ERROR = 10.0f;
    constexpr static const float CLAMP_JOG_ERROR        = 10.0f;
    constexpr static const float RUN_RATE_HZ            = 1000.0f;  // Change this to desired Hz
    constexpr static const float HOMING_SPEED           = 100.0f;   // Speed for homing in mm/s

    bool ENCODER_CLAMP_SPEED_HIGH        = false;
    bool ENCODER_JAW_POSITION_SPEED_HIGH = false;
    bool ENCODER_JAW_ROTATION_SPEED_HIGH = false;

    RotaryEncoder encoder_jaw_rotation_;
    RotaryEncoder encoder_jaw_pos_;
    RotaryEncoder encoder_clamp_;

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

    PCF8575 IOExtender_;

    // handy array of all motors
    StepperMotor* motors[3];

    // Filters and Controllers
    std::array<float, 3> encoder_natural_coeffs_{};
    std::array<float, 3> encoder_forced_coeffs_{};
    DiscreteFilter<3> encoderLowpassFilter;

    std::array<float, 3> natural_coeffs_jaw_rotation_PID{};
    std::array<float, 3> forced_coeffs_jaw_rotation_PID{};
    DiscreteFilter<3> JawRotationPID;
};
