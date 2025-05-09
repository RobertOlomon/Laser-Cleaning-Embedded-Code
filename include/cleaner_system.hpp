#pragma once

#include "AS5048A.h"
#include "PCF8575.h"
#include "RotaryEncoder.h"
#include "TMCStepper.h"
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

    static void is_updatePCF8575_message();
    void updatePCF8575();
    void printDriverDebug();
    struct State
    {
        float jaw_rotation;
        float jaw_pos;
        float clamp_pos;
        bool is_Estopped;

        // subtraction operator, keeps the bools the same
        State operator-(const State& other) const
        {
            return {
                jaw_pos - other.jaw_pos,
                jaw_rotation - other.jaw_rotation,
                clamp_pos - other.clamp_pos,
                is_Estopped};
        }
    };

    State getDesStateManual();

    State getRealState();

    State state_;
    State des_state_;

    AS5048A encoder_;

private:
    constexpr static char SERIAL_ACK = 'A';

    constexpr static int ENCODER_JAW_POSITION_SENSITIVITY =
        1.0f;  // Sensitivity for jaw position encoder
    constexpr static int ENCODER_JAW_ROTATION_SENSITIVITY =
        1.0f;                                               // Sensitivity for jaw rotation encoder
    constexpr static int ENCODER_CLAMP_SENSITIVITY = 1.0f;  // Sensitivity for clamp encoder

    constexpr static const float JAW_JOG_ERROR          = 10.0f;
    constexpr static const float JAW_ROTATION_JOG_ERROR = 10.0f;
    constexpr static const float CLAMP_JOG_ERROR        = 10.0f;
    constexpr static const float ENCODER_READ_RATE_HZ   = 1000.0f;  // Change this to desired Hz
    constexpr static const float HOMING_SPEED           = 100.0f;   // Speed for homing in mm/s

    bool ENCODER_CLAMP_SPEED_HIGH        = false;
    bool ENCODER_JAW_POSITION_SPEED_HIGH = false;
    bool ENCODER_JAW_ROTATION_SPEED_HIGH = false;

    bool last_jaw_rotation_button_state;
    bool last_jaw_position_button_state;
    bool last_clamp_button_state;

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

    PCF8575 IOExtender_;

    RotaryEncoder encoder_jaw_rotation_;
    RotaryEncoder encoder_jaw_pos_;
    RotaryEncoder encoder_clamp_;

    // handy array of all motors
    StepperMotor* motors[3];

    std::array<float, 3> natural_coeffs_{};
    std::array<float, 3> forced_coeffs_{};
    DiscreteFilter<3> encoderFilter;
};
