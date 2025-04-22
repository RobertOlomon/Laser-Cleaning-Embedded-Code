#pragma once

#include "TMCStepper.h"
#include "stepper_motor.hpp"
#include "AS5048A.h"
#include "serial_receiver.hpp"
#include "discrete_filter.hpp"
class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int reset();
    int shutdown();
    void home();
    void processCommand(SerialReceiver::CommandMessage command);
    void run();
    struct State
    {
        float jaw_rotation;
        float jaw_pos;
        float clamp_pos;
        bool is_Estopped;

        // subtraction operator, keeps the bools the same
        State operator-(const State& other) const {
            return {
                jaw_pos - other.jaw_pos,
                jaw_rotation - other.jaw_rotation,
                clamp_pos - other.clamp_pos,
                is_Estopped
            };
        }
    };
    
    State getDesStateManual();

    State getRealState();

    State state_;
    State des_state_;

    AS5048A encoder_;
private:
    constexpr static int JAW_ROTATION_CS_PIN = -1;   // Pin for jaw rotation motor
    constexpr static int JAW_POSITION_CS_PIN = -1;  // Pin for jaw position motor
    constexpr static int CLAMP_CS_PIN        = -1;  // Pin for clamp motor
    constexpr static int LIMIT_SWITCH_PIN    = -1;  // Pin for limit switch
    constexpr static int ENCODER_CS_PIN      = 10;   // Pin for encoder CS
    constexpr static int JAW_JOG_FORWARD_PIN = -1;
    constexpr static int JAW_JOG_BACKWARD_PIN = -1;
    constexpr static int JAW_ROTATION_JOG_FORWARD_PIN = -1;
    constexpr static int JAW_ROTATION_JOG_BACKWARD_PIN = -1;
    constexpr static int CLAMP_JOG_FORWARD_PIN = -1;
    constexpr static int CLAMP_JOG_BACKWARD_PIN = -1;



    constexpr static int SW_MOSI = 23;  // Pin for software SPI MOSI
    constexpr static int SW_MISO = 19;  // Pin for software SPI MISO
    constexpr static int SW_SCK  = 18;  // Pin for software SPI SCK

    constexpr static const float JAW_JOG_ERROR = 10.0f;
    constexpr static const float JAW_ROTATION_JOG_ERROR = 10.0f;
    constexpr static const float CLAMP_JOG_ERROR = 10.0f;
    constexpr static const float ENCODER_READ_RATE_HZ = 1000.0f; // Change this to desired Hz
    constexpr static const float HOMING_SPEED = 100.0f;          // Speed for homing in mm/s

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

    // handy array of all motors
    StepperMotor* motors[3];
    
    std::array<float, 3> natural_coeffs_{};
    std::array<float, 3> forced_coeffs_{};
    DiscreteFilter<3> encoderFilter;
};
