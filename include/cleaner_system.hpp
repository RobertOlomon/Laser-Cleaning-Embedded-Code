#pragma once

#include "TMCStepper.h"
#include "stepper_motor.hpp"

class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int reset();
    int shutdown();
    void home();
    struct State
    {
        float jaw_rotation;
        float jaw_pos;
        float clamp_pos;
        bool is_clamped;
        bool is_Estopped;
    };

    State getRealState();

    State state_;
    State des_state_;

private:
    constexpr static int JAW_ROTATION_CS_PIN = 9;   // Pin for jaw rotation motor
    constexpr static int JAW_POSITION_CS_PIN = 10;  // Pin for jaw position motor
    constexpr static int CLAMP_CS_PIN        = 11;  // Pin for clamp motor
    constexpr static int LIMIT_SWITCH_PIN    = 12;  // Pin for limit switch
    constexpr static int ENCODER_CS_PIN      = 2;   // Pin for encoder CS

    constexpr static int SW_MOSI = 23;  // Pin for software SPI MOSI
    constexpr static int SW_MISO = 19;  // Pin for software SPI MISO
    constexpr static int SW_SCK  = 18;  // Pin for software SPI SCK

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

};
