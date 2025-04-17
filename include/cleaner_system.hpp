#pragma once

#include "TMCStepper.h"
#include "stepper_motor.hpp"
#include "AS5048A.h"
#include "serial_receiver.hpp"
class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int reset();
    int shutdown();
    void home();
    void processCommand(SerialReceiver::CommandMessage command);
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

    AS5048A encoder_;
private:
    constexpr static int JAW_ROTATION_CS_PIN = -1;   // Pin for jaw rotation motor
    constexpr static int JAW_POSITION_CS_PIN = -1;  // Pin for jaw position motor
    constexpr static int CLAMP_CS_PIN        = -1;  // Pin for clamp motor
    constexpr static int LIMIT_SWITCH_PIN    = -1;  // Pin for limit switch
    constexpr static int ENCODER_CS_PIN      = 10;   // Pin for encoder CS

    constexpr static int SW_MOSI = 23;  // Pin for software SPI MOSI
    constexpr static int SW_MISO = 19;  // Pin for software SPI MISO
    constexpr static int SW_SCK  = 18;  // Pin for software SPI SCK

    constexpr static const float ENCODER_READ_RATE_HZ = 1000.0f; // Change this to desired Hz
    constexpr static const float HOMING_SPEED = 100.0f;          // Speed for homing in mm/s

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

};
