#include "TMC5160.h"
#include "stepper_motor.hpp"
class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int reset();

private:
    struct State
    {
        float jaw_rotation;
        float jaw_pos;
        float clamp_pos;
        bool is_clamped;
    };

    int JAW_ROTATION_CS_PIN = 9;   // Pin for jaw rotation motor
    int JAW_POSITION_CS_PIN = 10;  // Pin for jaw position motor
    int CLAMP_CS_PIN        = 11;  // Pin for clamp motor

    State state_;
    State des_state_;

    StepperMotor jaw_rotation_motor_;
    StepperMotor jaw_pos_motor_;
    StepperMotor clamp_motor_;

    TMC5160::PowerStageParameters jaw_power_params_;
    TMC5160::MotorParameters jaw_motor_params_;
    TMC5160::PowerStageParameters jaw_pos_power_params_;
    TMC5160::MotorParameters jaw_pos_motor_params_;
    TMC5160::PowerStageParameters clamp_power_params_;
    TMC5160::MotorParameters clamp_motor_params_;

    TMC5160::MotorParameters makeJawMotorParams();
    TMC5160::MotorParameters makeJawPosMotorParams();
    TMC5160::MotorParameters makeClampMotorParams();
};