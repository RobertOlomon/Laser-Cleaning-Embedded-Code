#include "TMC5160.h"
#include "cleaner_system_constants.hpp"
#include "stepper_motor.hpp"
class Cleaner
{
public:
    Cleaner();
    ~Cleaner();
    int reset();

private:
    float jaw_rotation_;
    float jaw_pos_;
    float clamp_pos_;
    bool is_clamped_;

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