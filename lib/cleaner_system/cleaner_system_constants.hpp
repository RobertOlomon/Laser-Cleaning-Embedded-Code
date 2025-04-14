#pragma once
#include "../TMC5160/TMC5160.h"

// 4 People have tried to fix this error, no clue why it doesn't work as an
// initializer list
// TMC5160::MotorParameters JawMotorParams {
//     16,                            // globalScaler
//     31,                            // irun
//     16,                            // ihold
//     TMC5160_Reg::FREEWHEEL_NORMAL, // freewheeling
//     30,                            // pwmOfsInitial
//     0                              // pwmGradInitial
// };

TMC5160::PowerStageParameters JawPowerParams; // uses defaults


TMC5160::MotorParameters JawMotorParams = [] {
    TMC5160::MotorParameters params;
    params.globalScaler = 16;
    params.irun = 31;
    params.ihold = 16;
    params.freewheeling = TMC5160_Reg::FREEWHEEL_NORMAL;
    params.pwmOfsInitial = 30;
    params.pwmGradInitial = 0;
    return params;
}();

