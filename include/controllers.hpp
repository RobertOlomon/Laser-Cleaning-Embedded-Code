#pragma once
#include "discrete_filter.hpp"
namespace controller
{
Coefficients<3, float> PIDControllerCoefficients(float kp, float ki, float kd, float ts);
Coefficients<2, float> PhaseLagLeadCoefficients(float k, float z, float p, float ts);
}
