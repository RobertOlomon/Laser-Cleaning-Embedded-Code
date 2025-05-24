#include <array>

#include "controllers.hpp"
#include "discrete_filter.hpp"


namespace controller
{
/**
 * @brief function for the getting the coefficients of a pid.
 * @param [in] kp Proportional gain.
 * @param [in] ki Integral gain.
 * @param [in] kd Derivative gain.
 * @param [in] ts Sampling time.
 *
 * This constructor initializes the PID controller coefficients based on the given gains and
 * sampling time.
 */
Coefficients<3, float> PIDControllerCoefficients(float kp, float ki, float kd, float ts)
{
    Coefficients<3, float> coefficients;
    std::array<float, 3> forced_coeffs_;
    std::array<float, 3> natural_coeffs_;
    // Natural coefficients (numerator)
    float a0       = 1.0f;
    float a1       = 0.0f;
    float a2       = -1.0f;
    natural_coeffs_ = {a0, a1, a2};
    
    // Forced coefficients (denominator)
    float b0        = kp + ki * ts / 2.0f + 2.0f * kd / ts;
    float b1        = ki * ts - 4.0f * kd / ts;
    float b2        = -kp - ki * ts / 2.0f + 2.0f * kd / ts;
    forced_coeffs_ = {b0, b1, b2};

    coefficients.forcedResponseCoefficients  = forced_coeffs_;
    coefficients.naturalResponseCoefficients = natural_coeffs_;
    return coefficients;
}

/**
 * @brief Function for getting PhaseLagLeadCoefficients.
 * @param [in] k Gain.
 * @param [in] z Zero of the transfer function.
 * @param [in] p Pole of the transfer function.
 * @param [in] ts Sampling time.
 */
Coefficients<2, float> PhaseLagLeadCoefficients(float k, float z, float p, float ts)
{
    Coefficients<2, float> coefficients;
    std::array<float, 2> forced_coeffs_;
    std::array<float, 2> natural_coeffs_;

    // Calculate the coefficients for the Lag-Lead controller
    float a0 = 1.0f;
    float a1 = (p * ts - 2.0f) / (p * ts + 2.0f);

    natural_coeffs_ = {a0, a1};

    float b0 = k * (z * ts + 2.0f) / (p * ts + 2.0f);
    float b1 = k * (z * ts - 2.0f) / (p * ts + 2.0f);

    forced_coeffs_ = {b0, b1};

    coefficients.forcedResponseCoefficients  = forced_coeffs_;
    coefficients.naturalResponseCoefficients = natural_coeffs_;
    return coefficients;
}
};  // namespace controller