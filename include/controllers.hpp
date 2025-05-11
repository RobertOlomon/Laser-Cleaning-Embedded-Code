#pragma once

#include <array>

class PIDControllerCoefficients
{
public:
    /**
     * @brief Constructor for the PIDControllerCoefficients class.
     * @param [in] kp Proportional gain.
     * @param [in] ki Integral gain.
     * @param [in] kd Derivative gain.
     * @param [in] ts Sampling time.
     *
     * This constructor initializes the PID controller coefficients based on the given gains and
     * sampling time.
     */
    PIDControllerCoefficients(float kp, float ki, float kd, float ts)
    {
        // Forced coefficients (denominator)
        float a0 = 1.0f;
        float a1 = 0.0f;
        float a2 = -1.0f;
        forced_coeffs_ = {a0, a1, a2};

        // Natural coefficients (numerator)
        float b0 = kp + ki * ts / 2.0f + 2.0f * kd / ts;
        float b1 = ki * ts - 4.0f * kd / ts;
        float b2 = -kp - ki * ts / 2.0f + 2.0f * kd / ts;
        natural_coeffs_ = {b0, b1, b2};
    }

    std::array<float, 3> getForcedResponseCoefficients() const { return forced_coeffs_; }
    std::array<float, 3> getNaturalResponseCoefficients() const { return natural_coeffs_; }

private:
    std::array<float, 3> natural_coeffs_;
    std::array<float, 3> forced_coeffs_;
};

class PhaseLagLeadCoefficients
{
public:
    /**
     * @brief Constructor for the PhaseLagLeadCoefficients class.
     * @param [in] k Gain.
     * @param [in] z Zero of the transfer function.
     * @param [in] p Pole of the transfer function.
     * @param [in] ts Sampling time.
     */
    PhaseLagLeadCoefficients(float k, float z, float p, float ts)
    : forced_coeffs_(),
      natural_coeffs_()
    {
        // Calculate the coefficients for the PID controller
        float a0 = 1.0f;
        float a1 = (p * ts - 2.0f) / (p * ts + 2.0f);

        forced_coeffs_ = {a0, a1};

        float b0 = k * (z * ts + 2.0f) / (p * ts + 2.0f);
        float b1 = k * (z * ts - 2.0f) / (p * ts + 2.0f);

        natural_coeffs_ = {b0, b1};
    }
    std::array<float, 2> getForcedResponseCoefficients() const { return forced_coeffs_; }
    std::array<float, 2> getNaturalResponseCoefficients() const { return natural_coeffs_; }

private:
    std::array<float, 2> natural_coeffs_;
    std::array<float, 2> forced_coeffs_;
};