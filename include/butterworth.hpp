#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>

enum FilterType : uint8_t
{
    LOWPASS = 0,
    HIGHPASS,
    BANDPASS,
    BANDSTOP,
};

/**
 * used to transform poles from the laplace domain to the
 * Z domain for descrete time using the bilinear transform
 *
 * @param [in] s a pole or zero from the laplace domain
 * @param [in] Ts the sample time
 * @return a complex number corresponding to the Z domain location
 */
std::complex<double> s2z(std::complex<double> s, double Ts)
{
    return (1.0 + (Ts / 2) * s) / (1.0 - (Ts / 2) * s);
}

/**
 * used to multiply out a series of zeros to obtain a list of coefficents
 *
 * @param [in] zeros a vector of complex poles or zeros to multiply out, works
 * for any polynomial
 *
 * @return a vector of coefficients for the polynomial with the 0th index being the
 * constant term and the last index being the leading coefficient
 * @note the coefficients are returned as doubles, the imaginary part of the
 * complex number is ignored
 */
template <uint8_t ORDER>
std::array<double, ORDER + 1> expandPolynomial(std::array<std::complex<double>, ORDER> zeros)
{
    std::array<std::complex<double>, ORDER + 1> coefficients{
        std::complex<double>(1.0, 0.0)};  // Start with 1

    for (size_t i = 0; i < ORDER; i++)
    {
        // Multiply current polynomial by (x - zero)
        std::array<std::complex<double>, ORDER + 1> newCoefficients{std::complex<double>(0.0, 0.0)};
        for (size_t j = 0; j < i + 1; j++)
        {
            newCoefficients[j] -=
                coefficients[j] * zeros[i];             // Multiply current coefficient by zero
            newCoefficients[j + 1] += coefficients[j];  // Shift current coefficient
        }
        coefficients.swap(newCoefficients);
    }
    std::array<double, ORDER + 1> stripped_coefficients{};
    // Extract the real part of each coefficient, the imaginary appears to be an
    // artifact of double
    for (size_t i = 0; i < ORDER + 1; i++)
    {
        stripped_coefficients[i] = coefficients[i].real();
    }

    return stripped_coefficients;
}

template <uint8_t ORDER>
std::complex<double> evaluateFrequencyResponse(
    const std::array<double, ORDER + 1>& b,
    const std::array<double, ORDER + 1>& a,
    double w,
    double Ts)
{
    std::complex<double> j(0, 1);  // imaginary unit i = sqrt(-1)
    std::complex<double> numerator(0, 0);
    std::complex<double> denominator(0, 0);

    double omega = w * Ts;  // omega is in terms of rad/sample

    // Evaluate numerator: b0 + b1 * e^{-jω} + b2 * e^{-j2ω} + ...
    for (size_t k = 0; k < b.size(); ++k)
    {
        numerator += b[k] * std::exp(-j * omega * static_cast<double>(k));
    }

    // Evaluate denominator: a0 + a1 * e^{-jω} + a2 * e^{-j2ω} + ...
    for (size_t k = 0; k < a.size(); ++k)
    {
        denominator += a[k] * std::exp(-j * omega * static_cast<double>(k));
    }

    return numerator / denominator;
}


template <uint8_t ORDER>
class Butterworth
{
public:
    /**
     * @param[in] wc   for LOW/HIGHPASS: cutoff ωc.
     *                 for BANDPASS/BANDSTOP: lower edge ωl.
     * @param[in] Ts   sample time.
     * @param[in] type filter type.
     * @param[in] wh   upper edge ωh (only used for band filters).
     */
    Butterworth(double wc, double Ts, FilterType type = LOWPASS, double wh = 0.0)
        : naturalResponseCoefficients(),
          forcedResponseCoefficients()
    {
        const int n = ORDER;

        // For band filters we treat wc as ωl
        double wl  = wc;
        double whp = wh;

        // pre-warp all edges for bilinear transform
        wl  = (2.0 / Ts) * std::atan(wl * (Ts / 2.0));
        whp = (2.0 / Ts) * std::atan(whp * (Ts / 2.0));

        // generate N prototype poles on unit circle
        std::array<std::complex<double>, ORDER> poles;
        for (int k = 0; k < n; ++k)
        {
            double theta = M_PI * (2.0 * k + 1) / (2.0 * n) + M_PI / 2.0;
            poles[k]     = std::complex<double>(std::cos(theta), std::sin(theta));
        }

        // apply the appropriate s-domain transform to each pole
        switch (type)
        {
            case LOWPASS:
                for (int j = 0; j < n; ++j) poles[j] *= wl;
                break;

            case HIGHPASS:
            {
                for (int j = 0; j < n; ++j)
                {
                    poles[j] = wl / poles[j];
                }
                break;
            }
            case BANDPASS:
            {
                if (whp <= wl) throw std::invalid_argument("wh must be > wl for BANDPASS");
                double B    = whp - wl;
                double W0sq = whp * wl;
                for (int j = 0; j < n; ++j)
                {
                    auto p = poles[j];
                    // s → (s² + ωh·ωl) / (s·(ωh−ωl))
                    poles[j] = (p * p + W0sq) / (p * B);
                }
                break;
            }

            case BANDSTOP:
            {
                if (whp <= wl) throw std::invalid_argument("wh must be > wl for BANDSTOP");
                double B    = whp - wl;
                double W0sq = whp * wl;
                for (int j = 0; j < n; ++j)
                {
                    auto p = poles[j];
                    // s → (s·(ωh−ωl)) / (s² + ωh·ωl)
                    poles[j] = (p * B) / (p * p + W0sq);
                }
                break;
            }

            default:
                throw std::invalid_argument("Unknown filter type");
        }

        // now map each analog pole into the z-plane
        std::array<std::complex<double>, ORDER> zPoles;
        for (int i = 0; i < n; ++i) zPoles[i] = s2z(poles[i], Ts);

        std::array<std::complex<double>, ORDER> zZeros;
        switch (type)
        {
            case LOWPASS:
                // zeros: for Butterworth lowpass all z-zeros at z = –1
                zZeros.fill(std::complex<double>(-1, 0));
                break;
            case HIGHPASS:
                // zeros: for butterworth highpass all z-zeros are at z = 1
                zZeros.fill(std::complex<double>(1, 0));
                break;
            default:
                break;
        }

        // get expanded polynomials
        auto b = expandPolynomial<ORDER>(zZeros);
        auto a = expandPolynomial<ORDER>(zPoles);

        // scale so DC gain = 1
        switch (type)
        {
            case LOWPASS:
            {
                double scale = 1 / std::abs(evaluateFrequencyResponse<ORDER>(b, a, 0, 1 / 500.0)); // DC Gain
                for (auto& coef : b) coef *= scale;
                break;
            }
            case HIGHPASS:
            {
                // std::complex<double> one = 1.0;
                double scale = 1 / std::abs(evaluateFrequencyResponse<ORDER>(b, a, 500, 1 / 500.0));
                for (auto& coef : b) coef *= scale;
                break;
            }
        }

        // store in member arrays (reverse order)
        for (size_t i = 0; i < ORDER + 1; ++i)
        {
            naturalResponseCoefficients[ORDER - i] = a[i];
            forcedResponseCoefficients[ORDER - i]  = b[i];
        }
    }

    std::array<float, ORDER + 1> getNaturalResponseCoefficients() const
    {
        return naturalResponseCoefficients;
    }
    std::array<float, ORDER + 1> getForcedResponseCoefficients() const
    {
        return forcedResponseCoefficients;
    }

private:
    std::array<float, ORDER + 1> naturalResponseCoefficients;
    std::array<float, ORDER + 1> forcedResponseCoefficients;
};