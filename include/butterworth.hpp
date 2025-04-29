#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
enum FilterType: uint8_t
    {
        LOWPASS = 0b00,
        HIGHPASS = 0b01,
        BANDPASS = 0b10,
        BANDSTOP = 0b11,
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
    const std::array<double, ORDER+1>& b, 
    const std::array<double, ORDER+1>& a, 
    double w,
    double Ts) 
{
    std::complex<double> j(0, 1);  // imaginary unit i = sqrt(-1)
    std::complex<double> numerator(0, 0);
    std::complex<double> denominator(0, 0);
    
    double omega = w * Ts; // omega is in terms of rad/sample

    // Evaluate numerator: b0 + b1 * e^{-jω} + b2 * e^{-j2ω} + ...
    for (size_t k = 0; k < b.size(); ++k) {
        numerator += b[k] * (std::cos(omega * static_cast<double>(k)) -j * std::sin(omega * static_cast<double>(k)));
    }

    // Evaluate denominator: a0 + a1 * e^{-jω} + a2 * e^{-j2ω} + ...
    for (size_t k = 0; k < a.size(); ++k) {
        denominator += a[k] * (std::cos(omega * static_cast<double>(k)) -j * std::sin(omega * static_cast<double>(k)));
    }

    return numerator / denominator;
}

 double complex_abs(std::complex<double> z) {
    return std::sqrt(z.real() * z.real() + z.imag() * z.imag());
}

 std::complex<double> complex_sqrt(std::complex<double> z)
{
    double r = std::sqrt(complex_abs(z));
    double theta = std::atan2(z.imag(), z.real()) * 0.5;
    return {r * std::cos(theta), r * std::sin(theta)};
}

 double factorial(int n)
{
    return (n <= 1) ? 1.0 : n * factorial(n - 1);
}

 double pow(double base, int exp)
{
    return (exp == 0) ? 1.0 : base * pow(base, exp - 1);
}


// Compute cosine via Taylor expansion
 double constexpr_cos(double x, int terms = 10)
{
    double sum = 1.0;
    for (int n = 1; n <= terms; ++n)
    {
        int exponent = 2 * n;
        double term = pow(-1.0, n) * pow(x, exponent) / factorial(exponent);
        sum += term;
    }
    return sum;
}

// Compute sine via Taylor expansion
 double constexpr_sin(double x, int terms = 10)
{
    double sum = x;
    for (int n = 1; n <= terms; ++n)
    {
        int exponent = 2 * n + 1;
        double term = pow(-1.0, n) * pow(x, exponent) / factorial(exponent);
        sum += term;
    }
    return sum;
}

template <uint8_t ORDER, FilterType Type = LOWPASS>
class Butterworth
{
public:

    /**
     * @note It's strongly recommended that you know what you're doing when using these filters
     * a general rule of thumb is there really isn't a reason to go over an order of 2 on the 
     * lowpass and highpass filters. When doing bandpass and bandstop it's important to keep the
     * size of the band lower than fs/4. Going too large and the band becomes more of a peak.
     * 
     * @param[in] wc   for LOW/HIGHPASS: cutoff ωc.
     *                 for BANDPASS/BANDSTOP: lower edge ωl.
     * @param[in] Ts   sample time.
     * @param[in] type filter type.
     * @param[in] wh   upper edge ωh (only used for band filters).
     */
         Butterworth(double wc,  
                double Ts,
                double wh = 0.0)
      : naturalResponseCoefficients(),
        forcedResponseCoefficients()
    {
        const int n = ORDER;

        // For band filters we treat wc as ωl
        double wl = wc;
        double whp = wh;

        // pre-warp all edges for bilinear transform
        wl  = (2.0 / Ts) * std::tan(wl  * (Ts / 2.0));
        whp = (2.0 / Ts) * std::tan(whp * (Ts / 2.0));

        // generate N prototype poles on unit circle
        std::array<std::complex<double>, COEFFICIENTS - 1> poles;
        for (int k = 0; k < n; ++k)
        {
            double theta = M_PI * (2.0 * k + 1) / (2.0 * n) + M_PI/2.0;
            poles[k] = std::complex<double>(std::cos(theta), std::sin(theta));
        }

        // apply the appropriate s-domain transform to each pole
        switch (Type)
        {
            case LOWPASS:
                for (int j = 0; j < n; ++j)
                    poles[j] = poles[j] * wl;
                break;

            case HIGHPASS:{
                for (int j = 0; j < n; ++j){
                    poles[j] = wl / poles[j];
                }
                break;
            }
            // In the case of a bandpass or a bandstop the ammount of poles doubles.
            case BANDPASS:
            {   
                if (whp <= wl){
                    throw std::invalid_argument("wh must be > wl for BANDPASS");
                }
                double B = whp - wl;
                double W0sq = whp * wl;

                for (int j = 0; j < ORDER; ++j)
                {
                    std::complex<double> p = poles[j];

                    std::complex<double> discriminant = (p * B) * (p * B) - 4.0 * W0sq;
                    std::complex<double> root = complex_sqrt(discriminant);

                    poles[2 * j]     = (p * B + root) * 0.5;
                    poles[2 * j + 1] = (p * B - root) * 0.5;
                }
                break;
            }

            case BANDSTOP:
            {
                if (whp <= wl){
                    throw std::invalid_argument("wh must be > wl for BANDSTOP");
                }
                double B = whp - wl;
                double W0sq = whp * wl;
                for (int j = 0; j < n; ++j)
                {
                    std::complex<double> p = poles[j];

                    std::complex<double> discriminant = B * B - (4.0 * -p * W0sq);
                    std::complex<double> root = complex_sqrt(discriminant);

                    poles[2 * j]     = (B + root) / (2.0*p);
                    poles[2 * j + 1] = (B - root) / (2.0*p);
                }
                break;
            }

            default:
                throw std::invalid_argument("Unknown filter type");
        }

        // now map each analog pole into the z-plane
        std::array<std::complex<double>, COEFFICIENTS - 1> zPoles;
        for (int i = 0; i < COEFFICIENTS - 1; ++i)
            zPoles[i] = s2z(poles[i], Ts);

        std::array<std::complex<double>, COEFFICIENTS - 1> zZeros;
        switch (Type){
            case LOWPASS: 
                // zeros: for Butterworth lowpass all z-zeros at z = –1
                zZeros.fill(std::complex<double>(-1,0));                
                break;
            case HIGHPASS:
                // zeros: for butterworth highpass all z-zeros are at z = 1
                zZeros.fill(std::complex<double>(1,0));
                break;
            case BANDPASS:
                for (int i = 0; i < COEFFICIENTS - 1; ++i)
                {
                    zZeros[i] = (i % 2 == 0) ? std::complex<double>(1, 0) : std::complex<double>(-1, 0);
                }
                break;
            case BANDSTOP:
                {
                    double omega0 = std::sqrt(wl*whp) * Ts; /* the notch (center) frequency in radians/sample */;

                    // Precompute real and imaginary parts manually
                    double realPart = constexpr_cos(omega0); // cos(omega0)
                    double imagPart = constexpr_sin(omega0); // sin(omega0)

                    std::complex<double> zeroPlus(realPart, imagPart);   // e^(+jω0)
                    std::complex<double> zeroMinus(realPart, -imagPart); // e^(-jω0)

                    for (int i = 0; i < COEFFICIENTS - 1; i += 2)
                    {
                        zZeros[i] = zeroPlus;     // first of pair
                        if (i + 1 < COEFFICIENTS)
                            zZeros[i + 1] = zeroMinus;  // second of pair
                    }
                }
                break;
            default:
                break;
        }


        // get expanded polynomials
        auto b = expandPolynomial<COEFFICIENTS - 1>(zZeros);
        auto a = expandPolynomial<COEFFICIENTS - 1>(zPoles);

        // scale the gain properly
        switch (Type){
            case LOWPASS:{
                // Eval at DC
                auto freqResp = evaluateFrequencyResponse<COEFFICIENTS - 1>(b, a, 0, Ts);
                auto mag = complex_abs(freqResp);
                double scale = 1 / mag;
                for (auto &coef : b) coef *= scale;
                break;
            }
            case HIGHPASS:{
                // Eval at niquest
                auto freqResp = evaluateFrequencyResponse<COEFFICIENTS - 1>(b, a, M_PI/Ts, Ts);
                auto mag = complex_abs(freqResp);
                double scale = 1 / mag; 
                for (auto &coef : b) coef *= scale;
                break;
            }
            case BANDPASS:{
                // Eval at center frequency sqrt(wh*wl)
                auto freqResp = evaluateFrequencyResponse<COEFFICIENTS - 1>(b, a, std::sqrt(wl*whp), Ts);
                auto mag = complex_abs(freqResp);
                double scale = 1/mag;
                for (auto &coef : b) coef *= scale;
                break;
            }
            case BANDSTOP:{
                // Eval at dc gain
                // auto freqResp = evaluateFrequencyResponse<COEFFICIENTS - 1>(b, a, std::sqrt(wl*whp), Ts);
                // auto mag = complex_abs(freqResp);
                // double scale = 1/mag;
                // for (auto &coef : b) coef *= scale;
                break;
            }
        }

        // store in member arrays (reverse order)
        for (size_t i = 0; i < COEFFICIENTS; ++i)
        {
            naturalResponseCoefficients[COEFFICIENTS-i - 1] = a[i];
            forcedResponseCoefficients[COEFFICIENTS-i - 1] = b[i];
        }
    }

    static constexpr size_t COEFFICIENTS = (1 + ((Type & 0b10) != 0)) * ORDER+1;

    std::array<float, COEFFICIENTS> getNaturalResponseCoefficients() const
    {
        return naturalResponseCoefficients;
    }

    std::array<float, COEFFICIENTS> getForcedResponseCoefficients() const  { return forcedResponseCoefficients; }
private:
    std::array<float, COEFFICIENTS> naturalResponseCoefficients;
    std::array<float, COEFFICIENTS> forcedResponseCoefficients;
};