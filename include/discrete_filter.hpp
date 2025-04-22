#pragma once

#include <array>
#include <cstdint>

/**
 * @brief DiscreteFilter class implements a discrete-time filter using the finite difference
 * equation.
 * @tparam SIZE The size of the filter coefficients.
 *
 * This class provides methods to filter input data using the finite difference equation and
 * maintain the internal state of the filter.
 */
template <uint8_t SIZE>
class DiscreteFilter
{
public:

    /**
     * @brief Constructor for the DiscreteFilter class.
     * @param [in] naturalResponseCoefficients The coefficients for the natural response (a).
     * @param [in] forcedResponseCoefficients The coefficients for the forced response (b).
     *
     * This constructor initializes the filter with the given coefficients and resets the filter
     * state to zero.
     */
    DiscreteFilter(
        std::array<float, SIZE> &naturalResponseCoefficients,
        std::array<float, SIZE> &forcedResponseCoefficients)
        : naturalResponseCoefficients(naturalResponseCoefficients),
          forcedResponseCoefficients(forcedResponseCoefficients)
    {
        reset();
    }

    /**
     * @brief Filters the input data using the finite difference equation.
     * @param [in] dat The input data to be filtered.
     * @return The filtered output data.
     *
     * This function implements a discrete-time filter using the finite difference equation.
     * It updates the internal state of the filter based on the input data and returns the
     * filtered output.
     *
     * \f$ y(n)=\frac{1}{a_{0}}\left[\sum_{\kappa=0}^{M} b_{\kappa}x(n-\kappa)-\sum_{k=1}^{N}
     * a_{k}y(n-k)\right]. \qquad{(2)} \f$
     *
     */
    float filterData(float dat)
    {
        for (int i = SIZE - 1; i > 0; i--)
        {
            forcedResponse[i] = forcedResponse[i - 1];
        }
        forcedResponse[0] = dat;

        float sum = 0;
        // Sum of forced response coefficients multiplied by the forced response X(n-k)
        // (previous input data)
        for (int i = 0; i < SIZE; i++)
        {
            sum += forcedResponseCoefficients[i] * forcedResponse[i];
        }
        // Sum of natural response coefficients multiplied by the natural response Y(n-k)
        // (previous output data)
        for (int i = 0; i < SIZE - 1; i++)
        {
            sum -= naturalResponseCoefficients[i + 1] * naturalResponse[i];
        }
        // Apply the 1/a_0 scaling to the output
        sum /= naturalResponseCoefficients[0];

        // Shift the natural response array to make room for the new output
        for (int i = SIZE - 1; i > 0; i--)
        {
            naturalResponse[i] = naturalResponse[i - 1];
        }

        naturalResponse[0] = sum;

        return naturalResponse[0];
    }

    /** @brief Returns the last filtered value*/
    float getLastFiltered() { return naturalResponse[0]; }

    /** @brief Resets the filter's state to zero, keeps the coefficients  */

    float reset()
    {
        // Reset the filter state to zero
        naturalResponse.fill(0.0f);
        forcedResponse.fill(0.0f);
        return 0.0f;
    }

private:
    std::array<float, SIZE> naturalResponseCoefficients;
    std::array<float, SIZE> forcedResponseCoefficients;
    std::array<float, SIZE> naturalResponse;
    std::array<float, SIZE> forcedResponse;
};
