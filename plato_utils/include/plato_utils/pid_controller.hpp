#ifndef PLATO_UTILS_PID_CONTROLLER_HPP
#define PLATO_UTILS_PID_CONTROLLER_HPP

#include <algorithm>

namespace plato_utils {

/**
 * @brief PID controller class for general control applications
 *
 * Implements a discrete-time PID controller with anti-windup,
 * output limiting, and output rate limiting (ramping).
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     *
     * @param P Proportional gain
     * @param I Integral gain
     * @param D Derivative gain
     * @param ramp Maximum rate of change of output (0 = no limit)
     * @param limit Maximum absolute value of output
     */
    PIDController(float P, float I, float D, float ramp, float limit);

    ~PIDController() = default;

    /**
     * @brief Compute PID output for given error
     *
     * @param error Current error value (setpoint - measured)
     * @param dt Time step in seconds
     * @return float Control output
     */
    float operator()(float error, float dt);

    /**
     * @brief Reset internal state
     */
    void reset();

    // Public gains for easy tuning
    float P;  ///< Proportional gain
    float I;  ///< Integral gain
    float D;  ///< Derivative gain
    float output_ramp;  ///< Maximum rate of change [units/second]
    float limit;  ///< Maximum absolute output value

private:
    float error_prev;     ///< Previous error value
    float output_prev;    ///< Previous output value
    float integral_prev;  ///< Previous integral term value
};

}  // namespace plato_utils

#endif  // PLATO_UTILS_PID_CONTROLLER_HPP
