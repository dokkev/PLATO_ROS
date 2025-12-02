#include "plato_utils/pid_controller.hpp"
#include <algorithm>
#include <cmath>

namespace plato_utils {

namespace {
// Constrain value within bounds
template<typename T>
inline T constrain(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}
}  // anonymous namespace

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)
    , limit(limit)
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
}

float PIDController::operator()(float error, float dt) {
    // Validate time step
    float Ts = dt;
    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;  // Default to 1ms if invalid
    }

    // Proportional term: u_p = P * e(k)
    float proportional = P * error;

    // Integral term using Tustin (trapezoidal) integration
    // u_i(k) = u_i(k-1) + I*Ts/2*(e(k) + e(k-1))
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);

    // Anti-windup: limit integral term
    integral = constrain(integral, -limit, limit);

    // Derivative term: u_d = D * (e(k) - e(k-1)) / Ts
    float derivative = D * (error - error_prev) / Ts;

    // Compute total output
    float output = proportional + integral + derivative;

    // Apply output limits
    output = constrain(output, -limit, limit);

    // Apply output rate limiting (if enabled)
    if (output_ramp > 0.0f) {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    // Save state for next iteration
    integral_prev = integral;
    output_prev = output;
    error_prev = error;

    return output;
}

void PIDController::reset() {
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}

}  // namespace plato_utils
