#include "parallel_grasp_controller/pid.hpp"

// Constrain value within bounds
template<typename T>
inline T constrain(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)         // output supply limit     [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
}

// PID controller function
float PIDController::operator() (float error, float dt){
    // Use provided dt parameter (time step in seconds)
    float Ts = dt;
    // quick fix for invalid time steps
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    // antiwindup - limit the output
    integral = constrain(integral, -limit, limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = constrain(output, -limit, limit);

    // if output ramp defined
    if(output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    return output;
}

void PIDController::reset(){
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}