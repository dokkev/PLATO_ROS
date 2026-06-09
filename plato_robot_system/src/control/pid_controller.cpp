#include "plato_robot_system/control/pid_controller.hpp"

#include <algorithm>

namespace plato_robot_system
{
namespace
{

double clamp_value(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace

PIDController::PIDController(
  const double kp_in,
  const double ki_in,
  const double kd_in,
  const double output_ramp_in,
  const double output_limit_in)
: kp(kp_in),
  ki(ki_in),
  kd(kd_in),
  output_ramp(output_ramp_in),
  output_limit(output_limit_in)
{
}

double PIDController::Compute(const double error, const double dt_sec)
{
  double sample_time = dt_sec;
  if (sample_time <= 0.0 || sample_time > 0.5) {
    sample_time = 1.0e-3;
  }

  const double proportional = kp * error;
  double integral = previous_integral_ + ki * sample_time * 0.5 * (error + previous_error_);
  integral = clamp_value(integral, -output_limit, output_limit);
  const double derivative = kd * (error - previous_error_) / sample_time;

  double output = proportional + integral + derivative;
  output = clamp_value(output, -output_limit, output_limit);

  if (output_ramp > 0.0) {
    const double output_rate = (output - previous_output_) / sample_time;
    if (output_rate > output_ramp) {
      output = previous_output_ + output_ramp * sample_time;
    } else if (output_rate < -output_ramp) {
      output = previous_output_ - output_ramp * sample_time;
    }
  }

  previous_integral_ = integral;
  previous_output_ = output;
  previous_error_ = error;
  return output;
}

void PIDController::Reset()
{
  previous_integral_ = 0.0;
  previous_output_ = 0.0;
  previous_error_ = 0.0;
}

}  // namespace plato_robot_system
