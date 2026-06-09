#ifndef PLATO_ROBOT_SYSTEM__CONTROL__PID_CONTROLLER_HPP_
#define PLATO_ROBOT_SYSTEM__CONTROL__PID_CONTROLLER_HPP_

#include <limits>

namespace plato_robot_system
{

class PIDController
{
public:
  PIDController() = default;
  PIDController(double kp, double ki, double kd, double output_ramp, double output_limit);

  double Compute(double error, double dt_sec);
  void Reset();

  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double output_ramp{0.0};
  double output_limit{std::numeric_limits<double>::max()};

private:
  double previous_error_{0.0};
  double previous_output_{0.0};
  double previous_integral_{0.0};
};

}  // namespace plato_robot_system

#endif  // PLATO_ROBOT_SYSTEM__CONTROL__PID_CONTROLLER_HPP_
