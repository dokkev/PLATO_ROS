#pragma once

#include <vector>
#include <algorithm>
#include "plato_utils/interpolation.hpp"

struct ImpedanceCommand {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> stiffness;
  std::vector<double> damping;
  std::vector<double> effort_ff;
};

class JointPositionController {
public:
  enum class State { kMotion, kForce };

  JointPositionController(double interp_alpha = 0.1,
                          double kp_force = 0.5,
                          double ki_force = 0.1,
                          double i_limit = 5.0);

  void setGains(const std::vector<double>& stiffness, const std::vector<double>& damping);

  ImpedanceCommand process(const std::vector<double>& position_cmd,
                           double desired_force,
                           double measured_force,
                           bool contact_estimator);

  State getState() const { return state_; }

private:
  std::vector<double> smoothed_position_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  double interp_alpha_{0.1};
  // Force control (PI)
  double kp_force_{0.5};
  double ki_force_{0.1};
  double i_limit_{5.0};
  double force_i_{0.0};
  State state_{State::kMotion};
};
