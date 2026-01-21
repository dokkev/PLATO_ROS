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
  JointPositionController(double interp_alpha = 0.1);

  void setGains(const std::vector<double>& stiffness, const std::vector<double>& damping);

  ImpedanceCommand process(const std::vector<double>& position_cmd);

private:
  std::vector<double> smoothed_position_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  double interp_alpha_{0.1};
};
