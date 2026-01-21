#include "joint_position_controller/joint_position_controller.hpp"

JointPositionController::JointPositionController(double interp_alpha)
    : interp_alpha_(interp_alpha)
{
  smoothed_position_.assign(8, 0.0);
  stiffness_.assign(8, 0.0);
  damping_.assign(8, 0.0);
}

void JointPositionController::setGains(const std::vector<double>& stiffness,
                                       const std::vector<double>& damping) {
  stiffness_ = stiffness;
  damping_ = damping;
  if (stiffness_.size() < 8) stiffness_.resize(8, 0.0);
  if (damping_.size() < 8) damping_.resize(8, 0.0);
  if (stiffness_.size() > 8) stiffness_.resize(8);
  if (damping_.size() > 8) damping_.resize(8);
}

ImpedanceCommand JointPositionController::process(const std::vector<double>& position_cmd) {
  const size_t n = std::max(smoothed_position_.size(), position_cmd.size());
  if (smoothed_position_.size() < n) {
    smoothed_position_.resize(n, 0.0);
  }

  for (size_t i = 0; i < n; ++i) {
    const double target = (i < position_cmd.size()) ? position_cmd[i] : 0.0;
    smoothed_position_[i] = util::Smooth(smoothed_position_[i], target, interp_alpha_);
  }

  ImpedanceCommand cmd;
  cmd.position = smoothed_position_;
  cmd.velocity = std::vector<double>(n, 0.0);
  cmd.stiffness = stiffness_;
  cmd.damping = damping_;
  cmd.effort_ff = std::vector<double>(n, 0.0);
  return cmd;
}
