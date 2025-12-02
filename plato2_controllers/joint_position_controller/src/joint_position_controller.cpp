#include "joint_position_controller/joint_position_controller.hpp"

JointPositionController::JointPositionController(double interp_alpha,
                                                 double kp_force,
                                                 double ki_force,
                                                 double i_limit)
    : interp_alpha_(interp_alpha),
      kp_force_(kp_force),
      ki_force_(ki_force),
      i_limit_(i_limit)
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

ImpedanceCommand JointPositionController::process(const std::vector<double>& position_cmd,
                                                  double desired_force,
                                                  double measured_force,
                                                  bool contact_estimator) {
  const size_t n = std::max(smoothed_position_.size(), position_cmd.size());
  if (smoothed_position_.size() < n) {
    smoothed_position_.resize(n, 0.0);
  }

  for (size_t i = 0; i < n; ++i) {
    const double target = (i < position_cmd.size()) ? position_cmd[i] : 0.0;
    smoothed_position_[i] = util::Smooth(smoothed_position_[i], target, interp_alpha_);
  }

  // State machine: force mode when both desired and measured are positive
  const bool force_requested = desired_force > 0.0;
  const bool force_detected = measured_force > 0.0;
  const bool has_contact = force_detected || contact_estimator;
  state_ = (force_requested && has_contact) ? State::kForce : State::kMotion;

  // PI force control applied only to joints 3-6 (indices 2-5)
  std::vector<double> effort(n, 0.0);
  if (state_ == State::kForce) {
    const double error = desired_force - measured_force;
    force_i_ += error;
    force_i_ = std::clamp(force_i_, -i_limit_, i_limit_);
    const double effort_cmd = kp_force_ * error + ki_force_ * force_i_;
    const size_t start = 2;
    const size_t end = std::min(static_cast<size_t>(6), n);
    static const double signs[4] = {-1.0, -1.0, 1.0, 1.0};  // joints 3,4 negative; 5,6 positive
    for (size_t idx = start; idx < end; ++idx) {
      effort[idx] = effort_cmd * signs[idx - start];
    }
  } else {
    force_i_ = 0.0;
  }

  ImpedanceCommand cmd;
  cmd.position = smoothed_position_;
  cmd.velocity = std::vector<double>(n, 0.0);
  cmd.stiffness = stiffness_;
  cmd.damping = damping_;
  cmd.effort_ff = effort;
  return cmd;
}
