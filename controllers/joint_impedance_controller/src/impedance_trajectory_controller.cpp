#include "joint_impedance_controller/impedance_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kFallbackFilterAlpha = 0.1;
constexpr double kGoalPositionTolerance = 1e-5;

double sanitize_filter_alpha(double alpha)
{
  return std::isfinite(alpha) ? std::clamp(alpha, 0.0, 1.0) : kFallbackFilterAlpha;
}
}

ImpedanceTrajectoryController::ImpedanceTrajectoryController(std::size_t dof)
    : dof_(dof == 0 ? 8 : dof) {
  stiffness_.assign(dof_, 0.0);
  damping_.assign(dof_, 0.0);
  measured_position_.assign(dof_, 0.0);
  measured_velocity_.assign(dof_, 0.0);
  desired_position_.assign(dof_, 0.0);
  desired_velocity_.assign(dof_, 0.0);
  effort_ff_.assign(dof_, 0.0);
  goal_position_.assign(dof_, 0.0);
  goal_effort_ff_.assign(dof_, 0.0);
}

void ImpedanceTrajectoryController::setGains(const std::vector<double>& stiffness,
                                       const std::vector<double>& damping) {
  stiffness_ = sanitizeToDof(stiffness);
  damping_ = sanitizeToDof(damping);
}

void ImpedanceTrajectoryController::setMeasuredState(const std::vector<double>& position,
                                               const std::vector<double>& velocity) {
  measured_position_ = sanitizeToDof(position);
  measured_velocity_ = sanitizeToDof(velocity);
  has_measured_state_ = true;

  if (!has_desired_state_) {
    desired_position_ = measured_position_;
    desired_velocity_.assign(dof_, 0.0);
    effort_ff_.assign(dof_, 0.0);
    has_desired_state_ = true;
  }
}

void ImpedanceTrajectoryController::setGoal(const std::vector<double>& target_position,
                                      double filter_alpha,
                                      const std::vector<double>& effort_ff) {
  initializeDesiredFromMeasurementIfNeeded();

  goal_position_ = desired_position_;
  const std::size_t position_len = std::min(dof_, target_position.size());
  std::copy_n(target_position.begin(), position_len, goal_position_.begin());
  goal_effort_ff_.assign(dof_, 0.0);
  const std::size_t effort_len = std::min(dof_, effort_ff.size());
  std::copy_n(effort_ff.begin(), effort_len, goal_effort_ff_.begin());

  filter_alpha_ = sanitize_filter_alpha(filter_alpha);
  mode_ = ExecutionMode::Executing;
}

void ImpedanceTrajectoryController::holdPosition() {
  initializeDesiredFromMeasurementIfNeeded();

  const auto& hold_source = has_measured_state_ ? measured_position_ : desired_position_;
  desired_position_ = hold_source;
  goal_position_ = hold_source;
  desired_velocity_.assign(dof_, 0.0);
  effort_ff_.assign(dof_, 0.0);
  goal_effort_ff_.assign(dof_, 0.0);

  mode_ = ExecutionMode::Holding;
}

ImpedanceCommand ImpedanceTrajectoryController::update(double dt_sec) {
  initializeDesiredFromMeasurementIfNeeded();

  if (dt_sec < 0.0) {
    dt_sec = 0.0;
  }

  if (mode_ == ExecutionMode::Executing) {
    const double alpha = filter_alpha_;
    bool reached_goal = true;

    for (std::size_t i = 0; i < dof_; ++i) {
      const double previous_position = desired_position_[i];
      const double error = goal_position_[i] - previous_position;
      desired_position_[i] = previous_position + alpha * error;
      desired_velocity_[i] = dt_sec > 0.0 ?
        (desired_position_[i] - previous_position) / dt_sec : 0.0;

      if (std::abs(goal_position_[i] - desired_position_[i]) > kGoalPositionTolerance) {
        reached_goal = false;
      }
    }

    effort_ff_ = goal_effort_ff_;

    if (reached_goal) {
      mode_ = ExecutionMode::Holding;
      desired_position_ = goal_position_;
      desired_velocity_.assign(dof_, 0.0);
    }
  } else if (mode_ == ExecutionMode::Idle && has_measured_state_) {
    desired_position_ = measured_position_;
    desired_velocity_.assign(dof_, 0.0);
    effort_ff_.assign(dof_, 0.0);
  }

  ImpedanceCommand cmd;
  cmd.position = desired_position_;
  cmd.velocity = desired_velocity_;
  cmd.stiffness = stiffness_;
  cmd.damping = damping_;
  cmd.effort_ff = effort_ff_;
  return cmd;
}

std::vector<double> ImpedanceTrajectoryController::sanitizeToDof(const std::vector<double>& in,
                                                           double fallback) const {
  std::vector<double> out(dof_, fallback);
  const std::size_t copy_len = std::min(dof_, in.size());
  std::copy_n(in.begin(), copy_len, out.begin());
  return out;
}

void ImpedanceTrajectoryController::initializeDesiredFromMeasurementIfNeeded() {
  if (has_desired_state_) {
    return;
  }

  if (has_measured_state_) {
    desired_position_ = measured_position_;
  } else {
    desired_position_.assign(dof_, 0.0);
  }

  desired_velocity_.assign(dof_, 0.0);
  effort_ff_.assign(dof_, 0.0);
  has_desired_state_ = true;
}
