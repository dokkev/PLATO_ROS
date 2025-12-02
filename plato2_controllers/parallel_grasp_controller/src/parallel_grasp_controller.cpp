#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

namespace
{
constexpr double kQ5Min = 1e-3;
}

ParallelGraspController::ParallelGraspController() {
  // Feasibility check: x-alignment requires |w| ≤ 2L
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_geom(double q3) const {
  const double arg = std::clamp(std::cos(q3) - w / L, -1.0, 1.0);
  return std::max(kQ5Min, std::acos(arg));
}

double ParallelGraspController::compute_delta_q5(double q3) const {
  return compute_q5_geom(q3) - q3;  // delta to add to q3
}

void ParallelGraspController::update(double u_cmd, const std::vector<double>& current_positions) {
  const size_t n = padded_positions_.size();
  for (size_t i = 0; i < n; ++i) {
    padded_positions_[i] = (i < current_positions.size()) ? current_positions[i] : 0.0;
  }

  switch (state_) {
    case State::Init: {
      if (init_progress_ == 0.0) {
        init_start_ = padded_positions_;
        const double q3_neutral = 0.0;
        const double q5_neutral = compute_q5_geom(q3_neutral);
        init_target_[0] = 0.0;
        init_target_[1] = 0.0;
        init_target_[2] = q3_neutral;
        init_target_[3] = -q3_neutral;
        init_target_[4] = q5_neutral;
        init_target_[5] = -q5_neutral;
        init_target_[6] = 0.785;
        init_target_[7] = 1.5708;
      }
      init_progress_ = std::min(1.0, init_progress_ + init_rate_);
      for (size_t i = 0; i < n; ++i) {
        joint_commands_[i] = util::Smooth(init_start_[i], init_target_[i], init_progress_);
      }
      if (init_progress_ >= 1.0) {
        state_ = State::Active;
      }
      return;
    }
    case State::Active: {
      const double u = std::clamp(u_cmd, 0.0, 1.0);
      const double close_ratio = std::clamp((0.5 - u) * 2.0, 0.0, 1.0);
      const double open_ratio = std::clamp((u - 0.5) * 2.0, 0.0, 1.0);

      double q3_target = 0.0;
      double q5_target = compute_q5_geom(0.0);

      if (open_ratio > 0.0) {
        q5_target = std::max(kQ5Min, q5_target - open_ratio * (q5_target - kQ5Min));
      } else {
        q3_target = std::clamp(
          qmax - close_ratio * (qmax - qmin),
          std::min(qmin, qmax), std::max(qmin, qmax));
        q5_target = std::max(kQ5Min, q3_target + compute_delta_q5(q3_target));
      }

      joint_commands_[0] = util::Smooth(joint_commands_[0], 0.0, alpha_);
      joint_commands_[1] = util::Smooth(joint_commands_[1], 0.0, alpha_);
      joint_commands_[2] = util::Smooth(joint_commands_[2], q3_target, alpha_);
      joint_commands_[4] = util::Smooth(joint_commands_[4], q5_target, alpha_);
      joint_commands_[6] = util::Smooth(joint_commands_[6], 0.785, alpha_);
      joint_commands_[7] = util::Smooth(joint_commands_[7], 1.5708, alpha_);

      const double q4_target = -padded_positions_[2];
      const double q6_target = -padded_positions_[4];
      joint_commands_[3] = util::Smooth(joint_commands_[3], q4_target, 0.9);
      joint_commands_[5] = util::Smooth(joint_commands_[5], q6_target, 0.9);
      return;
    }
  }
}
