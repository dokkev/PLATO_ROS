#include "joint_position_controller/parallel_grasp_controller.hpp"

ParallelGraspController::ParallelGraspController() {
  // Feasibility check: x-alignment requires |w| ≤ 2L
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_delta(double q3) {
  // Enforce x-alignment: x5 = x3  =>  cos(q5) = cos(q3) - w/L
  // We choose the opposite-motion branch so dq5/dq3 < 0 (fingers move in opposite directions)

  const double Araw = std::cos(q3) - w / L;

  // Check feasibility (may occur if q3 is at extreme angles)
  // Fall back to clamped value but exact x-alignment may not be achievable
  const double A = std::clamp(Araw, -1.0, 1.0);

  // Opposite-motion branch: use negative sgn(sin(q3))
  const double y = -sgn(std::sin(q3)) * std::sqrt(std::max(0.0, 1.0 - A * A));
  const double q5 = std::atan2(y, A);

  // Return wrapped angle difference: Δq = q5 - q3
  return wrap_pi(q5 - q3);
}

const std::vector<double>& ParallelGraspController::get_commands(double u_cmd) {
  // Map u ∈ [0,1] → q3 ∈ [qmin, qmax]
  const double u = std::clamp(u_cmd, 0.0, 1.0);
  const double q3 = qmin + u * (qmax - qmin);

  // Compute q5 using geometric constraint
  const double dq = compute_q5_delta(q3);
  double q5 = q3 + dq;

  // Mirror for opposite finger (joints 4 and 6) - parallelogram assumption
  const double q4 = -q3;
  const double q6 = -q5;

  // Update internal state
  last_u_ = u;

  // Build 8-element joint command vector (indices 2-5 contain grasp joints)
  joint_commands_.assign(8, 0.0);
  joint_commands_[2] = q3;
  joint_commands_[3] = q4;
  joint_commands_[4] = q5;
  joint_commands_[5] = q6;

  return joint_commands_;
}
