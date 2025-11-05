#include "joint_position_controller/parallel_grasp_controller.hpp"

ParallelGraspController::ParallelGraspController() {
  // Feasibility check: x-alignment requires |w| ≤ 2L
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_smooth(double q3) {
  if (q3 < 0.0) {
    // For closing (q3 < 0): enforce exact x-alignment (zero horizontal displacement)
    // Geometric constraint: cos(q5) = cos(q3) - w/L
    const double cos_q5 = std::cos(q3) - w / L;

    // Clamp to valid range [-1, 1]
    const double cos_q5_clamped = std::clamp(cos_q5, -1.0, 1.0);

    // For q3 < 0, we want q5 > 0 (opposite direction)
    // Since acos returns [0, π], this gives us the positive solution
    return std::acos(cos_q5_clamped);
  } else {
    // For opening (q3 >= 0): use smooth linear scaling
    const double scale_factor = -(1.0 + std::abs(w) / L);
    return q3 * scale_factor;
  }
}

const std::vector<double>& ParallelGraspController::get_commands(
    double u_cmd,
    const std::vector<double>& current_positions) {

  // Map u ∈ [0,1] → q3 ∈ [qmin, qmax]
  u_ = std::clamp(u_cmd, 0.0, 1.0);
  const double q3 = qmin + u_ * (qmax - qmin);

  // Build 8-element joint command vector
  joint_commands_[0] = 0.0;
  joint_commands_[1] = 0.0;
  joint_commands_[2] = q3;
  joint_commands_[3] = -current_positions[2];  // q4 mirrors current q3
  joint_commands_[4] = compute_q5_smooth(q3);
  joint_commands_[5] = -current_positions[4];  // q6 mirrors current q5
  joint_commands_[6] = 0.785;
  joint_commands_[7] = 1.5708;

  return joint_commands_;
}
