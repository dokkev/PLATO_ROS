#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

ParallelGraspController::ParallelGraspController() {
  // Feasibility check: x-alignment requires |w| ≤ 2L
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_smooth(double q3) {
  if (q3 < 0.0) {
    // Closing: enforce exact x-alignment (geometric constraint)
    const double cos_q5 = std::cos(q3) - w / L;
    const double cos_q5_clamped = std::clamp(cos_q5, -1.0, 1.0);
    const double q5_geometric = std::acos(cos_q5_clamped);

    // Store for opening transition
    prev_q3_ = q3;
    prev_q5_ = q5_geometric;

    return q5_geometric;
  } else {
    // Opening: linearly interpolate from last closing q5 to 0 (fully open)
    // Map q3 ∈ [0, qmax] → q5 ∈ [prev_q5_, 0]
    const double t = q3 / qmax;  // normalized [0, 1]
    return prev_q5_ * (1.0 - t);  // linear blend to zero
  }
}

const std::vector<double>& ParallelGraspController::get_commands(
    double u_cmd,
    const std::vector<double>& current_positions) {

  // Map u ∈ [0,1] → q3 ∈ [qmin, qmax]
  u_ = std::clamp(u_cmd, 0.0, 1.0);
  const double q3_target = qmin + u_ * (qmax - qmin);
  const double q5_target = compute_q5_smooth(q3_target);

  // Initialize on first call
  if (!initialized_) {
    joint_commands_[0] = 0.0;
    joint_commands_[1] = 0.0;
    joint_commands_[2] = q3_target;
    joint_commands_[4] = q5_target;
    joint_commands_[6] = 0.785;
    joint_commands_[7] = 1.5708;
    initialized_ = true;
  }

  // Interpolate all joints except [3] and [5]
  joint_commands_[0] += alpha_ * (0.0 - joint_commands_[0]);
  joint_commands_[1] += alpha_ * (0.0 - joint_commands_[1]);
  joint_commands_[2] += alpha_ * (q3_target - joint_commands_[2]);
  joint_commands_[4] += alpha_ * (q5_target - joint_commands_[4]);
  joint_commands_[6] += alpha_ * (0.785 - joint_commands_[6]);
  joint_commands_[7] += alpha_ * (1.5708 - joint_commands_[7]);

  // Mirror joints (fast response, no interpolation)
  joint_commands_[3] = -current_positions[2];  // q4 mirrors current q3
  joint_commands_[5] = -current_positions[4];  // q6 mirrors current q5

  return joint_commands_;
}
