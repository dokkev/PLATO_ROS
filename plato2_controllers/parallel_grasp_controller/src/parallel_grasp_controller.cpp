#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

// ============================================================================
// Constructor & Geometric Computations
// ============================================================================

ParallelGraspController::ParallelGraspController() {
  // Verify geometric feasibility: lateral offset must allow parallel alignment
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_geom(double q3) const {
  // Geometric constraint to maintain parallel fingertips
  // Based on: cos(q5) = cos(q3) - w/L
  const double cos_q5 = std::clamp(std::cos(q3) - w / L, -1.0, 1.0);
  return std::max(q5_min, std::acos(cos_q5));
}

double ParallelGraspController::compute_delta_q5(double q3) const {
  // Offset between q5 and q3 to maintain parallel constraint
  return compute_q5_geom(q3) - q3;
}

// ============================================================================
// Main Update Function
// ============================================================================

void ParallelGraspController::update(const std::array<double, 3>& commands,
                                      const std::vector<double>& current_positions,
                                      double current_force) {
  // Copy current joint positions with zero-padding for safety
  const size_t num_joints = padded_positions_.size();
  const size_t available = std::min(num_joints, current_positions.size());

  for (size_t i = 0; i < available; ++i) {
    padded_positions_[i] = current_positions[i];
  }
  for (size_t i = available; i < num_joints; ++i) {
    padded_positions_[i] = 0.0;
  }

  const double u_cmd   = commands[0];  // Grasp distance or velocity
  const double phi     = commands[1];  // Contact angle
  const double f_cmd   = commands[2];  // Desired force

  // ========================================================================
  // State Machine Transitions
  // ========================================================================

  if (state_ == State::kInit) {
    // Initialization phase: smooth transition to neutral pose
    if (init_progress_ == 0.0) {
      init_start_ = padded_positions_;
      const double q5_neutral = compute_q5_geom(neutral);
      init_target_ = {neutral, neutral, neutral, -neutral,
                      q5_neutral, -q5_neutral, joint6, joint7};
    }

    init_progress_ = std::min(1.0, init_progress_ + init_rate_);

    for (size_t i = 0; i < num_joints; ++i) {
      joint_commands_[i] = util::Smooth(init_start_[i], init_target_[i], init_progress_);
    }

    if (init_progress_ >= 1.0) {
      state_ = State::kMotion;
      u_internal_ = 0.5;  // Initialize to mid-position
    }
    return;
  }

  // ========================================================================
  // Control Logic
  // ========================================================================

  const double u_effective = update_f(f_cmd, current_force, u_cmd);  // Handle force control & state transitions

  update_u(u_effective, current_positions);  // Control proximal joints (q3, q5)
  update_phi(phi, current_positions);        // Control distal joints (q4, q6)

  // Static joints: keep at fixed target positions
  joint_commands_[0] = util::Smooth(joint_commands_[0], neutral, alpha_);  // q1
  joint_commands_[1] = util::Smooth(joint_commands_[1], neutral, alpha_);  // q2
  joint_commands_[6] = util::Smooth(joint_commands_[6], joint6, alpha_);   // q7
  joint_commands_[7] = util::Smooth(joint_commands_[7], joint7, alpha_);   // q8
}

// ============================================================================
// Grasp Distance Control (u parameter)
// ============================================================================

void ParallelGraspController::update_u(double u_cmd, const std::vector<double>& /* current_positions */) {
  const double u = std::clamp(u_cmd, 0.0, 1.0);
  const double q5_neutral = compute_q5_geom(neutral);

  double q3_target, q5_target;

  if (u > midpoint) {
    // Opening mode (u > 0.5): keep q3 neutral, reduce q5 to open fingertips
    q3_target = neutral;
    const double open_ratio = (u - midpoint) * 2.0;  // Map [0.5,1.0] -> [0,1]
    q5_target = std::max(q5_min, q5_neutral - open_ratio * (q5_neutral - q5_min));
  } else {
    // Closing mode (u <= 0.5): move q3 from qmax to qmin, q5 follows geometry
    const double close_ratio = (midpoint - u) * 2.0;  // Map [0,0.5] -> [1,0]
    q3_target = std::clamp(qmax - close_ratio * (qmax - qmin),
                           std::min(qmin, qmax), std::max(qmin, qmax));
    q5_target = std::max(q5_min, q3_target + compute_delta_q5(q3_target));
  }

  // Apply smoothed commands to proximal joints
  joint_commands_[2] = util::Smooth(joint_commands_[2], q3_target, alpha_);  // q3 (joint3)
  joint_commands_[4] = util::Smooth(joint_commands_[4], q5_target, alpha_);  // q5 (joint5)
}

// ============================================================================
// Force Control (f parameter)
// ============================================================================

double ParallelGraspController::update_f(double f_cmd, double current_force, double u_cmd) {
  const bool force_requested = (f_cmd > force_threshold);
  const bool force_detected  = (current_force > force_threshold);

  // State transitions
  if (state_ == State::kMotion && force_requested && force_detected) {
    // Transition to force control
    state_ = State::kForce;
    u_internal_ = u_cmd;  // Initialize internal u from current command
  } else if (state_ == State::kForce && !force_requested) {
    // Transition back to motion control
    state_ = State::kMotion;
  }

  // Compute effective u based on current state
  if (state_ == State::kForce) {
    // Force control mode: admittance-based regulation
    const double force_error = f_cmd - current_force;
    u_internal_ += admittance_gain * force_error;  // Adjust u based on force error
    u_internal_ = std::clamp(u_internal_, 0.0, 1.0);
    return u_internal_;
  } else {
    // Motion control mode: direct position control
    return u_cmd;
  }
}

// ============================================================================
// Contact Angle Control (phi parameter)
// ============================================================================

void ParallelGraspController::update_phi(double phi_cmd, const std::vector<double>& current_positions) {
  // Read current proximal joint angles for absolute angle control
  const double q3_current = current_positions.size() > 2 ? current_positions[2] : 0.0;
  const double q5_current = current_positions.size() > 4 ? current_positions[4] : 0.0;
  const double phi = std::clamp(phi_cmd, 0.0, 1.0);

  // Compute distal joint targets for pinching action
  // phi = 0: parallel grasp (mirrored angles)
  // phi = 1: maximum flexion (opposite directions for pinching)
  const double q4_target = -q3_current - phi * max_flexion_angle;  // Flexes negative
  const double q6_target = -q5_current + phi * max_flexion_angle;  // Flexes positive

  // Apply smoothed commands to distal joints
  joint_commands_[3] = util::Smooth(joint_commands_[3], q4_target, mirror_alpha);  // q4 (joint4)
  joint_commands_[5] = util::Smooth(joint_commands_[5], q6_target, mirror_alpha);  // q6 (joint6)
}
