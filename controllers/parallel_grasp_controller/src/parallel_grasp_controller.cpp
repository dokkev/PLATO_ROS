#include "parallel_grasp_controller/parallel_grasp_controller.hpp"
#include <iostream>

// ============================================================================
// Constructor & Geometric Computations
// ============================================================================

ParallelGraspController::ParallelGraspController() {
  // Verify geometric feasibility: lateral offset must allow parallel alignment
  // If |w| > 2L, the fingertips cannot physically achieve parallel orientation
  if (std::abs(w) > 2.0 * L) {
    throw std::runtime_error("parallel grasp: |w| > 2L; x-alignment impossible.");
  }
}

double ParallelGraspController::compute_q5_geom(double q3) const {
  // Geometric constraint to maintain parallel fingertips
  // Derived from kinematic analysis: cos(q5) = cos(q3) - w/L
  // where w is lateral offset between fingers and L is tip radius
  const double cos_q5 = std::clamp(std::cos(q3) - w / L, -1.0, 1.0);  // Clamp to valid cosine range
  return std::max(q5_min, std::acos(cos_q5));  // Enforce minimum angle constraint
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
                                      double measured_force) {
  // ========================================================================
  // Parse Input Commands
  // ========================================================================
  // commands[0] = u_d: grasp distance [0,1] where 0=closed, 1=open
  // commands[1] = u_phi: contact angle [0,1] where 0=parallel, 1=max flexion
  // commands[2] = f_d: desired force for force control

  const double u_d   = commands[0];   // Grasp distance or velocity
  const double u_phi = commands[1];   // Contact angle
  const double desired_force = commands[2];  // Desired force

  // ========================================================================
  // Copy & Pad Current Joint Positions
  // ========================================================================
  // Safely handle variable-length input by zero-padding to expected size (8 joints)
  const size_t num_joints = padded_positions_.size();  // Expected: 8 joints
  const size_t available = std::min(num_joints, current_positions.size());

  for (size_t i = 0; i < available; ++i) {
    padded_positions_[i] = current_positions[i];  // Copy available positions
  }
  for (size_t i = available; i < num_joints; ++i) {
    padded_positions_[i] = 0.0;  // Zero-pad missing joints
  }

  const double u_effective = std::clamp(u_d, 0.0, 1.0);

  // ========================================================================
  // Force Control State Machine
  // ========================================================================
  // Activate force control when both desired_force > 0 and measured_force > 0
  // Deactivate when u > 0.6
  const bool force_requested = desired_force > 0.0;
  const bool force_detected = measured_force > 0.0;
  const bool u_below_threshold = u_effective <= 0.6;

  // Debug: print state machine conditions
  static int print_counter = 0;
  if (++print_counter % 100 == 0) {  // Print every 100 cycles to avoid spam
    std::cout << "[STATE MACHINE] force_requested=" << force_requested
              << " force_detected=" << force_detected
              << " u_below_threshold=" << u_below_threshold
              << " (u_eff=" << u_effective << ")"
              << " desired_force=" << desired_force
              << " measured_force=" << measured_force
              << " active=" << force_control_active_ << std::endl;
  }

  if (force_requested && force_detected && u_below_threshold) {
    force_control_active_ = true;
  } else if (!u_below_threshold) {
    force_control_active_ = false;
  }

  // ========================================================================
  // Update Joint Commands
  // ========================================================================
  double u_d_final = u_effective;
  double u_phi_final = u_phi;

  if (force_control_active_) {
    // Admittance-based force control: directly modulate u_d and u_phi based on force error
    const double force_error = desired_force - measured_force;
    const double admittance_offset = admittance_gain_ * force_error;

    // Apply admittance offset to u_d (decrease u_d to close grasp when force_error > 0)
    u_d_final = std::clamp(u_effective - admittance_offset, 0.0, 0.6);

    // Update joints using adjusted u_d and original u_phi
    update_u(u_d_final, current_positions);  // Control proximal joints (q3, q5) for grasp width
    update_phi(u_phi_final, current_positions);       // Control distal joints (q4, q6) for contact angle

    // Debug print force control details
    static int force_counter = 0;
    if (++force_counter % 100 == 0) {
      std::cout << "[FORCE CTRL] force_error=" << force_error
                << " admittance_offset=" << admittance_offset
                << " admittance_gain=" << admittance_gain_ << std::endl;
    }

  } else {
    // Motion control mode
    update_u(u_d_final, current_positions);  // Control proximal joints (q3, q5) for grasp width
    update_phi(u_phi_final, current_positions);      // Control distal joints (q4, q6) for contact angle
  }

  // Debug print adjusted values
  static int adjust_counter = 0;
  if (++adjust_counter % 100 == 0) {
    std::cout << "[ADJUSTED] u_d: " << u_d << " -> " << u_d_final
              << " | u_phi: " << u_phi << " -> " << u_phi_final
              << " | active=" << force_control_active_ << std::endl;
  }

  // Static joints: maintain fixed positions
  joint_commands_[0] = neutral;  // q1 (index 0)
  joint_commands_[1] = neutral;  // q2 (index 1)
  joint_commands_[6] = joint6;   // q7 (index 6)
  joint_commands_[7] = joint7;   // q8 (index 7)
}

// ============================================================================
// Grasp Distance Control (u parameter)
// ============================================================================

void ParallelGraspController::update_u(double u_cmd, const std::vector<double>& /* current_positions */) {
  const double u = std::clamp(u_cmd, 0.0, 1.0);
  double q3_target, q5_target;

  if (u > midpoint) {
    const double ratio = (u - midpoint) * 2.0;  // Map [0.5, 1.0] -> [0, 1]
    const double q5_neutral = compute_q5_geom(neutral);

    q3_target = neutral;
    q5_target = std::max(q5_min, q5_neutral - ratio * (q5_neutral - q5_min));

  } else {
    const double ratio = (midpoint - u) * 2.0;  // Map [0.5, 0] -> [0, 1]

    q3_target = std::clamp(qmax - ratio * (qmax - qmin),
                           std::min(qmin, qmax), std::max(qmin, qmax));
    q5_target = std::max(q5_min, q3_target + compute_delta_q5(q3_target));
  }

  joint_commands_[2] = q3_target;  // q3 (index 2)
  joint_commands_[4] = q5_target;  // q5 (index 4)
}

// ============================================================================
// Contact Angle Control (phi parameter)
// ============================================================================

void ParallelGraspController::update_phi(double phi_cmd, const std::vector<double>& current_positions) {
  // Read current proximal joint angles (q3, q5) for absolute angle control
  const double q3_current = current_positions.size() > 2 ? current_positions[2] : 0.0;
  const double q5_current = current_positions.size() > 4 ? current_positions[4] : 0.0;
  const double phi = std::clamp(phi_cmd, 0.0, 1.0);

  // ========================================================================
  // Compute Distal Joint Targets
  // ========================================================================
  // Base mirroring: q4 = -q3, q6 = -q5 (maintains parallel orientation)
  // Additional flexion: add/subtract phi * max_flexion_angle for pinching
  //
  // q4 flexes negative (toward palm), q6 flexes positive (toward palm)
  const double q4_target = -q3_current - phi * max_flexion_angle;  // Mirror q3 + flex inward
  const double q6_target = -q5_current + phi * max_flexion_angle;  // Mirror q5 + flex inward

  joint_commands_[3] = q4_target;  // q4 (index 3)
  joint_commands_[5] = q6_target;  // q6 (index 5)
}
