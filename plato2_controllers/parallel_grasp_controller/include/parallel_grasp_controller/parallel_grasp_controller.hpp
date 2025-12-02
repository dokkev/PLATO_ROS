#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <plato_utils/interpolation.hpp>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif

#if __cplusplus >= 202002L
#include <numbers>  // C++20
#endif

/**
 * @brief Simplified parallel grasp controller
 *
 * Maps normalized command u ∈ [0,1] directly to q3 ∈ [qmin, qmax]
 * Computes q5 using geometric constraint to maintain parallel grasp
 */
class ParallelGraspController {
public:
  // Geometric parameters
  static constexpr double L    = 0.06;     // Tip radius (m)
  static constexpr double w    = 0.022;   // Lateral offset (m)
  static constexpr double qmin =  -45.0 * M_PI / 180.0; // Joint lower limit (rad)
  static constexpr double qmax =   0.0;     // Joint upper limit (rad); u=0 ⇒ fingertips meet
  static constexpr double q_default = -0.35; // Default starting q3 (rad), clamped to limits

  // Control parameters
  static constexpr double q5_min = 1e-3;                  // Minimum q5 angle (rad)
  static constexpr double midpoint = 0.5;                 // U midpoint between closing/opening
  static constexpr double neutral = 0.0;                  // Neutral angle (rad)
  static constexpr double joint6 = 0.785;                 // π/4 rad
  static constexpr double joint7 = 1.5708;                // π/2 rad
  static constexpr double mirror_alpha = 0.9;             // Smoothing factor for mirrored joints
  static constexpr double max_flexion_angle = 0.785;      // π/4 rad (~45°) maximum flexion

  // Force control parameters
  static constexpr double force_threshold = 0.1;          // Minimum force to activate force control (N)
  static constexpr double admittance_gain = 0.01;         // Admittance control gain

  /**
   * @brief Constructor with feasibility check
   * @throws std::runtime_error if |w| > 2L (x-alignment impossible)
   */
  ParallelGraspController();

  /**
   * @brief Update internal command vector based on input commands and current joints
   * @param commands 3-element array: [u, phi, f] where:
   *        - u: Normalized grasp distance [0,1] (0=closed, 1=open) or velocity in force mode
   *        - phi: Normalized contact angle [0,1] (0=parallel, 1=max flexion)
   *        - f: Desired force [N] (0=motion control, >0=force control)
   * @param current_positions Current joint positions (used for absolute angle control)
   * @param current_force Current measured force (N)
   */
  void update(const std::array<double, 3>& commands,
              const std::vector<double>& current_positions,
              double current_force = 0.0);

  /**
   * @brief Update grasp distance command (u parameter)
   * @param u Normalized grasp distance [0,1] (0=closed, 1=open)
   * @param current_positions Current joint positions
   */
  void update_u(double u, const std::vector<double>& current_positions);

  /**
   * @brief Update contact angle command (phi parameter)
   * @param phi Normalized contact angle [0,1] (0=parallel, 1=max flexion)
   * @param current_positions Current joint positions
   */
  void update_phi(double phi, const std::vector<double>& current_positions);

  /**
   * @brief Get latest joint command vector (after update)
   */
  const std::vector<double>& get_commands() const { return joint_commands_; }

  /**
   * @brief Set interpolation factor for trajectory smoothing
   * @param alpha Interpolation factor [0,1], where 0=no motion, 1=instant motion, default=0.1
   */
  void set_interpolation_alpha(double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }

private:
  /**
   * @brief Compute q5 geometric angle for a given q3 (clamped valid range)
   */
  double compute_q5_geom(double q3) const;

  /**
   * @brief Compute delta so that q5 = q3 + delta (geometric coupling)
   * @param q3 Joint angle (rad)
   * @return delta_q5 (rad)
   */
  double compute_delta_q5(double q3) const;

  // Internal state
  enum class State { kInit, kMotion, kForce };
  State state_ = State::kInit;
  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);
  double alpha_ = 0.1;  // Interpolation factor (10% per step)
  double init_progress_ = 0.0;
  double init_rate_ = 0.05;
  std::vector<double> init_start_ = std::vector<double>(8, 0.0);
  std::vector<double> init_target_ = std::vector<double>(8, 0.0);
  std::vector<double> padded_positions_ = std::vector<double>(8, 0.0);

  // Force control state
  double u_internal_ = 0.5;  // Internal u value for force control
};
