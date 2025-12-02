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

  /**
   * @brief Constructor with feasibility check
   * @throws std::runtime_error if |w| > 2L (x-alignment impossible)
   */
  ParallelGraspController();

  /**
   * @brief Update internal command vector based on input u and current joints
   * @param u_cmd Normalized command [0,1]
   * @param current_positions Current joint positions (mirroring uses this)
   */
  void update(double u_cmd, const std::vector<double>& current_positions);

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
  enum class State { Init, Active };
  State state_ = State::Init;
  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);
  double alpha_ = 0.1;  // Interpolation factor (10% per step)
  double init_progress_ = 0.0;
  double init_rate_ = 0.05;
  std::vector<double> init_start_ = std::vector<double>(8, 0.0);
  std::vector<double> init_target_ = std::vector<double>(8, 0.0);
  std::vector<double> padded_positions_ = std::vector<double>(8, 0.0);
};
