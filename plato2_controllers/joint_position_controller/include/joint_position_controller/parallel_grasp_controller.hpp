#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

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
  static constexpr double qmin = -0.524;  // Joint lower limit (rad) ≈ -60°
  static constexpr double qmax =  0.524;  // Joint upper limit (rad) ≈ +60°

  /**
   * @brief Constructor with feasibility check
   * @throws std::runtime_error if |w| > 2L (x-alignment impossible)
   */
  ParallelGraspController();

  /**
   * @brief Map normalized command to joint positions
   * @param u_cmd Normalized command [0,1] → q3 ∈ [qmin, qmax]
   * @param current_positions Current joint positions (8 elements), used to determine q4, q6
   * @return 8-element joint position vector (indices 2-5 contain q3,q4,q5,q6)
   */
  const std::vector<double>& get_commands(double u_cmd, const std::vector<double>& current_positions);

  /**
   * @brief Set interpolation factor for trajectory smoothing
   * @param alpha Interpolation factor [0,1], where 0=no motion, 1=instant motion, default=0.1
   */
  void set_interpolation_alpha(double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }

private:
  /**
   * @brief Compute q5 from q3 for smooth opposite motion
   * @param q3 Joint angle (rad)
   * @return q5 angle (rad)
   */
  double compute_q5_smooth(double q3);

  // Internal state
  double u_ = 0.5;
  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);
  double alpha_ = 0.1;  // Interpolation factor (10% per step)
  bool initialized_ = false;

  // State for smooth opening transition
  mutable double prev_q3_ = 0.0;
  mutable double prev_q5_ = 0.0;
};
