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
  static constexpr double qmax =  1.0472;  // Joint upper limit (rad) ≈ +60°

  /**
   * @brief Constructor with feasibility check
   * @throws std::runtime_error if |w| > 2L (x-alignment impossible)
   */
  ParallelGraspController();

  /**
   * @brief Map normalized command to joint positions
   * @param u_cmd Normalized command [0,1] → q3 ∈ [qmin, qmax]
   * @return 8-element joint position vector (indices 2-5 contain q3,q4,q5,q6)
   */
  const std::vector<double>& get_commands(double u_cmd);

private:
  /**
   * @brief Compute q5 delta from q3 using geometric constraint
   *
   * Enforces x-alignment: x5 = x3  =>  cos(q5) = cos(q3) - w/L
   * Uses opposite-motion branch: dq5/dq3 < 0 (fingers move in opposite directions)
   *
   * @param q3 Joint angle (rad)
   * @return Δq = q5 - q3 such that q5 = q3 + Δq maintains x-alignment
   */
  static double compute_q5_delta(double q3);

  /**
   * @brief Sign function
   */
  static inline double sgn(double x) { return (x >= 0.0) ? 1.0 : -1.0; }

  /**
   * @brief Wrap angle to (-π, π]
   */
  static inline double wrap_pi(double a) {
#if __cplusplus >= 202002L
    using std::numbers::pi;
    return std::remainder(a, 2.0 * pi);
#else
    return std::remainder(a, 2.0 * M_PI);
#endif
  }

  // Internal state
  double last_u_ = 0.5;
  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);
};
